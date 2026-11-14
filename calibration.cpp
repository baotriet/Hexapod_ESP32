#include "calibration.h"
#include "HexCfg.h"
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <atomic>
#include "serialOutput.h"

namespace
{
constexpr short MAX_OFFSET = 300; // Tenths of a degree.
constexpr uint32_t MAGIC = 0x4843414c;
struct Record
{
  uint32_t magic;
  uint16_t version;
  short offsets[NUM_OF_LEGS][NUM_SERVOS_PER_LEG];
  uint32_t checksum;
};

enum class Request { NONE, STATUS, ENTER, ZERO, MOVE, SAVE, EXIT };
enum class State { IDLE, REQUESTED, WORKING, DONE };
std::atomic<State> state{State::IDLE};
std::atomic<bool> active{false};
std::atomic<Request> request{Request::NONE};
byte requestLeg, requestJoint;
short requestOffset;
short offsets[NUM_OF_LEGS][NUM_SERVOS_PER_LEG];
short staged[NUM_OF_LEGS][NUM_SERVOS_PER_LEG];
short snapshot[NUM_OF_LEGS][NUM_SERVOS_PER_LEG];
bool snapshotActive, snapshotReady, zeroReady, storageReady;
short savedSnapshot[NUM_OF_LEGS][NUM_SERVOS_PER_LEG];
const char *result;

uint32_t checksum(const Record &record)
{
  uint32_t hash = 2166136261UL;
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
  for (size_t i = 0; i < offsetof(Record, checksum); ++i)
  {
    hash = (hash ^ bytes[i]) * 16777619UL;
  }
  return hash;
}

bool valid(const Record &record)
{
  if (record.magic != MAGIC || record.version != 1 || record.checksum != checksum(record))
  {
    return false;
  }
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    for (byte joint = 0; joint < NUM_SERVOS_PER_LEG; ++joint)
      if (record.offsets[leg][joint] < -MAX_OFFSET || record.offsets[leg][joint] > MAX_OFFSET)
        return false;
  return true;
}

const char PAGE[] = R"HTML(<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Servo calibration</title><h1>Servo calibration</h1>
<style>pre { white-space: pre-wrap; overflow-wrap: anywhere; }</style>
<p>No authentication. Use a trusted private network only.</p>
<p>Support the robot securely, clear obstructions, disable Control and close other control tabs.
Starting calibration immediately commands all 18 servos to zero plus their saved offsets.
Afterward, each move commands only the selected servo to zero plus its adjusted offset.
The firmware cannot verify the servo's physical position.
Keep clear of moving joints. Resetting runs normal startup positioning.</p>
<button id="enter" disabled>Start calibration</button>
<button id="zero" disabled>Retry all-servo zero</button>
<button id="exit" disabled>End calibration</button>
<fieldset id="controls" disabled>
<label>Leg <select id="leg"><option>RF</option><option>RM</option><option>RR</option>
<option>LF</option><option>LM</option><option>LR</option></select></label>
<label>Joint <select id="joint"><option>Coxa</option><option>Femur</option><option>Tibia</option></select></label>
<label>Offset in degrees <input id="offset" type="number" min="-30" max="30" step="0.1"></label>
<button id="move">Move selected servo</button><button id="save">Save all offsets</button></fieldset>
<h2>Offsets (degrees)</h2><pre id="offsets">Loading...</pre>
<p id="status" role="status">Loading offsets...</p><a href="/">Back to controller</a>
<script>
const $ = id => document.getElementById(id);
let values = [], saved = [], active = false, ready = false, busy = false;
function display() { if (values.length) $("offset").value = values[$("leg").selectedIndex][$("joint").selectedIndex] / 10; }
function enabled(on, zeroed) {
  active = on; ready = zeroed;
  $("controls").disabled = !on || !zeroed;
  $("enter").disabled = on; $("exit").disabled = !on; $("zero").disabled = !on;
}
function showOffsets() {
  const legs = ["RF", "RM", "RR", "LF", "LM", "LR"], joints = ["Coxa", "Femur", "Tibia"];
  $("offsets").textContent = legs.map((leg, i) => leg + ": " +
    joints.map((joint, j) => joint + " saved " + (saved[i][j] / 10).toFixed(1) +
      " / staged " + (values[i][j] / 10).toFixed(1)).join(", ")).join("\n");
}
async function call(action, extra = {}) {
  const response = await fetch("/api/calibration", {
    method: "POST", headers: {"Content-Type": "application/json"},
    body: JSON.stringify({action, ...extra})
  });
  const text = await response.text();
  if (!response.ok) throw Error(text);
  return JSON.parse(text);
}
async function refresh() {
  const response = await fetch("/api/calibration", {cache: "no-store"});
  const text = await response.text();
  if (!response.ok) throw Error(text);
  const data = JSON.parse(text); values = data.offsets; saved = data.saved;
  enabled(data.active, data.ready); display(); showOffsets();
  $("status").textContent = data.active ?
    (data.ready ? "Calibration active. Unsaved changes are discarded on End." :
      "All-servo zero incomplete. Inspect the robot, then retry or end calibration.") :
    "Ready. No servo moved.";
}
async function run(action, extra) {
  if (busy) return;
  busy = true; $("enter").disabled = true; $("controls").disabled = true;
  $("zero").disabled = true; $("exit").disabled = true;
  let uncertain = false;
  try { const data = await call(action, extra); values = data.offsets; saved = data.saved;
    active = data.active; ready = data.ready; display(); showOffsets();
    $("status").textContent = data.message;
  } catch (error) { uncertain = true; $("status").textContent = "Calibration failed: " + error.message +
      " The request may have taken effect; refresh before retrying."; }
  finally { busy = false; if (!uncertain) enabled(active, ready); }
}
$("enter").onclick = () => {
  if (confirm("All 18 servos will move to zero plus saved offsets. Robot supported, Control disabled, and joints clear?")) return run("enter");
};
$("zero").onclick = () => {
  if (confirm("All 18 servos will move to zero plus staged offsets. Robot supported and joints clear?")) return run("zero");
};
$("move").onclick = () => {
  const value = Number($("offset").value), tenths = Math.round(value * 10);
  if ($("offset").value === "" || !Number.isFinite(value) || Math.abs(value * 10 - tenths) > 0.00001 || tenths < -300 || tenths > 300) {
    $("status").textContent = "Use an offset from -30 to 30 degrees in 0.1 degree increments."; return;
  }
  return run("move", {leg: $("leg").selectedIndex, joint: $("joint").selectedIndex, offset: tenths});
};
$("save").onclick = () => run("save");
$("exit").onclick = () => { if (confirm("End calibration? Unsaved offsets will be discarded.")) return run("exit"); };
$("leg").onchange = $("joint").onchange = display;
refresh().catch(error => $("status").textContent = "Cannot load calibration: " + error.message);
</script></html>)HTML";

esp_err_t page(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
  return httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
}

esp_err_t error(httpd_req_t *req, const char *status, const char *message)
{
  Serial.println(message);
  httpd_resp_set_status(req, status);
  return httpd_resp_send(req, message, HTTPD_RESP_USE_STRLEN);
}

esp_err_t dispatch(httpd_req_t *req, Request next)
{
  request = next;
  State expected = State::IDLE;
  if (!state.compare_exchange_strong(expected, State::REQUESTED))
    return error(req, "409 Conflict", "Calibration request already pending.");
  for (unsigned i = 0; i < 1000 && state.load() == State::REQUESTED; ++i)
    delay(1);
  expected = State::REQUESTED;
  if (state.compare_exchange_strong(expected, State::IDLE))
    return error(req, "503 Service Unavailable", "Motion loop did not acknowledge calibration.");
  while (state.load() == State::WORKING)
    delay(1);
  if (result)
  {
    const char *message = result;
    state.store(State::IDLE);
    return error(req, "409 Conflict", message);
  }
  JsonDocument doc;
  doc["active"] = snapshotActive;
  doc["ready"] = snapshotReady;
  doc["message"] = next == Request::SAVE ? "Offsets saved." :
                   next == Request::MOVE ? "Servo target accepted; verify its position and save to keep offsets." :
                   (next == Request::ENTER || next == Request::ZERO) ?
                     "All servo targets accepted. Verify positions before adjusting." :
                   next == Request::EXIT ? "Calibration ended. Support robot before enabling Control." :
                   "Calibration status.";
  JsonArray rows = doc["offsets"].to<JsonArray>();
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    JsonArray row = rows.add<JsonArray>();
    for (byte joint = 0; joint < NUM_SERVOS_PER_LEG; ++joint)
      row.add(snapshot[leg][joint]);
  }
  JsonArray savedRows = doc["saved"].to<JsonArray>();
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
  {
    JsonArray row = savedRows.add<JsonArray>();
    for (byte joint = 0; joint < NUM_SERVOS_PER_LEG; ++joint)
      row.add(savedSnapshot[leg][joint]);
  }
  String json;
  serializeJson(doc, json);
  state.store(State::IDLE);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, json.c_str(), HTTPD_RESP_USE_STRLEN);
}

esp_err_t status(httpd_req_t *req) { return dispatch(req, Request::STATUS); }

esp_err_t command(httpd_req_t *req)
{
  char type[64], origin[128], host[96], expected[104];
  if (httpd_req_get_hdr_value_str(req, "Content-Type", type, sizeof(type)) != ESP_OK ||
      strcmp(type, "application/json"))
    return error(req, "400 Bad Request", "Use application/json.");
  if (httpd_req_get_hdr_value_len(req, "Origin"))
  {
    if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) != ESP_OK ||
        httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK)
      return error(req, "403 Forbidden", "Invalid calibration origin.");
    snprintf(expected, sizeof(expected), "http://%s", host);
    if (strcmp(origin, expected))
      return error(req, "403 Forbidden", "Cross-origin calibration denied.");
  }
  if (!req->content_len || req->content_len >= 256)
    return error(req, "400 Bad Request", "Invalid calibration command size.");
  char body[256];
  size_t received = 0;
  while (received < req->content_len)
  {
    int count = httpd_req_recv(req, body + received, req->content_len - received);
    if (count <= 0)
      return error(req, "400 Bad Request", "Incomplete calibration command.");
    received += count;
  }
  body[received] = 0;
  JsonDocument doc;
  if (deserializeJson(doc, body) || !doc["action"].is<const char *>())
    return error(req, "400 Bad Request", "Invalid calibration command.");
  const char *action = doc["action"];
  Request next;
  if (!strcmp(action, "enter")) next = Request::ENTER;
  else if (!strcmp(action, "zero")) next = Request::ZERO;
  else if (!strcmp(action, "save")) next = Request::SAVE;
  else if (!strcmp(action, "exit")) next = Request::EXIT;
  else if (!strcmp(action, "move"))
  {
    if (!doc["leg"].is<int>() || !doc["joint"].is<int>() || !doc["offset"].is<int>() ||
        doc["leg"].as<int>() < 0 || doc["leg"].as<int>() >= NUM_OF_LEGS ||
        doc["joint"].as<int>() < 0 || doc["joint"].as<int>() >= NUM_SERVOS_PER_LEG ||
        doc["offset"].as<int>() < -MAX_OFFSET || doc["offset"].as<int>() > MAX_OFFSET)
      return error(req, "400 Bad Request", "Invalid leg, joint or offset.");
    requestLeg = doc["leg"].as<int>();
    requestJoint = doc["joint"].as<int>();
    requestOffset = doc["offset"].as<int>();
    next = Request::MOVE;
  }
  else return error(req, "400 Bad Request", "Unknown calibration action.");
  return dispatch(req, next);
}
}

void calibration_init()
{
  const short *defaults[] = {CoxaAngleOffset, FemurAngleOffset, TibiaAngleOffset};
  for (byte leg = 0; leg < NUM_OF_LEGS; ++leg)
    for (byte joint = 0; joint < NUM_SERVOS_PER_LEG; ++joint)
      offsets[leg][joint] = (short)pgm_read_word(&defaults[joint][leg]);
  storageReady = EEPROM.begin(sizeof(Record));
  if (!storageReady)
  {
    Serial.println("Calibration EEPROM initialization failed; using configured defaults.");
    return;
  }
  Record record;
  EEPROM.get(0, record);
  if (valid(record))
    memcpy(offsets, record.offsets, sizeof(offsets));
  else
    Serial.println("No valid saved calibration; using configured defaults.");
}

short calibration_offset(byte leg, byte joint) { return offsets[leg][joint]; }

bool calibration_busy()
{
  return active.load() || (state.load() != State::IDLE && request.load() != Request::STATUS);
}

bool calibration_update(bool allowCalibration, CalibrationMoveResult (*move)(byte, byte, short))
{
  State expected = State::REQUESTED;
  if (state.compare_exchange_strong(expected, State::WORKING))
  {
    result = nullptr;
    if (request == Request::ENTER)
    {
      if (active.load()) result = "Calibration already active.";
      else if (!allowCalibration) result = "Disable Control and wait for motion to stop.";
      else
      {
        memcpy(staged, offsets, sizeof(staged));
        active.store(true);
        zeroReady = false;
      }
    }
    else if (request != Request::STATUS && !active.load())
      result = "Start calibration first.";
    if (!result && (request == Request::ENTER || request == Request::ZERO))
    {
      zeroReady = false;
      for (byte leg = 0; leg < NUM_OF_LEGS && !result; ++leg)
      {
        for (byte joint = 0; joint < NUM_SERVOS_PER_LEG; ++joint)
        {
          CalibrationMoveResult moved = move(leg, joint, staged[leg][joint]);
          if (moved == CalibrationMoveResult::POWER_UNAVAILABLE)
            result = "Battery readings unavailable or unsafe; all-servo zero incomplete. Inspect robot, then retry or end calibration.";
          else if (moved == CalibrationMoveResult::PWM_ERROR)
            result = "Servo PWM write failed; all-servo zero incomplete and physical positions unknown. Inspect robot, then retry or end calibration.";
          if (result) break;
        }
      }
      zeroReady = !result;
    }
    else if (!result && (request == Request::MOVE || request == Request::SAVE) && !zeroReady)
      result = "All-servo zero incomplete; inspect robot, then retry or end calibration.";
    else if (!result && request == Request::MOVE)
    {
      CalibrationMoveResult moved = move(requestLeg, requestJoint, requestOffset);
      if (moved == CalibrationMoveResult::POWER_UNAVAILABLE)
        result = "Battery readings unavailable or unsafe; servo move blocked.";
      else if (moved == CalibrationMoveResult::PWM_ERROR)
        result = "Servo PWM write failed; physical servo position is unknown.";
      else
        staged[requestLeg][requestJoint] = requestOffset;
    }
    else if (!result && request == Request::SAVE)
    {
      if (!storageReady) result = "Calibration EEPROM unavailable; offsets not saved.";
      else
      {
        Record record = {};
        record.magic = MAGIC;
        record.version = 1;
        memcpy(record.offsets, staged, sizeof(staged));
        record.checksum = checksum(record);
        EEPROM.put(0, record);
        if (!EEPROM.commit()) result = "Calibration EEPROM write failed; offsets not saved.";
        else memcpy(offsets, staged, sizeof(offsets));
      }
    }
    else if (!result && request == Request::EXIT)
    {
      active.store(false);
      zeroReady = false;
    }
    snapshotActive = active.load();
    snapshotReady = zeroReady;
    memcpy(snapshot, snapshotActive ? staged : offsets, sizeof(snapshot));
    memcpy(savedSnapshot, offsets, sizeof(savedSnapshot));
    state.store(State::DONE);
  }
  return active.load();
}

void calibration_register(httpd_handle_t server)
{
  const char *paths[] = {"/calibration", "/api/calibration", "/api/calibration"};
  esp_err_t (*handlers[])(httpd_req_t *) = {page, status, command};
  for (unsigned i = 0; i < 3; ++i)
  {
    httpd_uri_t uri = {};
    uri.uri = paths[i];
    uri.method = i == 2 ? HTTP_POST : HTTP_GET;
    uri.handler = handlers[i];
    if (httpd_register_uri_handler(server, &uri) != ESP_OK)
      Serial.printf("Cannot register calibration route %s\n", paths[i]);
  }
}
