#include <WiFi.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "wifey.h"
#include "ota.h"

// Track network-stack initialization separately from router connection; the hotspot can work
// offline.
static bool networkReady;
// One radio serves hotspot and router: a router retry scans every channel and stalls the hotspot.
#define ROUTER_RETRY_MS 20000
static bool routerRetrying;
static uint32_t routerLastSeen;

bool wifi_ready()
{
  return networkReady;
}

void wifi_limitRouterRetry(bool connected, uint32_t now)
{
  // Give up on an absent router so hotspot control is not interrupted by reconnect scans.
  if (!routerRetrying)
  {
    return;
  }
  if (connected)
  {
    routerLastSeen = now;
  }
  else if (uint32_t(now - routerLastSeen) >= ROUTER_RETRY_MS)
  {
    routerRetrying = false;
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(false);
    Serial.println("Router not reachable; stopped router scanning. Hotspot only until reset.");
  }
}

void wifi_init()
{
  networkReady = false;
  Preferences settings;
  // Read/write mode creates the namespace on first boot; read-only previously failed with
  // NOT_FOUND.
  if (!settings.begin("hex-network", false))
  {
    Serial.println("Network storage unavailable. Configure over Serial.");
    return;
  }
  // Provision outside source code so firmware sharing never requires embedding Wi-Fi credentials.
  String ssid = settings.getString("ssid");
  String password = settings.getString("password");
  String apSsid = settings.getString("ap-ssid");
  String apPassword = settings.getString("ap-password");
  settings.end();
  bool routerConfigured =
      !ssid.isEmpty() && ssid.length() <= 32 && password.length() >= 8 && password.length() <= 63;
  bool hotspotConfigured = !apSsid.isEmpty() && apSsid.length() <= 32 && apPassword.length() >= 8 &&
                           apPassword.length() <= 63;
  if (!hotspotConfigured && routerConfigured)
  {
    // Preserve existing installations until independent HOTSPOT credentials have been saved.
    apSsid = "Hexapod-Control";
    apPassword = password;
    hotspotConfigured = true;
    Serial.println("Using legacy hotspot settings; use HOTSPOT to set separate credentials.");
  }
  if (!hotspotConfigured && !routerConfigured)
  {
    // Leave Serial provisioning usable without starting sockets against an uninitialized TCP/IP
    // stack.
    Serial.println(
        "Network not configured. Serial: NET or HOTSPOT {\"ssid\":\"...\",\"password\":\"...\"}");
    return;
  }
  if (!WiFi.mode(WIFI_AP_STA))
  {
    Serial.println(
        "Wi-Fi initialization failed. Serial configuration remains available; reset to retry.");
    return;
  }
  networkReady = true;
  // Modem power save delays packets by beacon intervals; control latency matters more than power.
  WiFi.setSleep(false);
  if (!WiFi.softAP(apSsid.c_str(), apPassword.c_str(), 1, false, 2))
  {
    Serial.println("Hotspot startup failed.");
  }
  else
  {
    Serial.print("Hotspot ");
    Serial.print(apSsid);
    Serial.print(": http://");
    Serial.println(WiFi.softAPIP());
  }
  if (routerConfigured)
  {
    WiFi.begin(ssid.c_str(), password.c_str());
    routerRetrying = true;
    routerLastSeen = millis();
    Serial.println("Router connection started. No automatic motion.");
  }
  else
  {
    Serial.println("Router not configured; hotspot-only control. No automatic motion.");
  }
}

void wifi_update(bool allowConfiguration)
{
  ota_update(allowConfiguration);
  if (ota_locked())
  {
    return;
  }
  // Provision only while disabled and apply on reset; changing networks must not interrupt active
  // motion.
  static char buffer[640];
  static unsigned length;
  static bool overflow;
  static bool reported;
  bool connected = networkReady && WiFi.status() == WL_CONNECTED;
  wifi_limitRouterRetry(connected, millis());
  if (connected && !reported)
  {
    reported = true;
    Serial.print("Router control page: http://");
    Serial.println(WiFi.localIP());
  }
  if (!connected)
  {
    reported = false;
  }
  for (unsigned count = 0; count < 32 && Serial.available(); ++count)
  {
    // Bound parsing work so a long Serial stream cannot starve gait and switch updates.
    char ch = Serial.read();
    if (ch == '\r')
    {
      continue;
    }
    if (ch != '\n')
    {
      if (length < sizeof(buffer) - 1)
      {
        buffer[length++] = ch;
      }
      else
      {
        overflow = true;
      }
      continue;
    }
    buffer[length] = 0;
    if (!allowConfiguration)
    {
      Serial.println("Network configuration rejected: disable robot first.");
    }
    else if (overflow)
    {
      Serial.println("Configuration rejected: Serial line too long (maximum 639 bytes).");
    }
    else if (strncmp(buffer, "NET ", 4) && strncmp(buffer, "HOTSPOT ", 8))
    {
      Serial.println("Use NET/HOTSPOT {\"ssid\":\"...\",\"password\":\"...\"}; "
                     "OTA has no password. Input not echoed by firmware.");
    }
    else
    {
      JsonDocument doc;
      // Separate NVS keys let the robot hotspot keep its own name/password instead of copying the
      // router.
      bool hotspot = !strncmp(buffer, "HOTSPOT ", 8);
      auto error = deserializeJson(doc, buffer + (hotspot ? 8 : 4));
      String ssid = doc["ssid"] | "";
      String password = doc["password"] | "";
      const char *name = hotspot ? "HOTSPOT" : "NET";
      if (error)
      {
        Serial.printf("%s settings rejected: invalid JSON (%s). Use straight double quotes "
                      "and a complete JSON object.\n",
                      name, error.c_str());
      }
      else if (doc["password"].isNull())
      {
        Serial.printf("%s settings rejected: missing password field. Use the lowercase key "
                      "\"password\".\n",
                      name);
      }
      else if (!doc["password"].is<const char *>())
      {
        Serial.printf("%s settings rejected: password must be a JSON string in double quotes.\n",
                      name);
      }
      else if (password.length() < 8 || password.length() > 63)
      {
        Serial.printf("%s settings rejected: password must be 8-63 bytes.\n", name);
      }
      else if (!doc["ssid"].is<const char *>() || ssid.isEmpty() || ssid.length() > 32)
      {
        Serial.printf("%s settings rejected: ssid must be a JSON string of 1-32 bytes.\n", name);
      }
      else
      {
        Preferences settings;
        if (!settings.begin("hex-network", false))
        {
          Serial.println("Cannot open network storage.");
        }
        else
        {
          bool ok = settings.putString(hotspot ? "ap-ssid" : "ssid", ssid) > 0 &&
                    settings.putString(hotspot ? "ap-password" : "password", password) > 0;
          settings.end();
          Serial.println(ok ? (hotspot ? "Hotspot saved. Reset robot to apply."
                                      : "Network saved. Reset robot to apply.")
                            : "Network save failed; configure again.");
        }
      }
    }
    // Do not retain or echo the provisioning line, which contains the plaintext password.
    memset(buffer, 0, sizeof(buffer));
    length = 0;
    overflow = false;
  }
}
