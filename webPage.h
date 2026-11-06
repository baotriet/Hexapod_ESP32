#ifndef WEB_PAGE_H_
#define WEB_PAGE_H_

// Keep the page inside the raw string so the ESP32 can serve it without a filesystem.
static const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Hexapod control</title>
  <style>
    /* Small consistent controls keep the panel usable between two phone joysticks. */
    * {
      box-sizing: border-box;
    }
    body {
      background: #121a25;
      color: #e7eef7;
      font: 16px system-ui;
      margin: 0;
      padding: 16px;
      max-width: 1200px;
      margin-inline: auto;
      isolation: isolate;
    }
    h1 {
      font-size: 24px;
    }
    button,
    select {
      font: inherit;
      font-size: 13px;
      padding: 4px 8px;
      min-height: 32px;
      border-radius: 6px;
      margin: 3px;
      border: 1px solid #8295af;
      background: #24354b;
      color: white;
    }
    button:disabled {
      opacity: 0.45;
    }
    .stop {
      background: #a32936;
      font-weight: bold;
    }
    /* Keep Camera beside Control, with STOP accessible on the same compact row. */
    .command-row {
      display: flex;
      align-items: center;
      gap: 4px;
      flex-wrap: wrap;
    }
    .command-row button {
      margin: 0;
    }
    section {
      background: #1c293a;
      padding: 12px;
      border-radius: 12px;
      margin: 12px 0;
    }
    /* Reserve the middle for video; settings must never obscure the driving view. */
    .deck {
      display: grid;
      grid-template-columns: minmax(120px, 200px) minmax(0, 1fr) minmax(120px, 200px);
      gap: 12px;
      align-items: start;
    }
    .joystick {
      text-align: center;
      position: sticky;
      top: 12px;
    }
    .stick {
      width: 100%;
      aspect-ratio: 1;
      border: 2px solid #7596bb;
      border-radius: 50%;
      touch-action: none;
      position: relative;
      background: #101b29;
      user-select: none;
    }
    #left-pad {
      grid-column: 1;
      grid-row: 1;
    }
    #right-pad {
      grid-column: 3;
      grid-row: 1;
    }
    #camera-view {
      grid-column: 2;
      grid-row: 1;
      align-self: center;
      justify-self: center;
      width: 100%;
      max-width: 560px;
      aspect-ratio: 4 / 3;
      max-height: 55dvh;
      min-width: 0;
      display: grid;
      place-items: center;
      background: #101b29;
      border-radius: 8px;
      overflow: hidden;
    }
    #camera-view span {
      padding: 8px;
      text-align: center;
      color: #abb9ca;
    }
    #controls {
      /* Stay in the middle column below the camera so settings never widen past the sticks. */
      grid-column: 2;
      grid-row: 2;
      width: 100%;
      max-width: 560px;
      justify-self: center;
      margin: 0;
      padding: 6px;
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 6px;
      align-items: center;
      font-size: 13px;
    }
    #controls label {
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      gap: 2px;
      min-width: 0;
    }
    #controls button {
      width: auto;
      max-width: 100%;
      justify-self: start;
      margin: 0;
    }
    #controls select {
      width: 100%;
      margin: 0;
    }
    #controls input {
      width: 100%;
      margin: 0;
      min-height: 32px;
    }
    #controls .control-row {
      /* Shared four-column rows keep selectors and body/lift buttons aligned without extra descriptions. */
      grid-column: 1/-1;
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 6px;
      align-items: center;
    }
    #controls .body-row {
      gap: 4px;
    }
    #controls .body-row button {
      width: 100%;
      height: 100%;
      padding: 3px 4px;
      font-size: 12px;
    }
    #controls .toggle-row {
      /* Let secondary toggles wrap rather than overflow a narrow phone viewport. */
      grid-column: 1/-1;
      display: flex;
      flex-wrap: wrap;
      align-items: center;
      gap: 4px;
    }
    #controls .toggle-row button {
      padding: 3px 6px;
      white-space: nowrap;
    }
    #controls .speed-control {
      grid-column: 1/-1;
    }
    #controls select {
      min-width: 0;
      max-width: 100%;
    }
    /* In-page pickers keep heartbeat timers running without native popup focus changes. */
    .picker {
      position: relative;
      min-width: 0;
    }
    .picker summary {
      background: #101b29;
      border: 1px solid #7596bb;
      border-radius: 6px;
      padding: 6px;
      cursor: pointer;
      overflow-wrap: anywhere;
    }
    .picker[open] {
      z-index: 5;
    }
    .picker-options {
      position: absolute;
      top: 100%;
      left: 0;
      min-width: 100%;
      background: #1c293a;
      border: 1px solid #7596bb;
      border-radius: 6px;
      padding: 3px;
      display: grid;
      gap: 3px;
    }
    #controls .picker-options button {
      width: 100%;
      max-width: none;
      white-space: normal;
      font-size: 12px;
      padding: 6px;
    }
    button[aria-pressed="true"] {
      /* Fixed labels use color and accessible pressed state, not changing ON/OFF button text. */
      background: #215d46;
      border-color: #7eecad;
      color: #d6ffe7;
    }
    #orientation {
      display: none;
      background: #473b19;
      padding: 10px;
      border-radius: 8px;
    }
    #notice {
      color: #ffd47e;
      overflow-wrap: anywhere;
    }
    #notice:empty {
      display: none;
    }
    .knob {
      width: 42px;
      height: 42px;
      border-radius: 50%;
      background: #71b6ff;
      position: absolute;
      left: calc(50% - 21px);
      top: calc(50% - 21px);
      pointer-events: none;
    }
    #camera {
      display: none;
    }
    /* Preserve camera aspect ratio inside the clear middle panel without intercepting input. */
    body.camera-active #camera {
      display: block;
      width: 100%;
      height: 100%;
      object-fit: contain;
      pointer-events: none;
    }
    body.camera-active #camera-view span {
      display: none;
    }
    pre {
      white-space: pre-wrap;
      overflow-wrap: anywhere;
    }
    .on {
      color: #7eecad;
    }
    .off {
      color: #abb9ca;
    }
    /* Recommend landscape without forcing orientation; portrait must remain usable instead of overflowing. */
    @media (max-width: 950px) and (orientation: landscape) {
      body {
        padding: 8px;
        font-size: 14px;
      }
      .deck {
        grid-template-columns: clamp(110px, 20vw, 170px) minmax(0, 1fr) clamp(110px, 20vw, 170px);
        gap: 8px;
      }
      h1 {
        font-size: 16px;
        margin: 4px 0;
      }
      #connection-panel {
        padding: 4px 8px;
        margin: 4px 0;
      }
    }
    @media (max-width: 600px) and (orientation: portrait) {
      #orientation {
        display: block;
      }
      .deck {
        grid-template-columns: minmax(80px, 1fr) minmax(0, 1.5fr) minmax(80px, 1fr);
        gap: 6px;
      }
      .joystick {
        position: static;
      }
      #controls {
        grid-column: 1/-1;
        grid-row: 2;
      }
      .stick {
        max-width: 200px;
        margin: auto;
      }
    }
  </style>
  <!-- Connection and control ownership -->
  <h1>Hexapod control</h1>
  <p id="link">Connecting...</p>
  <p id="notice" role="status"></p>
  <p id="orientation" role="status">
    Rotate to landscape, or tap Landscape to request fullscreen rotation.
  </p>
  <div class="command-row">
    <button id="arm" aria-pressed="false">Control</button>
    <button id="video" aria-pressed="false">Camera</button>
    <button id="landscape">Landscape</button>
    <button class="stop" id="stop">STOP MOTION / HOLD</button>
  </div>
  <small>STOP retains torque; not a hardware emergency stop.</small>
  <section id="connection-panel">
    <div id="network"></div>
    <a href="/update">Firmware update (.bin)</a>
  </section>
  <!-- Keep labels compact and omit grip controls; data-action values must still match the firmware API. -->
  <div class="deck">
    <div class="joystick" id="left-pad">
      <p>Left stick (WASD)</p>
      <div class="stick" id="left"><span class="knob"></span></div>
    </div>
    <div id="camera-view">
      <span>Camera preview</span>
      <img id="camera" alt="Live robot camera" />
    </div>
    <section id="controls">
      <div class="control-row">
        <label>
          Operation
          <select data-action="mode">
            <option value="0">Walk</option>
            <option value="1">Translate</option>
            <option value="2">Rotate</option>
            <option value="3">Single leg</option>
          </select>
        </label>
        <label>
          Gait
          <select data-action="gait">
            <option value="0">Tripod 8</option>
            <option value="1">Ripple 12</option>
            <option value="2">Triple tripod 12</option>
            <option value="3">Triple tripod 16</option>
            <option value="4">Wave 24</option>
            <option value="5">Tripod 6</option>
          </select>
        </label>
        <label>
          Height
          <select data-action="height">
            <option value="45">45 mm creep</option>
            <option value="56" selected>56 mm</option>
            <option value="65">65 mm</option>
            <option value="85">85 mm</option>
          </select>
        </label>
        <label>
          Single leg
          <select data-action="leg">
            <option value="255">None</option>
            <option value="0">RF</option>
            <option value="1">RM</option>
            <option value="2">RR</option>
            <option value="3">LF</option>
            <option value="4">LM</option>
            <option value="5">LR</option>
          </select>
        </label>
      </div>
      <div class="control-row body-row">
        <!-- Height/Lift only choose which value the shared +5/-5 buttons change; they send nothing. -->
        <button id="adjust-height" data-target="body" aria-pressed="false">Height</button>
        <button id="adjust-lift" data-target="lift" aria-pressed="false">Lift</button>
        <button data-action="adjust" data-step="Up" data-value="0">+5 mm</button>
        <button data-action="adjust" data-step="Down" data-value="0">-5 mm</button>
      </div>
      <label class="speed-control">
        <span>
          Speed:
          <output id="speed-value" for="speed">50%</output>
        </span>
        <input id="speed" type="range" data-action="speed" min="0" max="100" value="50" />
      </label>
      <div class="toggle-row">
        <button data-action="travel" data-toggle data-label="Double travel" aria-pressed="false">
          Double travel
        </button>
        <button data-action="balance" data-toggle data-label="Balance" aria-pressed="false">
          Balance
        </button>
        <button data-action="imu" data-toggle data-label="IMU active" aria-pressed="false">
          IMU active
        </button>
        <button data-action="feet" data-toggle data-label="Foot sensor" aria-pressed="false">
          Foot sensor
        </button>
        <button data-action="hold" data-toggle data-label="Single-leg hold" aria-pressed="false">
          Single-leg hold
        </button>
      </div>
    </section>
    <div class="joystick" id="right-pad">
      <p>Right stick (arrow keys)</p>
      <div class="stick" id="right"><span class="knob"></span></div>
    </div>
  </div>
  <!-- Camera and robot telemetry -->
  <section id="camera-panel">
    <p id="battery">Battery: awaiting telemetry</p>
    <div id="feet"></div>
    <pre id="robot">Awaiting robot state</pre>
  </section>
  <script>
    "use strict";

    // Keep command retry timeout below the firmware's 2000 ms lease so a stalled controller still stops.
    const CONTROL_INTERVAL_MS = 50;
    const COMMAND_TIMEOUT_MS = 1500;
    const TELEMETRY_INTERVAL_MS = 200;
    const TELEMETRY_TIMEOUT_MS = 1000;
    const DISARM_GRACE_MS = 1000;

    const $ = (id) => document.getElementById(id);
    const client = new Uint32Array(1);
    // A per-page identity separates controller leases; opening a second page must not inherit movement.
    crypto.getRandomValues(client);
    if (!client[0]) client[0] = 1;
    let enabled = false;
    let axes = [128, 128, 128, 128]; // Left X/Y, right X/Y; neutral is 128.
    let actions = [];
    let stopping = false;
    let flight = false;
    let video = false;
    let sequence = 0;
    let enableSince = 0;
    let lastAccepted = 0;
    // A changed boot ID means the robot restarted; keep the reason visible until Control is re-enabled.
    let bootId = null;
    let restartNote = "";
    // Codes match the firmware's STOP_REASONS table so robot messages name the actual trigger.
    const STOP_TEXT = [
      "",
      "STOP button",
      "Control off",
      "no reply in 1.5 s",
      "network error",
      "robot busy 1.5 s",
      "page lost focus",
      "page hidden",
      "Space key",
    ];
    let stopReason = 0;
    const keys = new Set();
    const pointers = new Map();

    // Motion reset and control enable/disable.
    function neutral() {
      axes = [128, 128, 128, 128];
      keys.clear();
      pointers.clear();
      draw();
    }
    function stop(disable = false, reason = 0) {
      // Clear held keys/actions as well as axes so reconnecting cannot replay pre-stop movement.
      const active = enabled || stopping;
      neutral();
      actions = [];
      stopping = !disable || active;
      if (active || !disable) stopReason = reason;
      sequence++;
      if (disable) enabled = false;
    }
    $("stop").onclick = () => stop(false, 1);
    $("arm").onclick = () => {
      // One fixed-label button toggles explicit enable/disable; viewing telemetry never claims control.
      if (enabled) {
        stop(true, 2);
        return;
      }
      neutral();
      enabled = true;
      restartNote = "";
      enableSince = Date.now();
      lastAccepted = enableSince;
      actions = [];
      actions.push({ action: "enable", value: 1 });
      sequence++;
    };
    // Joystick drawing and pointer input.
    function draw() {
      for (const [id, i] of [
        ["left", 0],
        ["right", 2],
      ]) {
        const el = $(id),
          k = el.firstElementChild,
          range = Math.max(0, el.getBoundingClientRect().width / 2 - 23);
        k.style.transform = `translate(${((axes[i] - 128) / 128) * range}px,${((axes[i + 1] - 128) / 128) * range}px)`;
      }
    }
    for (const [id, index] of [
      ["left", 0],
      ["right", 2],
    ]) {
      const el = $(id);
      function move(e) {
        if (pointers.get(id) !== e.pointerId) return;
        const r = el.getBoundingClientRect(),
          radius = r.width / 2;
        let x = (e.clientX - r.left - radius) / radius,
          y = (e.clientY - r.top - radius) / radius;
        const m = Math.max(1, Math.hypot(x, y));
        x /= m;
        y /= m;
        axes[index] = Math.max(0, Math.min(255, Math.round(128 + x * 127)));
        axes[index + 1] = Math.max(0, Math.min(255, Math.round(128 + y * 127)));
        draw();
      }
      el.onpointerdown = (e) => {
        if (!enabled) return;
        // Track each stick's pointer independently so releasing one thumb does not cancel the other.
        pointers.set(id, e.pointerId);
        el.setPointerCapture(e.pointerId);
        move(e);
      };
      el.onpointermove = move;
      function release(e) {
        if (pointers.get(id) !== e.pointerId) return;
        pointers.delete(id);
        axes[index] = axes[index + 1] = 128;
        draw();
      }
      el.onpointerup = release;
      el.onpointercancel = release;
      el.onlostpointercapture = release;
    }
    // Keyboard input and focus-loss stopping.
    function keyboard() {
      axes = [128, 128, 128, 128];
      if (keys.has("a")) axes[0] = 0;
      if (keys.has("d")) axes[0] = 255;
      if (keys.has("w")) axes[1] = 0;
      if (keys.has("s")) axes[1] = 255;
      if (keys.has("ArrowLeft")) axes[2] = 0;
      if (keys.has("ArrowRight")) axes[2] = 255;
      if (keys.has("ArrowUp")) axes[3] = 0;
      if (keys.has("ArrowDown")) axes[3] = 255;
      draw();
    }
    window.onkeydown = (e) => {
      if (e.code === "Space") {
        e.preventDefault();
        stop(false, 8);
        return;
      }
      // Typing/changing a focused control must not also drive the robot through keyboard shortcuts.
      if (/INPUT|SELECT|TEXTAREA/.test(e.target.tagName) || !enabled) return;
      if (["w", "a", "s", "d", "ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"].includes(e.key)) {
        e.preventDefault();
        keys.add(e.key);
        keyboard();
      }
    };
    window.onkeyup = (e) => {
      if (keys.delete(e.key)) keyboard();
    };
    window.onblur = () => {
      // Settings use in-page menus, so a real window focus loss must still disarm.
      stop(true, 6);
    };
    document.addEventListener("visibilitychange", () => {
      // Background tabs throttle timers; disarm rather than relying on continued joystick heartbeats.
      if (document.hidden) stop(true, 7);
    });
    // Settings buttons/selectors and camera streaming.
    $("speed").oninput = () => {
      $("speed-value").textContent = `${$("speed").value}%`;
    };
    let adjustTarget = "";
    document.querySelectorAll("[data-target]").forEach((el) => {
      el.onclick = () => {
        // Local choice only, so selecting a target can never move the robot by itself.
        adjustTarget = el.dataset.target;
        document
          .querySelectorAll("[data-target]")
          .forEach((b) => b.setAttribute("aria-pressed", String(b === el)));
      };
    });
    document.querySelectorAll("[data-action]").forEach((el) => {
      const send = () => {
        const sensorExit =
          el.dataset.action === "feet" && el.getAttribute("aria-pressed") === "true";
        if (sensorExit) {
          // HOLD may retain joystick input, STOP, or queued settings; exit must supersede all three.
          stop(true);
          stopping = false;
          actions.push({ action: "feet", value: 0 });
          $("arm").setAttribute("aria-pressed", "false");
          return;
        }
        // Sensor HOLD may already have disarmed Control; its confirmed green button must still allow exit.
        if (!enabled) {
          $("link").textContent = "Enable control first";
          return;
        }
        if (actions.length >= 8) {
          // Bound settings backlog so repeated taps cannot create a long delayed sequence.
          $("link").textContent = "Wait for pending actions";
          return;
        }
        let value = Number(el.dataset.value ?? el.value);
        let action = el.dataset.action;
        if (action === "adjust") {
          if (!adjustTarget) {
            $("link").textContent = "Select Height or Lift first";
            return;
          }
          // Relative firmware steps stay correct when taps arrive faster than telemetry.
          action = adjustTarget + el.dataset.step;
        }
        // Show intuitive speed percentage while preserving firmware's inverse added-delay setting.
        if (el.dataset.action === "speed") value = 100 - value;
        if (el.hasAttribute("data-toggle")) {
          if (actions.some((a) => a.action === el.dataset.action)) return;
          value = Number(
            el.getAttribute("aria-pressed") === "true"
              ? (el.dataset.off ?? 0)
              : (el.dataset.on ?? 1),
          );
        }
        actions.push({ action, value });
      };
      if (el.tagName === "BUTTON") el.onclick = send;
      else el.onchange = send;
    });
    const pickers = [];
    document.querySelectorAll("select[data-action]").forEach((el) => {
      // Avoid native OS menus: their blur/focus behavior can interrupt Control during mode selection.
      const picker = document.createElement("details"),
        summary = document.createElement("summary"),
        options = document.createElement("div");
      picker.className = "picker";
      options.className = "picker-options";
      summary.textContent = el.selectedOptions[0].textContent;
      summary.setAttribute("aria-label", el.parentElement.firstChild.textContent.trim());
      picker.append(summary, options);
      el.hidden = true;
      el.after(picker);
      el.picker = picker;
      el.pickerSummary = summary;
      summary.onclick = () => {
        // Opening settings neutralizes travel but does not revoke explicit arming.
        neutral();
        pickers.forEach((other) => {
          if (other !== picker) other.open = false;
        });
      };
      for (const option of el.options) {
        const button = document.createElement("button");
        button.type = "button";
        button.textContent = option.textContent;
        button.onclick = () => {
          el.value = option.value;
          summary.textContent = option.textContent;
          picker.open = false;
          el.onchange();
          summary.focus();
        };
        options.append(button);
      }
      picker.onkeydown = (e) => {
        if (e.key === "Escape") {
          picker.open = false;
          summary.focus();
        }
      };
      pickers.push(picker);
    });
    document.addEventListener("pointerdown", (e) => {
      pickers.forEach((picker) => {
        if (!picker.contains(e.target)) picker.open = false;
      });
    });
    async function requestLandscape(fullscreen) {
      // Browser policy may require a user gesture/fullscreen; never fake rotation with CSS.
      try {
        if (typeof screen === "undefined" || !screen.orientation?.lock)
          throw Error("orientation lock is not supported");
        if (fullscreen && !document.fullscreenElement) {
          if (!document.documentElement.requestFullscreen)
            throw Error("fullscreen is not supported");
          await document.documentElement.requestFullscreen();
        }
        await screen.orientation.lock("landscape");
        $("orientation").textContent = "Landscape orientation requested.";
      } catch (e) {
        $("orientation").textContent =
          `Rotate the phone manually, or tap Landscape. Browser rotation unavailable: ${e.message}`;
      }
    }
    $("landscape").onclick = () => requestLandscape(true);
    if (typeof matchMedia !== "undefined" && matchMedia("(pointer: coarse)").matches)
      requestLandscape(false);
    $("video").onclick = () => {
      // Camera is a separate port-81 stream; stopping the image must not stop or claim robot control.
      video = !video;
      $("video").setAttribute("aria-pressed", String(video));
      // Change presentation only; camera toggling must not arm, stop, or reset movement controls.
      document.body.classList.toggle("camera-active", video);
      if (video) $("camera").src = `http://${location.hostname}:81/stream`;
      else $("camera").removeAttribute("src");
    };
    // Serialize HTTP requests so slow responses cannot reorder settings or build a stale movement backlog.
    async function update() {
      if (flight || document.hidden || (!enabled && !stopping && !actions.length)) return;
      flight = true;
      const action = actions[0] || { action: "", value: 0 };
      const wasStop = stopping,
        version = sequence;
      const controller = new AbortController(),
        timer = setTimeout(() => controller.abort(), COMMAND_TIMEOUT_MS);
      try {
        const response = await fetch("/api/control", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            client: client[0],
            enable: enabled,
            stop: wasStop,
            lx: axes[0],
            ly: axes[1],
            rx: axes[2],
            ry: axes[3],
            ...action,
          }),
          signal: controller.signal,
        });
        if (!response.ok) {
          if (response.status === 409) {
            const rejection = await response.json();
            if (rejection.code === "busy" && Date.now() - lastAccepted < COMMAND_TIMEOUT_MS) {
              // A full mailbox is temporary, not lost ownership; retain the action but bound retries.
              $("link").textContent = "Robot busy - retrying latest command";
              return;
            }
            throw Error(rejection.error || "Command rejected");
          }
          throw Error(await response.text());
        }
        lastAccepted = Date.now();
        if (action.action && actions[0] === action) actions.shift();
        // A late response to an older stop must not clear a newer stop requested while it was in flight.
        if (wasStop && version === sequence) stopping = false;
        $("link").textContent = enabled
          ? "Connected - control requested"
          : restartNote || "Connected - disabled";
      } catch (e) {
        // A real interruption drops local arming and sends at most a best-effort stop; never auto-resume.
        const wasEnabled = enabled;
        enabled = false;
        neutral();
        actions = [];
        stopping = wasEnabled;
        $("link").textContent =
          "Control interrupted: " +
          (e.name === "AbortError" ? "no reply from robot within 1.5 s" : e.message);
      } finally {
        clearTimeout(timer);
        flight = false;
        if (!enabled) $("arm").setAttribute("aria-pressed", "false");
      }
    }
    setInterval(update, CONTROL_INTERVAL_MS);

    // Read actual robot state, including rejected settings and disconnects.
    let telemetryFlight = false;
    async function updateTelemetry() {
      if (telemetryFlight || document.hidden) return;
      telemetryFlight = true;
      const controller = new AbortController(),
        timer = setTimeout(() => controller.abort(), TELEMETRY_TIMEOUT_MS);
      try {
        const r = await fetch("/api/status", { signal: controller.signal });
        if (!r.ok) throw Error(r.status);
        const s = await r.json();
        if (s.boot !== undefined) {
          if (bootId !== null && s.boot !== bootId) {
            // A restarted robot is disarmed; never carry the old page's arming or movement across it.
            enabled = false;
            neutral();
            actions = [];
            stopping = false;
            restartNote = `Robot restarted (${s.resetReason || "unknown reason"}) - enable Control explicitly`;
            $("link").textContent = restartNote;
          }
          bootId = s.boot;
        }
        if (s.sensorBypass && !s.armed && enabled && !actions.some((a) => a.action === "enable")) {
          // Reflect sensor cancellation locally so the next Control click enables instead of disabling again.
          enabled = false;
          neutral();
          actions = [];
          stopping = false;
        }
        $("network").textContent = s.preview
          ? "Local preview - not connected to the robot"
          : `Robot IP on router: ${s.station || "not connected"} | Robot hotspot IP: ${s.hotspot || "off"}`;
        $("battery").textContent =
          s.cell1 == null
            ? "Battery unavailable"
            : `Battery cells: ${s.cell1.toFixed(2)} V / ${s.cell2.toFixed(2)} V`;
        $("feet").textContent = ["RF", "RM", "RR", "LF", "LM", "LR"]
          .map((n, i) => `${n}: ${s.contacts & (1 << i) ? "CONTACT" : "released"}`)
          .join(" | ");
        $("robot").textContent =
          `Control: ${s.armed ? "ARMED" : "DISARMED"} | Motion: ${s.hold ? "HOLD" : s.enabled ? "enabled" : "disabled"}\n` +
          `Foot sensor: ${s.feetProtection ? "ON" : "OFF"} | Landing: ${s.landing ? "pending" : "settled"}\n` +
          `Height: ${s.height} mm | Speed: ${100 - s.speed}%\nRecovery: ${s.recoveryState || "IDLE"}\n${s.result || ""}${s.fault ? "\nLast fault: " + s.fault : ""}`;
        // Distinguish cancelable sensor HOLD from hard faults instead of presenting every rejection as a freeze.
        $("notice").textContent = s.hold
          ? `HOLD: ${s.fault || s.result || "Motion stopped"}. ${
              s.hardFault
                ? "Correct battery/geometry fault and restart."
                : s.feetProtection
                  ? "Foot sensor can cancel sensor recovery and retain this pose."
                  : "Pose retained; enable Control and send fresh movement to continue."
            }`
          : s.result || "";
        // Do not hide an exit rejection behind the original HOLD reason.
        if (s.hold && s.result && s.result !== s.fault) $("notice").textContent += ` ${s.result}`;
        if (s.supportWaiting)
          $("notice").textContent += " Waiting for support; missing feet search down.";
        if (s.fallback) $("notice").textContent += " Slow one-leg wave fallback active.";
        if (s.paused)
          $("notice").textContent += ` Paused: ${s.paused}. Step rejected; change movement or height to continue.`;
        $("video").disabled = !s.cameraAvailable;
        // The Lift target button doubles as the current lift readout.
        if (s.lift !== undefined) $("adjust-lift").textContent = `Lift ${s.lift} mm`;
        $("arm").setAttribute("aria-pressed", String(enabled && s.armed));
        if (!actions.length) {
          // Color follows confirmed telemetry, not a tap/HTTP queue acknowledgement that may later be rejected.
          const fields = {
            mode: "mode",
            gait: "gait",
            height: "height",
            speed: "speed",
            lift: "lift",
            travel: "travel",
            balance: "balance",
            imu: "imu",
            feet: "feetProtection",
            leg: "leg",
            hold: "singleLegHold",
          };
          document.querySelectorAll("[data-action]").forEach((el) => {
            const value = s[fields[el.dataset.action]];
            if (value === undefined) return;
            if (el.hasAttribute("data-toggle")) {
              const on = Number(value) === Number(el.dataset.on ?? 1);
              el.setAttribute("aria-pressed", String(on));
              el.textContent = el.dataset.label;
            } else if (document.activeElement !== el && !el.picker?.open) {
              el.value =
                el.dataset.action === "speed"
                  ? String(100 - value)
                  : typeof value === "boolean"
                    ? String(Number(value))
                    : String(value);
              if (el.dataset.action === "speed") $("speed-value").textContent = `${el.value}%`;
              if (el.pickerSummary)
                // Manual body steps and folded startup need a height label even without a preset option.
                el.pickerSummary.textContent =
                  el.dataset.action === "height"
                    ? `${value} mm${Number(value) === 45 ? " creep" : ""}`
                    : el.selectedOptions[0]?.textContent;
            }
          });
        }
        if (
          enabled &&
          s.ready &&
          !s.armed &&
          !actions.length &&
          !flight &&
          Date.now() - enableSince > DISARM_GRACE_MS
        ) {
          // Allow time for explicit enable acknowledgement, but never silently stay armed after robot rejection.
          enabled = false;
          neutral();
          $("link").textContent = "Robot disarmed: enable explicitly after checking state";
        }
      } catch (e) {
        $("robot").textContent = "Telemetry unavailable: " + e.message;
      } finally {
        clearTimeout(timer);
        telemetryFlight = false;
      }
    }
    setInterval(updateTelemetry, TELEMETRY_INTERVAL_MS);
  </script>
</html>)HTML";
#endif
