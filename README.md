# Hexapod_1.0_Nano

## Firmware layout

- [Hex.h](Hex.h) is the small public interface: Arduino `setup()` and `loop()`
  declarations only. It can be included without defining state or pulling in
  hardware dependencies.
- [Hexapod_1_0_ESP32_v0.ino](Hexapod_1_0_ESP32_v0.ino) includes that interface.
- [Hex.cpp](Hex.cpp) owns runtime state, geometry tables, gait/kinematics,
  controller dispatch, hardware initialization, and motion scheduling. PlatformIO
  compiles this implementation once; do not include it from other files.
- [HexCfg.h](HexCfg.h) holds geometry, servo limits, and motion constants;
  [setup.h](setup.h) selects firmware features.
- Hardware drivers, foot sensing/landing, HTTP transport, and the browser page
  remain in their existing modules.

The separation itself changes code organization only. Host tests and simulation extract the actual functions from
[Hex.cpp](Hex.cpp).

## Motion timing and MG90S testing

Normal servo output is scheduled at most once per 20 ms instead of letting bus
traffic determine the loop-count-based smoother's speed. Contact walking keeps
its existing accepted-frame scheduler and immediate contact placement. Startup
still commands the folded home pose, with a 1 ms yield between positioning
attempts. Gait waypoints, offsets, 50 Hz PWM, and I2C clock speed are unchanged.
Only successful PWM writes are cached; stop, changed pulses, initialization,
and sleep invalidation still issue the required commands. Failed writes are
reported and retried rather than cached.
Each servo-output pass stages its commands, then writes consecutive changed
channels in blocks of at most seven channels (29 bytes including the register
pointer). Unchanged and unused channels break a block and are never overwritten.
The installed PCA9685 library enables register auto-increment during frequency
setup. For the current channel map, a fully changed 18-servo pose needs six
transfers instead of 18; sparse changes may use smaller blocks. Failed or
short-buffer transfers invalidate every affected channel so the next accepted
pose retries them. Transfers are not atomic across boards, and a bus failure
can leave a partially applied pose. Calibration and stop commands outside a
servo-output pass remain immediate single-channel writes.

Battery sampling starts a two-channel ADS1115 pair every 100 ms, without waiting
for conversion completion. Battery protection and all HTTP consumers share the
same complete pair; HTTP handlers never access I2C. Existing voltage thresholds
and stability checks remain, and only new pairs enter the stability history.
No complete pair, a conversion/bus error, or readings older than 500 ms block
motion. A conversion has a 50 ms deadline. Unavailable voltages are omitted
from controller status; `/battInfo` returns HTTP 503. A fresh successful pair
clears the ADC-data fault, but does not release a contact-walking HOLD.
I2C transactions themselves still take time and can time out on a faulty bus.
The control loop acknowledges an OTA request before battery or foot-sensor
servicing. Once OTA locks motion, both are suspended until reset, including after
a failed transfer. Flash stalls therefore do not trigger ADC timeout/staleness
buzzer alarms. Rejected OTA requests leave normal battery protection running.
Restart initializes sampling normally; this does not silence an existing
low-voltage alarm or change voltage thresholds.

The default [PlatformIO build](../platformio.ini) enables
`BUILDCFG_MOTION_TIMING_ENABLED=1`. Servo output emits at most one short Serial
summary of each type at most every five seconds, and only when the UART has
space for the whole line:

```text
Motion us: avg=20000 max=23000 work=8500 late=0 n=249
PWM us: avg=540 max=800 passAvg=6000 passMax=8500 writes=2800 fail=0
```

These numbers are illustrative, not measured robot results. `avg`/`max` are
intervals between output passes, `work` is the longest pass, `late` counts gaps
over 25000 microseconds, and `n` counts intervals. Skipped diagnostic messages
do not wait for UART space. These statistics measure software command timing,
not physical servo position or the PCA9685's actual pulse edges. Ignore the first
window after startup, an idle period, or HOLD when comparing continuous walking.
Set the build flag to `0` to disable these summaries.

The `PWM us:` line times actual single-channel `setPWM` calls or block transfers,
after the cache check and
before any error logging. `avg`/`max` measure one write, `passAvg`/`passMax`
measure the sum of write times in one servo-output pass (including zero-write
passes), `writes` counts attempts, and `fail` counts failed attempts. Cached or
invalid commands are not write attempts. A block is one attempt regardless of
its channel count, so per-write averages are not directly comparable with
pre-batching firmware; use `passAvg`/`passMax` and Motion intervals instead.
Direct calibration/stop writes enter
the write statistics but not the per-pass totals. Clock subtraction handles
`micros()` rollover. The two summary types have independent UART-space checks
and reporting windows, so compare steady trials rather than treating adjacent
lines as exactly matched frames. Compare `passMax` with Motion `work`: a similar
value suggests PWM/I2C calls consume most of the output pass. No timing is printed
after individual writes. This does not change servo commands or I2C speed.

`IMU/IK us:` reports `imuAvg`/`imuMax` for an entire PID update (MPU read,
angle access, and three PID calculations, excluding debug printing), and
`calls` counts those updates. `ikAvg`/`ikMax` time one complete six-leg
`doKinetic()` call, including any IMU updates inside it; `ikN` counts those
calls. Do not add IMU and IK times together: they overlap. Ordinary kinematics
samples IMU/PID once at the start of `doKinetic()`, even when browser IMU
correction is off, and all six legs use that snapshot. Previously each leg
sampled separately. Enabled correction intentionally uses one coherent pose
snapshot instead of six potentially different readings; manual rotations,
degree conversion, and PID settings are unchanged. Contact walking still
samples outside `doKinetic()` and reuses its existing cached corrections inside
it. Expect `calls / ikN` near one in steady ordinary walking, not six; contact
walking and other kinematics callers can produce a different ratio.
The IMU/IK line uses its own five-second UART-space gate and resets both
statistics only after a successful report. Gait and sensor enable behavior
are unchanged. Compare steady windows and ignore startup/idle transitions.

1. Support the robot securely. Upload over USB/programming adapter, keeping the
   servos on their normal external supply; USB must not power all 18 servos.
2. Open Serial at 115200 baud and control over Wi-Fi. Use the same gait, height,
   lift, travel input, and speed for each trial.
3. Walk steadily for at least 15 seconds, comparing camera off/on and foot
   protection off/on. Save timing lines and note any visible hesitation or
   protection/recovery messages. Fewer late frames do not prove smooth mechanics.
4. If hesitation remains under load, measure servo supply voltage during motion
   and check wiring, binding, and torque. Keep PWM at 50 Hz and do not alter
   calibration offsets as a timing fix.

Run `powershell -ExecutionPolicy Bypass -File test\motion\run.ps1` for the actual
ADC/battery/PWM drivers with simulated I2C, including errors, stale readings,
stop/re-enable, retries, timing thresholds, and clock rollover. Run
`powershell -ExecutionPolicy Bypass -File test\web_controller\run.ps1` for
motion-scheduler and controller regressions, and `pio run` to build firmware.
Hardware timing and MG90S response still require the trials above.

## Source formatting

Per-leg geometry and limit tables in [Hex.cpp](Hex.cpp) use RF, RM, RR, LF, LM,
LR order, matching the IDs in [HexCfg.h](HexCfg.h) and channel labels in
[servo.cpp](servo.cpp). The front/rear entries were previously reversed.
Aligning them changes the corner legs' commanded trajectories; it is not just
a naming change. Servo channels and calibration offsets have not been swapped.
The front/rear geometry values in [HexCfg.h](HexCfg.h) have subsequently been
swapped on each side to retain the pre-reorder channel geometry: front-labelled
legs use positive neutral/body Z and -60-degree coxa references; rear-labelled
legs use negative neutral/body Z and +60-degree references. IDs and calibration
remain in RF, RM, RR, LF, LM, LR order. This preserves the former numerical
geometry convention rather than independently establishing physical leg names.
Verify physical leg/channel and switch labels with the robot securely supported
before using this geometry on hardware. Simulation can compare commands but
cannot confirm the physical wiring or prove that this fixes a contact-aware HOLD.

The project [clang-format configuration](../.clang-format) uses two-space indentation,
Allman braces (opening braces on their own line), and explicit braces around
every `if`/`else` and loop body. Use clang-format 15 or newer for `InsertBraces`
support. Format C/C++ headers, source, and Arduino sketches using this configuration;
review preprocessor-dependent control flow, where automatic brace insertion may
be skipped. The embedded HTML/CSS/JavaScript retains its separate web formatting.

## Wi-Fi browser controller

Contact-aware walking interpolates horizontal foot coordinates and gait rotation
with the actual vertical descent progress. It does not apply the next horizontal
target while the foot is still at the preceding full-lift height. Raw-contact
pauses therefore pause both components. The nominal gait remains separate from
the output coordinates; pose retention stores the coordinates
actually issued. Joint/calibrated-servo limits and unreachable walking targets
pause or recover instead of HOLD (see below). Host regression tests exercise the real contact scheduler, FK, IK,
and calibration for 56 mm height / 30 mm lift in both travel directions; these
tests do not certify physical servo response or floor contact.

### Editing the page

[webPage.h](webPage.h) contains the complete page inside the `R"HTML(...)HTML"`
C++ raw string:

- `<style>`: colors, button sizes, grid spacing, and mobile layout.
- HTML below `</style>`: labels, buttons, selectors, joysticks, and camera.
- `<script>`: input handling, command delivery, and telemetry updates.
  Timing constants are grouped at the top; `update()` sends commands and
  `updateTelemetry()` displays the robot's actual state.

Keep element IDs and `data-action` values unchanged unless you also update
their JavaScript/firmware handlers. For toggle text, update both its visible
label and `data-label`, since telemetry restores the latter. Keep the raw-string
opening/closing delimiters intact. Rebuild/upload after changes on the robot;
restart the local preview and refresh the browser for preview-only checks.
With Camera on, video fills the clear panel between the joysticks without
stretching its aspect ratio. Settings occupy a separate full-width row below
the sticks and camera, never covering the image. Camera off closes the stream.

The default build enables `HTTP_ENABLED` and `WEB_CONTROLLER_ENABLED` in
[setup.h](setup.h). The ESP32 serves its own offline page: no app, CDN,
or internet connection is required. Phone and desktop
browsers use the same interface. Bluetooth Classic and MQTT implementations
are archived under [../legacy](../legacy) and are not part of the active
firmware or browser interface.
On touch phones the page attempts a landscape lock. If browser policy rejects
it, tap **Landscape** to request fullscreen and landscape orientation, or rotate
manually. Browser/OS support and user permission are required; an ordinary HTTP
page cannot guarantee rotation. Rotation never enables motion. Portrait remains
usable with smaller sticks flanking the camera and all settings below.

### First connection

After normal initialization, the active-low red LED (GPIO 33) blinks three
times (200 ms on, 100 ms off) and stays off to indicate initialization is done.
This adds 0.9 seconds to startup and does not run in isolated foot-test mode.

1. Support the robot securely before uploading/resetting. Existing folded
   startup positioning is unchanged; opening the page does not enable motion.
2. Open the USB Serial monitor at 115200 baud, newline/LF. Send one line:

   ```text
   NET {"ssid":"YOUR_ROUTER_SSID","password":"YOUR_ROUTER_PASSWORD"}
   ```

   SSID must be 1-32 bytes; password must be 8-63 bytes. Provision only while
   robot control is disabled. Reset after the `Network saved` message.
   Firmware does not echo credentials. Serial local echo defaults OFF in
   [platformio.ini](../platformio.ini); keep it off while provisioning.
3. Settings persist in ESP32 Preferences/NVS, not source code. Full flash
   erasure removes them. Storage is not encrypted; protect physical access.
   Router passwords shorter than eight characters/open networks are not supported.
4. Join either the router network or the `Hexapod-Control` hotspot (using the
   **stored hotspot password**). Router association is nonblocking; the
   hotspot stays available if the router is absent. On the hotspot open
   `http://192.168.4.1/`; on the router use the address printed on Serial.
   If a phone reports no internet, choose to stay connected.
5. Support the robot, then click **Control** to enable. The first normal enable
   after power-on selects the 56 mm standing preset and moves toward it.
   Check actual robot state before moving; select another validated height if
   desired. Foot protection remains OFF until selected.

No Wi-Fi credentials are compiled into this firmware. Before provisioning,
there is no intentionally open hotspot or default password. The web server
stays stopped until saved settings initialize Wi-Fi; an unconfigured first
boot stays running to accept the Serial `NET` command. Reset after saving.
The web server
uses plain HTTP and has no individual-user authentication: anyone on either
network can operate it after obtaining the controller lease. Use trusted
private networks only; do not forward ports 80/81 to the internet. The control
API rejects browser cross-origin JSON commands; this is not a substitute for
network access control.

### Servo calibration

Support the robot securely, keep clear of the joints, disable Control and wait
for motion to stop. Open **Servo calibration** from the controller. Start
calibration, select one leg (RF, RM, RR, LF, LM, LR) and joint, enter an offset
from -30 to 30 degrees in 0.1-degree steps, and explicitly move that servo to
its zero reference plus the offset. Adjust until the physical zero is correct.
Only the selected servo moves; selecting a different servo does not move it.

**Save all offsets** writes the staged values to the ESP32's EEPROM emulation
in flash and applies them to subsequent motion. **End calibration** discards
unsaved changes. Normal control and firmware updates are blocked during
calibration. If the page disconnects, reopen it to end calibration; a reset
also ends it but runs the normal startup positioning. Support the robot before
ending calibration or re-enabling control.

The saved offsets are loaded before startup positioning and used for motion
and servo-limit checks. On first boot or if the stored record is invalid, the
existing offsets in [HexCfg.h](HexCfg.h) are used; storage failures are reported
on Serial and a failed save is reported on the page. A full flash erase removes
the saved calibration.

### Changing router and hotspot settings

These are separate networks: `NET` selects the router the robot joins, not
the name the robot broadcasts. With Control disabled and the gait settled,
send either command through USB Serial (115200 baud, LF):

```text
NET {"ssid":"YOUR_ROUTER_NAME","password":"YOUR_ROUTER_PASSWORD"}
HOTSPOT {"ssid":"YOUR_ROBOT_NAME","password":"YOUR_ROBOT_PASSWORD"}
```

Each command changes only its own saved settings. Names must be 1-32 bytes
and passwords 8-63 bytes. Reset after the save confirmation, then reconnect
your phone to the new hotspot name/password. Its page stays at
`http://192.168.4.1/`. You can configure only HOTSPOT if no router is needed.
For compatibility, until HOTSPOT is configured, existing NET settings still
provide `Hexapod-Control` with the router password. Once HOTSPOT is saved,
changing NET no longer changes the hotspot password.

### Firmware updates over Wi-Fi (browser .bin upload)

The first OTA-enabled version **must be uploaded by USB/serial** using the
`esp32` environment. It installs the standard `default.csv` partition table with
two 1,310,720-byte application slots instead of the previous single large slot.
Do not OTA this version onto the old partition layout. Normal PlatformIO
uploads still use serial on COM4.
Back up any filesystem data before changing partitions.

Support the robot securely for every update: successful OTA restarts the robot
and runs its existing startup positioning. Disable **Control**, turn **Camera**
off, close other control tabs, and keep a stable power supply. Do not update
while walking. Given an unstable router connection, prefer the direct robot
hotspot and keep the computer connected despite the "no internet" warning.

**OTA has no password authentication and transfers are not encrypted. Anyone
who can reach the robot can install firmware while Control is disabled.**
Use only trusted private networks. The hotspot/router Wi-Fi passwords are
unchanged. The old Serial `OTA` command is no longer supported, and previously
saved OTA passwords are ignored. ArduinoOTA/espota and its UDP listener have been removed.
Install this browser-update version by USB/serial before using the update page.

For subsequent updates:

1. Build with `pio run -e esp32`.
2. Connect the computer/phone to the robot hotspot or the same router.
3. Open `http://192.168.4.1/update` on the hotspot, or
   `http://ROBOT_ROUTER_IP/update` on the router. The controller also has a
   **Firmware update (.bin)** link.
4. Select `.pio/build/esp32/firmware.bin` from the project. This must be the
   **application image built for this robot**, not `bootloader.bin`,
   `partitions.bin`, or a merged/full-flash image. On a phone, first copy the
   application file onto the phone.
5. Disable Control, wait until motion is disabled, turn Camera off, close other
   control tabs, and support the robot. Click **Upload firmware** and confirm.
6. Wait for the success message and automatic restart, then refresh the
   controller. Do not disconnect power or close the upload page during flashing.

Uploads use HTTP on port 80, without UDP invitations, reverse TCP connections,
mDNS, environment variables, or an `esp32-ota` environment. Do not expose the
robot's HTTP port to the internet. Invalid origins, content types, empty images,
and images larger than the inactive application slot are rejected before
flashing. ESP32 Update validates the image before selecting it for the next boot;
this is not signature authentication or proof that firmware is safe/correct.

The motion thread must acknowledge that both Control and motion are disabled
before flashing starts. Once a transfer starts, motion and command execution
remain locked until restart, including after a failed transfer. Reset the robot before retrying a
failed transfer; do not assume the previous upload succeeded. Successful
uploads reboot automatically; verify the success message on the update page and refresh the
controller page. Interrupted updates normally leave the previous application
bootable, but this is not power-loss rollback or validation of a newly booted
application. Keep serial flashing available for recovery.

### Controls and stopping

Two independent touch/pointer sticks and WASD/arrow keys supply the existing
four axes. Release, pointer cancellation, or key-up returns the affected input
to neutral. Space requests STOP; changing tab or losing window focus requests
disable. Hidden/closed pages stop heartbeats, so the firmware also detects
loss without depending on a final browser message.
Opening a native dropdown can blur a mobile browser without leaving the
page: that case centers inputs but does not disable Control. Actual tab
hiding and other focus loss still request disable.
The single **Control** button toggles enable/disable: green means this page
has requested control and the robot confirms it is armed; dark means not armed.
Its label stays fixed. Disabling preserves controlled stopping and servo torque.
The first normal enable from a settled, folded startup pose selects the 56 mm
standing preset. Subsequent enables preserve the current height and pose,
including after sensor-bypass recovery; an already-active or non-startup pose
is not reset by the first browser enable.
Camera sits beside Control, with STOP on the same compact command row.

The page exposes operation mode, all six gaits, standing-height presets,
speed percentage (higher is faster), lift, double travel, balance/IMU balance,
foot-protection toggle, single-leg selection/hold, and camera streaming.
Speed sets the requested gait delay, not a guaranteed physical speed. Both walking
schedulers accept speed changes while walking without requiring centered sticks
or settled feet. Speed changes do not replace the current pose; contact-aware
walking completes the current step with its original delay. Height, gait, and
other geometry settings still require centered controls and a settled gait.
Both walking
schedulers finish the smoothed servo commands and allow a full 50 Hz PWM frame at
the final target before advancing. Normal walking measures its delay after the
first servo output, excluding calculation and battery-read time; contact-aware
walking retains the current step's delay even if the speed slider changes.
Thus 100% means the fastest rate allowed by these timing checks, not skipping
unfinished targets. This checks commanded positions, not measured joint positions:
loaded or underpowered servos may still lag. Validate speed changes with the body
supported and monitor the servo supply.

Walking uses [servoMotion.h](servoMotion.h) to interpolate all airborne joints
from their last commanded angles on one elapsed-time cubic easing profile.
Different joint distances finish together instead of converging over a
loop-dependent number of smoothing updates. Acceleration and deceleration are
gentler at segment boundaries; this is joint-space easing, not a continuous
Cartesian trajectory with nonzero velocity through every gait boundary.
The requested interval reserves one final PWM frame. Larger moves extend the
profile so its peak commanded speed does not exceed the configured nominal
`SERVO_ANGLE_6V / SERVO_TIME_6V` rating. With the configured 60 degrees / 0.08 seconds,
a 90-degree move takes at least 180 ms plus the final PWM frame. This nominal
6 V rating is not a measured loaded-servo limit.

Both sensor-off walking and airborne segments of contact-aware walking use
this profile. Landing, search, seating, and supporting-foot commands still
bypass easing so contact can stop new descent commands immediately; changed
targets cancel stale segments. Nonwalking body-height, single-leg, startup,
and calibration updates retain the fixed-fraction smoother. The gait delay is
captured per step in both schedulers, so a speed-slider change cannot truncate
the current segment. Existing reach, calibrated joint, support, and HOLD
checks remain in place. Physical speed and smoothness still require hardware
validation.

Changing height can request one gait cycle to reposition the feet for the new
stance width. With centered controls, the legs then finish landing and stay
still; the settling state does not request additional lift cycles. Releasing
travel or pressing STOP likewise finishes recentering without restarting swings.

The web **Foot sensor** button enables only with centered inputs, a settled
gait, WALK mode, a standing height above the folded pose, no selected single
leg, valid sensor mapping, and a usable support polygon from stable CONTACT
readings (hardware-debounced by default; software debounce is optional). Four
or five well-distributed contacts can pass; contact count alone
is not sufficient. Otherwise it
stays dark and the status message asks you to check the sensors/settings.
Recheck the named contacts and click again after correcting the issue;
there is no delayed automatic activation. Green follows confirmed robot
telemetry only. This preflight does not guarantee valid geometry or support
through a later step. With centered inputs, clicking the green **Foot sensor**
button can cancel sensor-related HOLD/recovery even when Control is disarmed.
The browser clears held joystick/keyboard input and pending settings/STOP when
requesting this exit, and sends a neutral, disarmed command. A simultaneous STOP
does not block cancellation. Non-neutral requests from other clients are rejected
with an explicit message; exit rejection details remain visible alongside HOLD.
It retains torque and the last issued targets, disables Control, and clears
stored travel. Re-enable **Control**, then send fresh movement to continue
without contact checks. Battery/workspace/joint faults cannot be cleared this way.
Binary settings use fixed-label toggle buttons, including double
travel: green means enabled, the normal dark color means disabled. Button
labels do not append ON/OFF; `aria-pressed` exposes the state to screen readers.
Their highlighted/pressed state reflects robot telemetry, not an assumed
successful command; incompatible or rejected changes remain OFF/ON as
reported by the robot. Multi-choice settings retain selectors.
**Height** and **Lift** choose what the shared **+5 mm** / **-5 mm** buttons
change; choosing one sends nothing, and +/-5 does nothing until one is chosen.
Height steps the body 5 mm (`bodyUp`/`bodyDown`, 25-85 mm, including folding).
Lift steps leg lift 5 mm (`liftUp`/`liftDown`, `MIN_LEG_LIFT_MM` 25 to
`MAX_LEG_LIFT_MM` 50); the Lift button shows the current lift from telemetry.
Steps are relative so fast taps are not lost to stale telemetry. Lift changes
are ignored while creeping. As with the original buttons, manual heights other
than exactly 45 mm are not creep-restricted; avoid trying to walk in the folded
pose or unvalidated low heights.
The **Foot sensor** button controls contact-aware walking; grip controls are
not shown. Controls sit below the camera, between the joysticks;
buttons fit their labels, with reduced padding and a 32 px minimum height.
Operation, Gait, Height, and Single leg share one row; Height, Lift, +5 mm and
-5 mm share the next.
Double travel and other toggle buttons share a compact horizontal row, wrapping on narrow screens.
The **IMU active** label controls the existing IMU body-balance setting; it
does not power the IMU hardware on or off.
IMU readings and [PID corrections](pid.cpp) use degrees. [Body kinematics](Hex.cpp)
converts corrections to its tenths-of-a-degree convention for both standing and
cached contact-aware walking corrections. PID output is limited to +/-40 degrees,
preserving the intended original PID range. Correcting the units increases the
effective maximum from 4 to 40 degrees. Gains and axis mapping are unchanged. Verify correction
direction and leg travel with the robot securely supported before hardware use;
software tests do not establish physical stability.
The speed slider displays its current percentage and sends `100 - percentage`
as the existing firmware delay (0-100 ms). Zero percent selects the slowest
setting, not stop; this is not measured physical velocity.
A selected single leg must be deselected before contact-aware walking.
Single-leg behavior itself is otherwise unchanged.

Commands use serialized HTTP polling at a requested 50 ms interval, with at
most one in-flight command and no accumulating movement queue. This reuses
the existing ESP-IDF HTTP server instead of requiring WebSocket support.
Settings are validated and handed to the main loop; HTTP `accepted` means
queued, not mechanically completed. Actual settings and rejected-action
messages appear in telemetry, polled every 200 ms. Center sticks and let the
gait settle before changing settings. Creep restrictions still apply.

Only one controller owns input at a time. Another client gets HTTP 409 while
the 750 ms lease is active; merely viewing telemetry does not take ownership.
Temporary mailbox-busy replies carry `code: "busy"` and are retried without
immediately disabling Control or dropping a setting. Retries are bounded to
less than 600 ms since the last accepted command; persistent backpressure,
ownership conflicts, and network errors still disable control. The server's
750 ms watchdog is unchanged; rejected busy requests do not renew its lease.
The status notice above the controls shows rejected actions and HOLD reasons,
so a blocked safety state is not presented as a frozen page.
A command with no reply shows "Control interrupted: no reply from robot".
Telemetry carries a per-boot ID and the ESP32 reset reason (`boot`,
`resetReason`). When the ID changes, the page reports "Robot restarted
(reason)", for example a brownout from servo current, and drops its own
arming. That message stays until Control is enabled again.
If commands cease for 750 ms, or Disable is requested, stored travel is
zeroed and valid gait landing/settling finishes before control is disabled.
There is no automatic resume: a new explicit Enable action with centered
inputs is required. A late packet or a different owner's movement packet
cannot re-arm the robot. In single-leg mode, stopping also selects hold for the
selected leg only and zeros single-leg motion requests.

Operation/gait/height/leg menus open inside the page instead of native OS
dropdowns, so selecting a mode does not depend on popup focus behavior or
interrupt the command heartbeat. Opening a menu centers the inputs without
disarming. Actual window focus loss, hidden tabs, and connection loss still stop
control and require explicit re-enabling.
The Height menu displays the firmware-reported body-height target, including
folded startup and intermediate Body +10/-10 mm settings, even when no preset
matches. The four preset choices remain unchanged. This is a commanded height,
not a physical measurement from a ground-distance sensor.

For single-leg control, first choose **Operation: Single leg**, then select a
leg. Selection alone does not automatically lift it. Left-stick X/Y set the
leg's X/Z offsets; right-stick Y sets its vertical offset. All three are absolute
position offsets from that leg's standing position when first moved, not
velocities. Repeating a deflection cannot accumulate vertical movement. Each
axis retains its last position when its stick returns to center.

**Single-leg hold** is saved independently for each leg. The button shows the
selected leg's confirmed state (green = ON, dark = OFF); with None selected it
shows OFF, and a hold command requires a selected leg. For example, holding RF
does not hold RM: selecting RM shows RM's own state and permits moving RM if OFF.
Held legs retain their positions when selecting another leg, None, or another
operation mode. A leg/mode change returns only displaced legs whose own Hold is
OFF; other legs' holds remain unchanged. Turning a leg's Hold OFF alone does not
return it; a subsequent leg/mode change is required.
Returning uses the actual saved stance, not the folded startup coordinates.
Retaining a displaced leg across a mode change is intentional; it does not
guarantee that walking from that pose is stable. Support the robot when testing.

STOP zeros travel without cutting servo power. If a foot fault or automatic
recovery is active, STOP/disconnection freezes further targets instead of
continuing the retreat. Enable alone cannot clear that HOLD. Secure the body;
the Foot sensor button can cancel a sensor-related HOLD as described above.
Battery/geometry HOLD requires correcting the fault and restarting.
Fault-free gait stopping is controlled settling, not
instantaneous mechanical braking. PWM/servo torque remains active, and
previously queued/physical motion may continue. This is **not a hardware
emergency stop** or a safety-rated link; synchronous firmware/library calls
can delay servicing beyond the nominal timeout. Keep a physical power
disconnect available and test link loss with the body supported first.

Named foot contacts, landing/recovery state, last fault, battery cells,
network addresses, and actual settings are shown. Battery sampling runs on
the main thread; HTTP responses use a cached snapshot rather than accessing
I2C from the server task. Camera is optional MJPEG on port 81; Start camera
opens it, Stop camera closes the image request (not camera power). Test link
responsiveness while streaming before loaded operation.
With PSRAM, streaming uses QVGA **320x240**, JPEG quality 14 and one framebuffer;
without PSRAM it retains the smaller HQVGA fallback. Delivery is capped at
**10 frames/s** with a yielding wait after each frame. This conservative profile
fits the small central view on S10/S25 Ultra-sized layouts without paying for
the phone's full display resolution. It prioritizes Wi-Fi/control headroom over
fine image detail; actual frame rate and motion latency still need hardware
testing. Camera configuration is in [camera.cpp](camera.cpp).

Gait touchdown is based on a lifted foot's Y coordinate, not the sign of its
forward/backward Z displacement. Tripod 6's two-position swing lands at its
front-down phase rather than issuing an extra half-lift there. Regressions cover
every gait/leg and repeated mirrored forward/backward cycles. These fixes do not
certify every combination of stride, lift, height, and body correction as reachable.

### Legacy clients and tests

`LEGACY_HTTP_COMMAND_ENABLED` in [setup.h](setup.h) optionally restores
`POST /values` with seven integer bytes `value0`..`value6` and the original
checksum. It is OFF by default. With the web controller compiled, legacy
commands and browser commands cannot own input simultaneously. `/battInfo`
remains available with `cell1`/`cell2` (503 until the first cached sample).
The legacy packet protocol retains its old button and enable semantics;
the browser enable/timeout policy does not retrofit a legacy-client watchdog.

Host regression commands (MSVC Build Tools, Python, Node, and the existing
firmware ArduinoJson library required):

```powershell
.\test\foot_sensor\run.ps1
.\test\web_controller\run.ps1
```

The web tests execute the embedded browser script with mock inputs, extracted
actual main-loop dispatch, and the actual HTTP protocol implementation with
host HTTP/RTOS shims. They do not simulate servo physics or Wi-Fi timing.
For an offline browser-only layout fixture, run
`node .\test\web_controller\preview.js` and open `http://127.0.0.1:8765/`.
The fixture is loopback-only and never controls hardware.
It explicitly shows "Local preview - not connected to the robot" instead of
simulated network addresses. On the real robot, "Robot IP on router" means
the robot's own address on the router network, not the router's address.

## Standing-height coordinates

The folded home coordinate remains `CHexInitY = 25` mm. The height button
cycles absolute, neutral-body foot Y presets of 45 (creep), 56, 65, and 85 mm.
Internally `bodyPosY` and the controller height offsets are additional lift
above home: the corresponding offsets are 20, 31, 40, and 60 mm.
`MAX_BODY_Y` is the absolute maximum (85); `MAX_BODY_Y_OFFSET` limits added
lift (60). Body-down can still return to the folded pose separately.

Stance widths are 54, 59, 60, and 54 mm for those presets, selected using absolute height
thresholds. These rounded distances from each coxa pivot target a vertical tibia
in the neutral, level-ground pose with calibrated 25/35/65 mm links:
`XZ = coxa + sqrt(femur^2 - (height - tibia)^2)`.
Integer coordinates introduce a small angular error. The table does not maintain
vertical tibias during gait, body corrections, uneven contact, or intermediate
manual heights; those still use the existing preset-threshold selection.
These dimensions are not a guarantee that every walking, rotation,
or single-leg target is reachable; foot protection must be enabled separately.

Normal leg lift is `DEFAULT_LEG_LIFT_MM = 30` mm. The browser Lift +/-5 buttons
select 25-50 mm; the original controller's high-lift button still toggles
`HIGH_LEG_LIFT_MM = 40` mm and back to 25 mm.
These are operator-selected values, not a verified safe envelope; lifts above
40 mm have not been checked against reach at low heights. In particular,
the 30 mm default lift did not pass the host IK reach check at the 56 mm stance,
and 20 mm did not pass at the 45 mm creep height. The defaults are now 25 mm
and 15 mm respectively. A host geometry sweep passed the 45 mm creep profile
with 15 mm lift, translation capped at +/-16 mm, no turning, and neutral body
corrections. This is not physical validation. Test with the body supported;
contact-aware workspace/joint checks can HOLD rather than completing an
unreachable requested lift. Startup remains folded, and manual body-down can
still select heights below the first preset. Travel, turning, and balance
controls remain unrestricted outside the creep preset.

### Creep preset

In WALK mode, selecting the 45 mm preset activates creep: 8-step tripod gait (type 0),
15 mm lift (`CREEP_LEG_LIFT_MM`), X/Z travel scaled from joystick input and capped at +/-16 mm,
zero turning, and at least 250 ms per timed step. Double travel and high-lift
buttons are ignored while creeping, as are gait and balance toggles.
Translation, body rotation, rotation-center offsets, and temporary vertical
shift are neutralized on entry; balance corrections are disabled.
Select another operation mode to use its normal controls after stopping.

Center the sticks and allow the gait to finish settling before changing
height. Height changes while the gait is active are rejected. Leaving creep
via mode selection also requires settling. Selecting a higher height or
leaving WALK restores the previously selected normal gait, lift, and balance
settings; old body translations/rotations are not restored.
Manual heights other than exactly 45 mm retain normal behavior and are not
protected by the creep restrictions. Folded startup and single-leg control
are unchanged. Test with the body supported before placing weight on the
feet. Foot-contact landing is controlled separately by the walking feature below.

## Contact-aware walking

`FOOT_SENSOR_WALK_ENABLED` in [setup.h](setup.h) compiles contact-aware landing
for all WALK gaits and heights, including creep. Runtime protection starts OFF.
With sticks centered and the gait settled, newly press `INPUT_TIP_SENSOR`
(mask `0x8000`) to toggle protection. Repeated held-button packets do not toggle
it again. Enabling requires the same standing/support preflight as the web
button. Enabling while moving, landing, faulted, or recovering is rejected.
A centered press while enabled cancels sensor recovery/HOLD, but not a hard
battery/geometry fault. Before first activation the normal legacy walking
path is used when protection is off. After cancellation the pose-preserving
WALK scheduler remains active without contact checks. It requires
`FOOT_SENSOR_ENABLED`; it does not alter the isolated test or single-leg mode.
Normal WALK uses a nonblocking step/output schedule (20 ms output target)
instead of the old servo busy-wait, allowing command and switch servicing
during descent. The existing network/ADC/library calls are still synchronous;
this is not a hard real-time or safety-rated controller.

Choose a standing height while stationary first. The estimated body-center
projection must lie inside the convex hull of contacting feet with
`FOOT_WALK_SUPPORT_MARGIN_MM = 2` clearance. Before a lift, the same check
excludes the proposed swing feet. Three feet on one side do not qualify;
four or five distributed feet can. Positions are commanded estimates and the
center projection is an approximation, not a measured center of mass or load.

If a proposed lift would remove needed support, its gait targets/step are
rolled back. Any current airborne group lands first. Automatic slow-wave
fallback is OFF by default: `BUILDCFG_SLOW_WAVE_FALLBACK_ENABLED = 0` in
[setup.h](setup.h). The robot waits for support without changing its selected
gait. Set the flag to `1` to enable automatic fallback. Only between swings,
the controller selects the Wave 24 gait (one swing leg at a time), normal
20 mm maximum lift (`FALLBACK_LEG_LIFT_MM`), translation capped at 16 mm, no turning, and at least
`FOOT_WALK_FALLBACK_STEP_MS = 400` per step. The selected gait/lift are restored
after stopping and settling. This fallback can override creep's usual tripod;
it does not waive contact, support, workspace, or joint checks. If even one-leg
lifting is unsupported, no new lift is issued.
This flag does not disable support checks or the separate exact body-shift retreat
used by missing-ground recovery.

A supporting switch release has `FOOT_WALK_SUPPORT_GRACE_MS = 200` grace.
If the remaining support polygon is usable, a persistent release starts a
bounded downward search for that foot instead of immediately causing HOLD.
Searches block new gait steps until contact is resolved. Insufficient current
support blocks new gait steps but allows the existing bounded downward searches
to reach output; it no longer causes a separate HOLD after 200 ms.
Persistent missing contacts still end in HOLD at the search timeout; faulty
switches are not silently treated as load-bearing feet.

On lift-off a foot must demonstrate stable release; a switch stuck pressed is
not accepted as touchdown. Stable readings use hardware debounce by default;
set `BUILDCFG_FOOT_SENSOR_DEBOUNCE_ENABLED` to `1` in [setup.h](setup.h) to
add software debounce.
Descending gait segments interpolate toward the planned Y over the step
duration, including the first 50 Hz PWM frame on the initial descent output;
this avoids waiting for an extra scheduler cycle before the feet begin lowering.
Already-confirmed contact still prevents that initial advance. After a previously
released switch first reports raw contact, the
foot enters slow seating: at most `FOOT_WALK_SEAT_MM = 2` additional millimeters
at `FOOT_WALK_SEAT_MM_S = 2` mm/s, in 1 mm increments every 500 ms. Stable contact
stops seating immediately and latches the height; when software debounce is
enabled, contact that stabilizes during the 15 ms debounce receives no extra
seating movement.
This is not a forced 2 mm push after every touchdown. Bounce does not restart
the seating distance/time budget. If contact does not stabilize within the
seating duration plus any enabled debounce allowance, the robot enters sensor
HOLD rather than continuing downward or automatically reversing. Seating remains bounded
by the existing target-plus-10 mm search ceiling and search timeout, and every
output still passes workspace and joint checks; new swings require support.
A switch that never
released is not allowed to start seating.
The landed height offset persists through support until the next lift.
The next gait step is blocked until all pending descending feet have finished
their segment or confirmed touchdown at their final landing. A tripod landing
therefore waits for each of its three feet, not just the first switch.

At commanded nominal-ground arrival without contact, the foot searches down
at `FOOT_WALK_SEARCH_MM_S = 10`, with an extra-distance ceiling of
`FOOT_WALK_EXTRA_DOWN_MM = 10` and `FOOT_WALK_TIMEOUT_MS = 2000`
(seating: `FOOT_WALK_SEAT_MM = 2` at `FOOT_WALK_SEAT_MM_S = 5`). These are
starting values for MG90S servos: 10 mm/s keeps each 1 mm step (100 ms) longer
than contact detection (5 ms sample + optional 15 ms software debounce + 20 ms
output frame), so overtravel after touchdown stays within about 1 mm. Keep the timeout at least
`EXTRA_DOWN_MM / SEARCH_MM_S` seconds.
The timeout is measured from commanded nominal arrival, NOT measured servo
arrival. At 10 mm/s the 10 mm search takes about one second; scheduling/contact
pauses can make the timeout occur before the full distance is reached.
Reaching the extra-distance ceiling stops further descent but waits for that
same timeout, rather than raising an early missing-ground HOLD.
Support-loss searches use the same bounds from the retained support target;
first raw contact during a search also permits bounded seating.
Serial reports `Support search`, `Landing search`, and each `Foot search`
millimeter so the slow movement can be distinguished from a blocked target.
`Foot seating` identifies the separate slow switch-engagement movement.
Active landing/support targets bypass angle smoothing to avoid a queued
downward target after touchdown; actual servo motion/overtravel remains possible.
Gait/body/mode changes are ignored during a landing transaction. Centered
sticks can stop the stored travel request while landing completes.
Body-balance samples and balance corrections are frozen during the transaction.

Unresolved/stuck contact, insufficient supporting geometry, or invalid sensor
mapping cause `Foot landing HOLD` with a leg/reason. No more servo
targets are issued; PWM continues holding the last successfully issued pose.
No power-off occurs. This can prevent further commanded steps but cannot
guarantee support or prevent a fall.

An offset-corrected walking joint limit violation, or a walking target the leg
cannot reach because of the commanded height, body pose, travel, or lift, does
not HOLD. Only that frame is rejected: the gait, landing state, and joint angles
are restored, the last valid pose is held, and the web notice shows `Paused:
walking joint limit (leg)` or `Paused: unreachable walking target (leg)`. While
paused, no landing checks run and landing timers (descent, grace, search,
seating) are frozen. Changing movement, height, body pose, lift, or gait
resumes normal running, and landing checks continue from where they stopped.

If the target becomes unreachable only because a landing foot searched below
its planned height without finding ground (reachable without the search depth),
the target is not issued and the frame is rolled back. It is then handled as
missing ground (`no ground within leg reach`), exactly like the search timeout
below.

On a genuine missing-ground timeout (including a support foot lost at a table
edge), recovery does not require the prior travel direction or a recorded path.
The failed foot is retracted from its downward search to its last valid support
height. From actual foot geometry, the controller remembers each failed
outward side and selects a small retreat/placement direction that is not toward
any remembered side. Opposing failed sides therefore constrain the next move
to a perpendicular direction, rather than reversing toward an older failure.
Only the foot whose search actually timed out (plus feet already marked lost)
is a failure. Another foot that is still searching at that moment is not
remembered, retracted, or re-placed; it keeps searching and recovery waits for
it, so it becomes a failed side only if its own search later times out.
Each lost foot is handled sequentially, with bounded escalation per direction:

- **Step A:** choose the highest safe clearance lift from 20 down to 8 mm in
  2 mm decrements, checked with landing Y and the existing IK/calibration margins.
  Follow with an 8 mm inward placement and one ordinary bounded contact search.
  If no clearance (even 8 mm) is safe, skip the lift and enter Step B directly;
  an unsafe horizontal placement also enters Step B without issuing that target.
- **Step B (always enabled):** choose the same adaptive clearance, then tuck the
  lost foot inward along its measured hip-to-foot radius in separate **5 mm servo
  moves**, at regular step time, up to **15 mm total**. Stop at the first rejected
  increment, retaining the last safe position. Only then move the body away in 6 mm
  increments, up to 30 mm per direction, keeping planted feet fixed on the
  ground. Keep the lost foot at that lifted, tucked X/Z throughout the slide, with
  no intermediate contact searches. After the final increment (30 mm or the first
  rejected increment), lower straight down at the held X/Z and search once. A
  zero-distance slide skips this probe and escalates directly. If even the minimum
  clearance is unsafe, escalate to Step C/the next direction **without sliding an
  unlifted foot**. Every increment checks
  the planted-foot support polygon, a 5 mm femur-pivot reach margin, and 5 degree
  joint/calibration margins before issuing any motion. The first rejected
  increment stops shifting: the constant is a ceiling, not a promise of travel.
  Approximate radial room at body heights 45/56/65/85 mm is 14/31/34/13 mm;
  actual geometry and calibration determine the runtime limit.
- **Step C (optional):** `BUILDCFG_RECOVERY_WALK_AWAY_ENABLED=1` permits up to two
  additional cycles per direction: sequentially lift, re-center and contact-search
  each supporting foot, then shift the body up to another 15 mm (in checked
  increments). The lost foot stays at its validated clearance and held tuck through
  the whole cycle (no additional tuck beyond 15 mm), then lowers straight down and searches once at the end. The default is
  **1 (ON)**; setting it to **0** disables only these walk-away cycles.
- **Step D:** when those actions are exhausted, mark the direction tried and
  undo the outstanding body shift only if the return pose passes the same
  checks; otherwise retain the safe pose. Try the next direction avoiding every
  remembered failed side only while a lost foot remains. Otherwise complete a
  sequential neutral re-center instead of starting another placement. HOLD occurs when no supported safe action remains.

After placement, recovery ensures at least 8 mm of actual body displacement
along the selected direction, counting prior shifts rather than repeating them.
If a post-placement retreat is rejected at the reach/support limit with no lost
feet, stop translating and proceed directly to sequential neutral re-centering;
never reuse the completed-foot sentinel as a leg index. Invalid active-leg or
recovery-direction indices are guarded before any array access and cause safe
re-centering or a diagnostic HOLD. A blocked completion re-center ends in HOLD,
not an endless undo/retry loop.

It then re-centers **all** feet one at a time while the body keeps its retreat
position: first every support foot, then the recovered foot(s) last. Each foot
lifts by the highest clearance (20 down to 8 mm, like Step A) at which both the
lift and the lifted neutral target are safe, moves to that neutral target, then
does an ordinary bounded contact
search, so the robot drags itself away from the edge one leg at a time.
If undo completion still holds an airborne recovery foot (lifting clears its lost
bit), first finish lowering/searching that foot at its held X/Z before lifting
another support.
The final stance is neutral and the outstanding body offset is zero. Body shifts
and re-centering use regular stepping time, with no recovery 400 ms floor and
no wave-24 gait substitution. The separate optional slow-wave fallback retains
its own behavior; the operator's selected gait/lift are restored afterward.
Integer-millimeter targets round vector endpoints (not individual increments);
retreat rounds outward when necessary to preserve the minimum displacement.

A foot that loses ground during re-centering is another recovery loss: remember
its measured outward side and avoid it as well. If a previously recovered foot
still has no ground at neutral, escalate to further bounded shifting (with a
fresh shift budget, since the supports are already neutral) rather than
looping on the same successful inward placement. Eight compass candidates plus
up to two geometry-derived perpendicular candidates per remembered side keep
direction selection finite. Battery and mechanical/geometry hard faults are
never retried. Failed-side memory and retry accounting clear only after all six
contacts are stable and settled. Center sticks, then provide fresh movement to
resume normal walking.

The support polygon is a commanded-foot geometry check, not a measured
center-of-mass or force estimate. Servo lag, slipping, obstacles, uneven
surfaces, and switch faults can still make a valid target unsafe; automatic
recovery cannot guarantee stability.

After stopping, all sticks must be centered in a received command before fresh
movement is accepted. Commands cannot change mode, height, or direction during
automatic recovery. To cancel sensor-related HOLD/recovery, center the inputs
and press **Foot sensor** (or newly press TIP SENSOR). No new PWM target is
issued by cancellation; the last issued gait targets and landing offsets are
retained, not replaced with home coordinates. Control is disabled and queued
travel is cleared. Enable Control explicitly with centered inputs, then send
fresh movement; enabling alone does not move the legs. This resumes WALK
without contact checks, so secure the robot before using it. Only WALK/speed
settings are allowed in this pose-preserving bypass; after settling, Foot
sensor can be re-enabled if its support preflight passes, preserving offsets.
Battery and geometry/joint faults remain latched and require correction/restart.
There is no measured physical position, force, or center-of-mass feedback;
switch contact and a commanded support polygon cannot guarantee stability.

For initial validation use supported creep at 45 mm, short commands, and
surfaces under ALL feet. Verify multi-foot touchdown, no-surface HOLD, stuck
switch HOLD, sequential losses from every leg, below-support-depth retraction,
the single 20 mm clearance attempt and held-foot slide before one vertical probe, opposite-side perpendicular selection,
no travel/history prerequisite, bounded no-safe-direction HOLD, four/five-contact
support patterns, sensor HOLD cancellation and explicit resume, hard-fault
refusal, and that no next support group lifts before the pending landing is
resolved. Only proceed to loaded testing after
mechanical clearance/overtravel is confirmed. Higher gaits, high lift, larger
strides, turns, and balance-enabled walking can still violate geometry and
will HOLD rather than be made valid by the contact feature.

## Foot contact bring-up (PCF8575 + 74HC14)

One inverter per NO switch, with COM connected to ground, makes contact HIGH
at the PCF8575. Electrical level compatibility must already be verified.

`FOOT_SENSOR_ENABLED` in [setup.h](setup.h) enables polling and Serial
diagnostics at 115200 baud. This does not change walking or single-leg control.
Normal firmware retains its existing startup servo positioning.

### Identify the pins without test motion

For a sensor-only, supported-body session, enable `FOOT_SENSOR_TEST_ENABLED` in
[setup.h](setup.h) and leave all motion values zero in
[footSensorCfg.h](footSensorCfg.h). This test
firmware bypasses normal startup positioning, Wi-Fi, MPU, app commands, walking,
single-leg control, ADS1115 initialization, and battery monitoring. Provide and
monitor a suitable power supply externally during this isolated test.
The PWM drivers are initialized, but no servo targets
are issued until an accepted `POSITION` command. Support the body securely;
do not rely on the other five legs to hold it.

After mapping, stable-state changes print once as `RF: CONTACT` or `RF: RELEASED`.
Send `STATUS` to see all six named contact states. An invalid/unconfigured
mapping instead reports raw and stable physical bits in P05..P00 order
(leading zeros are omitted). Press each physical switch, then fill in its named
`FOOT_SENSOR_*_PIN` macro using bit numbers 0..5. Use -1 for an unknown input.
The verified physical assignments are RF=P00, RM=P01, RR=P02, LF=P05,
LM=P04, LR=P03. The mapped mask uses configured leg indices:
RF=0, RM=1, RR=2, LF=3, LM=4, LR=5.
All six assignments must be valid and unique before motion is allowed.

Polling is requested every 5 ms. Software debounce is off by default because
the sensors already provide hardware debounce; set
`BUILDCFG_FOOT_SENSOR_DEBOUNCE_ENABLED` to `1` in
[setup.h](setup.h) to require 15 ms of stable contact and release readings.
These are software targets, not hard real-time guarantees. I2C reads, Serial
output, and the normal walking loop can delay samples. Periodic diagnostics are
off by default; contact/release events are still printed. Enable `DEBUG_FOOT_SENSOR` in
[setup.h](setup.h) to print them every 500 ms instead.
The PCF8575 library read does not provide a separate bus-error status;
these switches are not a safety-rated interlock.

### Supported RF touchdown/search test

Keep `FOOT_SENSOR_TEST_ENABLED` enabled and configure measured safe values:

- `FOOT_TEST_START_Y_MM`: positive coxa-relative starting Y. Increasing Y lowers
  the foot. This is independent of the walking home position.
- `FOOT_TEST_GROUND_Y_MM`: expected surface Y, at or below the start position
  (numerically greater or equal). Defaults to the starting Y.
- `FOOT_TEST_EXTRA_DOWN_MM`: maximum distance below this test's ground reference.
  The maximum target Y is `FOOT_TEST_GROUND_Y_MM + FOOT_TEST_EXTRA_DOWN_MM`.
- `FOOT_TEST_SPEED_MM_S`: downward speed limit in mm/s.
- `FOOT_TEST_TIMEOUT_MS`: total timeout from `LOWER`, including contact pauses.

Zero start, travel, speed, or timeout locks out motion. Implementation ceilings
of 100 mm extra travel, 50 mm/s, and 60000 ms are software bounds, NOT measured
safe limits for this robot. Choose much smaller limits as appropriate after
checking geometry and mechanical clearances. Unreachable and out-of-limit
joint targets are rejected rather than silently clamped. `FOOT_TEST_LEG`
defaults to RF.

Send commands with a newline from the Serial monitor. Commands are
case-insensitive: `POSITION`, `position`, and `Position` are equivalent.

In PlatformIO, click inside the Serial Monitor terminal, type the command,
and press Enter. There is no separate input box. The project enables local
echo (`monitor_echo = yes`) and LF line endings (`monitor_eol = LF`).
Restart an already-open monitor to apply these settings. In the running
PlatformIO monitor, Ctrl+T followed by Ctrl+E also toggles local echo.

1. `STATUS`: inspect state and configuration lock.
2. `POSITION`: command ONLY the test leg to its configured start pose. This
   initial positioning is not speed-ramped; ensure the leg can move there
   unobstructed, then wait for it to settle.
3. Verify the switch is physically released. `LOWER` requires stable release.
4. `LOWER`: descend in 1 mm target increments at no more than the configured
   rate. Below the configured ground reference it continues into bounded search.
5. `STOP`: stop generating new targets and hold the last commanded pose.

Raw contact immediately pauses further target increments. Stable contact enters
`CONTACT`; no additional descent is generated. If contact bounces away, descent
resumes only after stable release, without a catch-up jump. Reaching
the distance/timeout bound without contact enters `NO_GROUND`. Joint rejection
enters `FAULT`. These terminal states hold the last
target; none automatically lifts, reverses, or removes servo torque.
Use an external power disconnect if required; Serial STOP is not an emergency
hardware stop. The servo may still finish moving toward its last target.

Repeat tests with a flat surface, a slightly lower surface, no surface,
switch bounce, a switch held pressed before LOWER, STOP during descent, and
joint-limit rejection. Confirm actual stopping distance and switch overtravel.
`POSITION` explicitly restarts a test after a terminal state.

Adaptive walking and whole-gait scheduling changes are not enabled in this
stage. The general recovery described above is limited to cautious contact
placement, bounded body shifts, and sequential neutral re-centering. Disable `FOOT_SENSOR_TEST_ENABLED`
to restore normal robot operation; contact sensing remains diagnostic only
there.

### Software validation

On Windows with MSVC C++ Build Tools installed, run
`powershell -ExecutionPolicy Bypass -File test\foot_sensor\run.ps1` from the
project root. The harness compiles the actual sensor and test-controller
sources with simulated time, switches, Serial, and servo callbacks. Test-only
travel limits are fixtures, not recommended hardware settings.
Run `python test\foot_sensor\check_creep.py` to additionally verify the actual
creep-profile entry/restoration functions, settling guard, and input scaling.

Build normal firmware with `pio run`. To compile the supported-body test
without changing the saved configuration, set PowerShell's
`$env:PLATFORMIO_BUILD_FLAGS='-DFOOT_SENSOR_TEST_ENABLED'` before `pio run`,
then remove that environment variable afterward. These checks do not validate
physical wiring, load support, servo overtravel, or actual movement.
