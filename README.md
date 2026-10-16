# Hexapod_1.0_Nano

## Standing-height coordinates

The folded home coordinate remains `CHexInitY = 25` mm. The height button
cycles absolute, neutral-body foot Y presets of 45 (creep), 56, 65, and 85 mm.
Internally `bodyPosY` and the controller height offsets are additional lift
above home: the corresponding offsets are 20, 31, 40, and 60 mm.
`MAX_BODY_Y` is the absolute maximum (85); `MAX_BODY_Y_OFFSET` limits added
lift (60). Body-down can still return to the folded pose separately.

Stance widths are 45, 45, 55, and 25 mm for those presets, selected using absolute height
thresholds. These dimensions are not a guarantee that every walking, rotation,
or single-leg target is reachable; terrain-aware walking is not enabled yet.

Normal leg lift is `DEFAULT_LEG_LIFT_MM = 15` mm; the high-lift button selects
`HIGH_LEG_LIFT_MM = 30` mm. Startup remains folded, and manual body-down can
still select heights below the first preset. Travel, turning, and balance
controls remain unrestricted outside the creep preset. The calculation that passed
used 15 mm lift, translation within +/-16 mm, no turning, and neutral body
corrections; neither high lift nor unrestricted controls are validated by it.

### Creep preset

In WALK mode, selecting the 45 mm preset activates creep: 8-step tripod gait (type 0),
15 mm lift, X/Z travel scaled from joystick input and capped at +/-16 mm,
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
it again. Enabling requires valid mapping. Toggle requests while moving,
landing, faulted, or recovering are rejected rather than bypassing protection.
The normal legacy walking path is used when protection is off. It requires
`FOOT_SENSOR_ENABLED`; it does not alter the isolated test or single-leg mode.
Normal WALK uses a nonblocking step/output schedule (20 ms output target)
instead of the old servo busy-wait, allowing command and switch servicing
during descent. The existing network/ADC/library calls are still synchronous;
this is not a hard real-time or safety-rated controller.

Choose a standing height while stationary first. Before the first gait step,
all six switches must show stable contact. On lift-off a foot must demonstrate
stable release; a switch stuck pressed is not accepted as touchdown.
Descending gait segments interpolate toward the planned Y over the step
duration. Raw contact pauses descent; stable contact latches that foot's height.
The landed height offset persists through support until the next lift.
The next gait step is blocked until all pending descending feet have finished
their segment or confirmed touchdown at their final landing. A tripod landing
therefore waits for each of its three feet, not just the first switch.

At commanded nominal-ground arrival without contact, the foot searches down
at `FOOT_WALK_SEARCH_MM_S = 1`, with an extra-distance ceiling of
`FOOT_WALK_EXTRA_DOWN_MM = 5` and `FOOT_WALK_TIMEOUT_MS = 3000`.
The timeout is measured from commanded nominal arrival, NOT measured servo
arrival. At 1 mm/s only about 2-3 mm of extra travel is possible before timeout,
so the 5 mm ceiling is not normally reached in one attempt.
Active landing/support targets bypass angle smoothing to avoid a queued
downward target after touchdown; actual servo motion/overtravel remains possible.
Gait/body/mode changes are ignored during a landing transaction. Centered
sticks can stop the stored travel request while landing completes.
Body-balance samples and balance corrections are frozen during the transaction.

Unstable/stuck contact, lost contact on a previously supporting
foot, invalid sensor mapping, unreachable walking IK, or offset-corrected joint
limit violations cause `Foot landing HOLD` with a leg/reason. No more servo
targets are issued; PWM continues holding the last successfully issued pose.
No power-off occurs. This can prevent further commanded steps but cannot
guarantee support or prevent a fall.

On a genuine missing-ground search timeout, the controller first attempts to
retrace the airborne feet's recorded commanded gait-coordinate paths, holding
the other foot targets. Per-foot history has 256 frames; overflow invalidates
it instead of silently using a truncated path. Returning to the old coordinates
is not enough: all six switches must indicate contact before opposite travel
starts. Path recovery has a 10-second timeout. These coordinates are requested
targets, not measured physical positions; servo lag, body motion, slipping,
and obstacles can invalidate a physical retrace.

If history is unavailable or restoration fails, opposite travel is permitted
only with all six contacts; otherwise recovery enters HOLD. With support
restored, X/Z translation and rotation reverse the last nonzero travel request.
The controller requests three complete gait periods of opposite travel, then
zeros travel and finishes settling the feet. Each reverse target still passes
landing, workspace, and joint-limit checks. A new fault during retreat holds
instead of recursively reversing again. Foot placement/settling may require
additional steps beyond those three periods.

After stopping, all sticks must be centered in a received command before fresh
movement is accepted. Commands cannot change mode, height, or direction during
automatic recovery. An unrecoverable HOLD requires securing the body and
restarting; the TIP SENSOR toggle does not clear it or waive checks.
There is no physical position feedback, force sensing, or support-polygon check.

For initial validation use supported creep at 45 mm, short commands, and
surfaces under ALL feet. Verify multi-foot touchdown, no-surface HOLD, stuck
switch HOLD, missing-history HOLD, recorded-path recovery, three-cycle opposite
travel and fresh-input interlock, and that no next support group lifts
before the pending landing is resolved. Only proceed to loaded testing after
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

After mapping, debounced changes print once as `RF: CONTACT` or `RF: RELEASED`.
Send `STATUS` to see all six named contact states. An invalid/unconfigured
mapping instead reports raw and debounced physical bits in P05..P00 order
(leading zeros are omitted). Press each physical switch, then fill in its named
`FOOT_SENSOR_*_PIN` macro using bit numbers 0..5. Use -1 for an unknown input.
The verified physical assignments are RF=P00, RM=P01, RR=P02, LF=P05,
LM=P04, LR=P03. The mapped mask uses configured leg indices:
RF=0, RM=1, RR=2, LF=3, LM=4, LR=5.
All six assignments must be valid and unique before motion is allowed.

Polling is requested every 5 ms, with 15 ms contact/release debounce. These
are software targets, not hard real-time guarantees. I2C reads, Serial output,
and the normal walking loop can delay samples. Periodic diagnostics are off
by default; contact/release events are still printed. Enable `DEBUG_FOOT_SENSOR` in
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

Raw contact immediately pauses further target increments. Debounced contact
enters `CONTACT`; no additional descent is generated. If contact bounces away,
descent resumes only after stable release, without a catch-up jump. Reaching
the distance/timeout bound without contact enters `NO_GROUND`. Joint rejection
enters `FAULT`. These terminal states hold the last
target; none automatically lifts, reverses, or removes servo torque.
Use an external power disconnect if required; Serial STOP is not an emergency
hardware stop. The servo may still finish moving toward its last target.

Repeat tests with a flat surface, a slightly lower surface, no surface,
switch bounce, a switch held pressed before LOWER, STOP during descent, and
joint-limit rejection. Confirm actual stopping distance and switch overtravel.
`POSITION` explicitly restarts a test after a terminal state.

Adaptive walking, whole-gait scheduling changes, and automatic retreat are
not enabled in this stage. Disable `FOOT_SENSOR_TEST_ENABLED` to restore
normal robot operation. Contact sensing remains diagnostic only there.

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
