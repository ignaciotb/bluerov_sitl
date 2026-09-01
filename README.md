# bluerov_sitl

A ROS2 layer that subscribes to `mavros/rc/override` (`mavros_msgs/OverrideRCIn`)
and reproduces what **ArduSub's MANUAL mode** would do with those values on a
real **BlueROV2 Heavy** (`vectored_6dof` frame, 8 thrusters) — down to
per-thruster PWM, published on a ROS2 topic.

## Why this is structured the way it is

You asked for something that behaves as close as possible to the real vehicle,
using as much of the actual ArduSub code as possible. The one piece that
determines "how close to real" this gets — the 8x6 thruster mixing matrix in
`AP_Motors6DOF.cpp` — is **not hand-copied into this repo**. That file has a
documented history of real bugs
([ArduPilot/ardupilot#10924](https://github.com/ArduPilot/ardupilot/issues/10924):
"Motor matrix does not represent real vehicle"), so retyping constants from
memory or from a random blog post risks silently baking in a stale/wrong
matrix with no way for you to tell.

Instead:

1. `scripts/fetch_ardusub_reference.sh` vendors the **real** ArduSub source
   from `ArduPilot/ardupilot` (sparse-checkout of `ArduSub/`,
   `libraries/AP_Motors/`, `libraries/RC_Channel/`) into `reference/`.
2. `scripts/extract_motor_matrix.py` **parses** that real source and
   regenerates `config/motor_matrix_vectored_6dof.yaml` from it — the actual
   `add_motor_raw_6dof(...)` calls, `ROLL_PITCH_INPUT_MAX`, and RC deadzones,
   pulled straight out of the files, not retyped.
3. The ROS2 node loads that generated YAML at runtime. Point it at a newer
   ArduSub tag and re-run steps 1–2 to pick up firmware changes automatically.

This also means **I have not personally verified the exact current values**
of everything in this pipeline against upstream (I don't have a live
checkout in this environment) — see "What's verified vs. what needs your
review" below before trusting this for anything beyond a rough prototype.

## Quick start

```bash
# 1. Vendor the real ArduSub source (defaults to the ArduSub-stable tag)
./scripts/fetch_ardusub_reference.sh ArduSub-stable

# 2. Generate the real thruster matrix from it
python3 -m pip install pyyaml
python3 scripts/extract_motor_matrix.py \
    --ardupilot-root reference/ardupilot \
    --frame SUB_FRAME_VECTORED_6DOF \
    --out ros2_ws/src/bluerov_sitl/config/motor_matrix_vectored_6dof.yaml

# 3. Build the ROS2 package
cd ros2_ws
colcon build --packages-select bluerov_sitl
source install/setup.bash

# 4. Run it
ros2 launch bluerov_sitl bluerov_sitl.launch.py
```

With mavros (real vehicle, SITL, or just `ros2 topic pub`) publishing to
`mavros/rc/override` and `mavros/state`, the node publishes per-thruster PWM
on `bluerov_sitl/thruster_pwm` (`mavros_msgs/RCOut`), indexed exactly
like the real `mavros/rc/out` (index 0 = Main1/motor 1, ... index 7 =
Main8/motor 8).

For quick bench testing without real mavros, set
`assume_armed_for_testing: true` in the config and just publish
`OverrideRCIn` manually:

```bash
ros2 topic pub --once mavros/rc/override mavros_msgs/msg/OverrideRCIn \
  "{channels: [1500, 1900, 1500, 1500, 1500, 1500, 0, 0]}"
# channel[1] = roll (2nd position, 0-indexed) driven hard over -> watch
# bluerov_sitl/thruster_pwm move
```

## What's implemented, faithfully sourced

| Piece | Source | Confidence |
|---|---|---|
| Default RC channel mapping (ch1=pitch...ch6=lateral) | `ardusub.com/developers/rc-input-and-output.html` (matches `RCMAP_*` defaults) | High — directly documented |
| Override sentinels (`0`=ignore, `65535`=release) | `RC_Channels` override handling, confirmed via multiple ArduPilot issue threads | High |
| Per-axis deadzones (30us / 40us for yaw) | `ArduSub/radio.cpp`, `Sub::init_rc_in()` — also re-extracted live by the script | High — but **not used by MANUAL mode**, see below |
| MANUAL mode = direct passthrough, no stabilization, **no gain** | `ArduSub/mode_manual.cpp` — six `set_*()` calls, no PIDs | High — read from the vendored source |
| MANUAL uses `norm_input()`, so **no deadzone anywhere in this path** | `ArduSub/mode_manual.cpp` + `RC_Channel::norm_input()` | High — read from the vendored source |
| Throttle = `(norm_input() + 1) / 2`, centred on **trim** (not `set_range()`) | `ArduSub/mode_manual.cpp` | High — read from the vendored source |
| 8x6 thruster factor matrix | **Extracted at build time from your vendored checkout** | As good as your checkout |
| Bidirectional throttle mapping (`2*(x-0.5)`) | `AP_Motors_Class::get_throttle_bidirectional()` | High — confirmed in the vendored source |
| **Two-group** saturation: `{roll,pitch,throttle}` and `{yaw,forward,lateral}` normalized independently | `AP_Motors6DOF::output_armed_stabilizing_vectored_6dof()` | High — read from the vendored source |
| Linear thrust→PWM with per-side spans | `AP_Motors6DOF::calc_thrust_to_pwm()` | High — read from the vendored source |
| Per-motor direction (`MOT_n_DIRECTION`) | `AP_Motors6DOF::_motor_reverse[]` | Mechanism confirmed; **the values are vehicle-specific — read them off your hull** |
| Override staleness timeout (3s default) | General RC_Channels override-timeout pattern | **Needs your verification** — exact constant/param name has moved across releases |

## Flight modes

Modes live in `modes.py`, one class per ArduSub mode, each a pure function of
(RC channels, attitude, dt) → the six demands `AP_Motors6DOF` consumes. They
touch no ROS, so each one diffs cleanly against its upstream `.cpp`.

| mode | upstream | needs feedback? | status |
|---|---|---|---|
| `manual` | `ArduSub/mode_manual.cpp` | no (open loop) | **verified bit-exact against a real BlueROV2** |
| `stabilize` | `ArduSub/mode_stabilize.cpp` | yes — attitude + gyro | faithful to source, **not verified against hardware** |

Select with the `flight_mode` parameter, switchable at runtime:

```bash
ros2 param set /bluerov_sitl flight_mode stabilize
```

### STABILIZE

Pilot commands **lean angles** for roll/pitch and a **yaw rate**; the attitude
controller holds them. Throttle, forward and lateral still pass straight
through exactly as in MANUAL — the attitude loop only touches roll/pitch/yaw.

Heading behaviour follows `mode_stabilize.cpp:41-60`: while you are yawing it
tracks the rate; for 250 ms after you let go it commands zero yaw *rate* and
keeps re-latching the heading (this is what stops the vehicle springing back
after a fast yaw, since inertia carries it past where the stick stopped);
after that it holds the latched bearing absolutely.

**It is a closed loop and needs `/mavros/imu/data`** (parameter `imu_topic`)
for orientation and body rates. If that data is missing or older than
`attitude_timeout_s`, the node holds every thruster neutral and warns — it
does not fall back to MANUAL, because silently changing what the sticks mean
on a live vehicle is worse than stopping.

The controller in `attitude_control.py` is a port of `AC_AttitudeControl` /
`AC_AttitudeControl_Sub` / `AC_PID` / `sqrt_controller`, with every gain taken
from the vendored headers rather than retyped — `ANGLE_P 6.0`,
rate roll/pitch `P 0.135  I 0.090  D 0.0036  IMAX 0.444  FLT 30Hz`,
rate yaw `P 0.180  I 0.018  D 0.0  IMAX 0.222  FLT 5Hz`. Two Sub defaults make
the PID port exact rather than approximate: `srmax = 0` disables the slew
limiter, and `filt_E_hz = 0` makes the error filter a passthrough.

**What is deliberately not ported:** input shaping and rate feedforward. The
node takes `AC_AttitudeControl`'s own no-feedforward branch — the `else` of
`if (_rate_bf_ff_enabled)` — which is what the real firmware runs with
`ATC_RATE_FF_ENAB = 0`. Stock default is 1, so expect softer, less
anticipatory response than a stock vehicle. Notch filters, `PD_MAX`, `D_FF`
and EKF-reset handling are all inactive at Sub defaults and omitted.

**Not validated against hardware.** The MANUAL path was checked bit-exact
against a real recording; there is no STABILIZE recording, so treat STABILIZE
as "faithful to the source", not "verified against the vehicle". Getting a
short STABILIZE bag with roll/pitch/yaw disturbances is the single most
useful thing you can do to raise confidence here.

## What's explicitly out of scope

- Camera pan/tilt (ch7/8), lights (ch9/10), video switch (ch11)
- DEPTH HOLD / ALT_HOLD / POSHOLD / AUTO — these add `AC_PosControl` and a
  depth source on top of the attitude loop; neither is implemented here
- The althold lean-angle limiting in `update_althold_lean_angle_max()`
  (`lean_angle_max_cd` is a fixed parameter here)
- RC/GCS/battery/leak/EKF failsafes
- The `roll_pitch_toggle` joystick-button behavior from `ArduSub/joystick.cpp`

## Validated against a real vehicle

A comparison run — identical keyboard-teleop inputs sent to the real
BlueROV2 Heavy and to the sim, recorded in `rosbag2_2026_08_26-12_07_22` —
found four defects, all since fixed. Replaying that bag's
`/mavros/rc/override` through this package now reproduces the real
vehicle's `/mavros/rc/out` **bit-exactly on 468 of 469 samples (99.8%)**;
the one straggler is link latency at a key-repeat boundary. Before the
fixes it matched 39%.

What was wrong:

1. **A `pilot_gain` of 0.5 was applied to every axis.** ArduSub's pilot gain
   lives in `ArduSub/joystick.cpp` (`rpyScale = 0.4*gain`), where a *joystick*
   is translated into RC channel values — upstream of the RC override this
   node consumes. `mode_manual.cpp` never touches it, so applying it here
   double-counted. There is no gain parameter any more: if your pilot gain
   is not 100%, it is already baked into the PWM arriving on
   `mavros/rc/override`.
2. **The deadzone was being applied.** MANUAL mode calls `norm_input()`, not
   `norm_input_dz()`. A +100us stick is 0.25, not 70/370 = 0.189. This is
   also why yaw was off by a *different* amount than the other axes — its
   deadzone is 40us rather than 30us.
3. **Throttle used the `set_range()` 0..1 mapping.** MANUAL uses
   `(norm_input() + 1) / 2`, centred on **trim**. The two agree only when
   trim sits exactly at the midpoint of min/max — as it does at
   1100/1500/1900, which is why this one stayed latent rather than showing
   up in the bag.
4. **`_motor_reverse[]` was not implemented at all** — see below.

A fifth defect was fixed but is not visible in that bag: saturation was
scaling all 8 motors by a single global peak, where
`output_armed_stabilizing_vectored_6dof()` normalizes `{roll,pitch,throttle}`
and `{yaw,forward,lateral}` by their own independent maxima. Nothing in the
run saturated, so it never showed; it will diverge under combined
full-deflection input.

## The one value you must supply yourself

`motor_direction` in `config/bluerov2_heavy.yaml` is your vehicle's
`MOT_1_DIRECTION` … `MOT_8_DIRECTION`. It records how the props and ESCs on
*your* hull were wired, so unlike everything else here it cannot be
extracted from the ArduSub source — the firmware default of all `+1` will
silently invert whichever thrusters your vehicle has reversed.

The committed default `[-1, -1, 1, 1, -1, 1, 1, -1]` was solved for from the
comparison run above. Motors 1 and 2 are confirmed by three independent axes
(forward, lateral and yaw all agree). **Motors 5–8 rest on the throttle axis
alone**, because that run never pressed `x`/`c`/`g`/`v` — exercise roll and
pitch and re-check before trusting the vertical stack's signs.

## Still unverified

The **override staleness timeout**. Grep your vendored `RC_Channels.cpp` /
`RC_Channel.h` for the override timeout handling and set
`override_timeout_s` explicitly rather than relying on the 3.0s default.

## Layout

```
scripts/
  fetch_ardusub_reference.sh   # vendors real ArduSub source
  extract_motor_matrix.py      # parses it into config/*.yaml
reference/                     # (created by the fetch script, gitignored)
  ardupilot/...
ros2_ws/src/bluerov_sitl/
  bluerov_sitl/
    rc_channel.py              # RC_Channel-equivalent: override storage,
                                # deadzone, normalization
    motor_mixer_6dof.py        # AP_Motors6DOF-equivalent: weighted-sum
                                # mixing, saturation scaling, PWM conversion
    sitl_node.py        # the ROS2 node wiring it all together
  config/
    bluerov2_heavy.yaml # node parameters
    motor_matrix_vectored_6dof.yaml  # generated by extract_motor_matrix.py
  launch/bluerov_sitl.launch.py
  test/
    test_motor_mixer.py        # unit tests (synthetic fixture, not real numbers)
    test_rc_channel.py
    fixtures/synthetic_test_matrix.yaml
```

## Extending toward C++ / real-time parity

This is a Python (`rclpy`) prototype, chosen so the extraction script and
the node can share the same YAML format and iterate quickly. If you need
tighter timing parity with the 50-200Hz control loop the real Pixhawk/Navigator
runs at, the natural next step is a straight C++ port of `motor_mixer_6dof.py`
and `rc_channel.py` into an `rclcpp` node — the algorithms carry over 1:1,
only the RC data types and pub/sub boilerplate change.
