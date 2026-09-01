"""
modes.py

ArduSub flight modes, one class per mode, each turning the current RC
channel state into the six demands AP_Motors6DOF consumes.

    FlightMode          -- shared interface + arming/disarm handling
      ManualMode        -- ArduSub/mode_manual.cpp
      StabilizeMode     -- ArduSub/mode_stabilize.cpp

A mode is a pure function of (RC channels, vehicle attitude, dt) -> demands.
It does not touch ROS: the node owns the subscriptions and the mixer, and
calls run() once per control iteration. That keeps each mode diffable
against its upstream .cpp, which is the whole point of this package.

Adding a mode: subclass FlightMode, implement run(), and register it in
MODES at the bottom. ALT_HOLD/DEPTH_HOLD would additionally need
AC_PosControl and a depth source, neither of which is vendored here.
"""
import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from bluerov_sitl.attitude_control import (
    AttitudeControlSub,
    Quat,
    SubAttitudeDefaults,
    constrain,
)

# ArduSub/config.h:149 -- roll/pitch/yaw RC input range in centidegrees
ROLL_PITCH_INPUT_MAX = 4500.0
# ArduSub/config.h:152 -- DEFAULT_ANGLE_MAX, the ATC_ANGLE_MAX default
DEFAULT_ANGLE_MAX_CD = 4500.0
# ArduSub/config.h:113 -- ACRO_YAW_P, also the g.acro_yaw_p default
ACRO_YAW_P = 3.375
# ArduSub/Parameters.cpp:304 -- JS_GAIN_DEFAULT, clamped to 0.1..1.0 in
# joystick.cpp:43. Only used by STABILIZE, and only to shrink the yaw
# deadzone (mode_stabilize.cpp:35) -- it does NOT scale any output here.
JS_GAIN_DEFAULT = 0.5
# mode_stabilize.cpp:50 -- grace period before heading hold re-latches
YAW_INPUT_GRACE_MS = 250


@dataclass
class Demands:
    """The six inputs to AP_Motors6DOF, in the units the mixer expects."""
    roll: float = 0.0            # -1..+1
    pitch: float = 0.0           # -1..+1
    yaw: float = 0.0             # -1..+1
    throttle_0_to_1: float = 0.5  # 0..1, 0.5 = neutral
    forward: float = 0.0         # -1..+1
    lateral: float = 0.0         # -1..+1


def get_pilot_desired_lean_angles(roll_in: float, pitch_in: float,
                                  angle_max: float,
                                  lean_angle_max_cd: float) -> Tuple[float, float]:
    """
    Direct port of Sub::get_pilot_desired_lean_angles()
    (ArduSub/Attitude.cpp:5). Inputs and outputs are centidegrees.

    The circular limit keeps combined roll+pitch inside angle_max rather
    than letting the corners reach angle_max * sqrt(2), and the final line
    converts a lateral tilt into the equivalent euler roll.
    """
    angle_max = constrain(angle_max, 1000.0, lean_angle_max_cd)

    scaler = lean_angle_max_cd / ROLL_PITCH_INPUT_MAX
    roll_in *= scaler
    pitch_in *= scaler

    total_in = math.hypot(pitch_in, roll_in)
    if total_in > angle_max:
        ratio = angle_max / total_in
        roll_in *= ratio
        pitch_in *= ratio

    # lateral tilt -> euler roll
    roll_in = (18000.0 / math.pi) * math.atan(
        math.cos(pitch_in * (math.pi / 18000.0)) * math.tan(roll_in * (math.pi / 18000.0)))

    return roll_in, pitch_in


class FlightMode:
    """
    Base class. `channels` is the axis-name -> RCChannel dict the node owns.

    Subclasses implement _run(); the base handles the disarmed case, which
    every ArduSub mode opens with ("if not armed, ... return").
    """

    name = "base"

    def __init__(self, channels: Dict, params: Optional[Dict] = None):
        self.channels = channels
        self.params = params or {}

    def on_enter(self, now_s: float) -> None:
        """ModeX::init() equivalent. Called when this mode becomes active."""

    def on_disarmed(self, now_s: float) -> None:
        """Called every iteration while disarmed, so state can be held reset."""

    def run(self, now_s: float, dt: float, armed: bool) -> Demands:
        if not armed:
            self.on_disarmed(now_s)
            return Demands()
        return self._run(now_s, dt)

    def _run(self, now_s: float, dt: float) -> Demands:
        raise NotImplementedError


class ManualMode(FlightMode):
    """
    ArduSub/mode_manual.cpp -- direct passthrough, no stabilization, no gain.

    Verified bit-exact (468/469 samples) against a real BlueROV2 Heavy
    recording; see README. Uses norm_input(), NOT norm_input_dz(): there is
    no deadzone anywhere in this path.
    """

    name = "manual"

    def _run(self, now_s: float, dt: float) -> Demands:
        ch = self.channels
        yaw_scale = self.params.get("yaw_scale", 1.0)
        return Demands(
            roll=ch["roll"].norm_input(now_s),
            pitch=ch["pitch"].norm_input(now_s),
            yaw=ch["yaw"].norm_input(now_s) * yaw_scale,
            throttle_0_to_1=ch["throttle"].throttle_0_to_1(now_s),
            forward=ch["forward"].norm_input(now_s),
            lateral=ch["lateral"].norm_input(now_s),
        )


class StabilizeMode(FlightMode):
    """
    ArduSub/mode_stabilize.cpp -- pilot commands LEAN ANGLES for roll/pitch
    and a YAW RATE, and the attitude controller holds them. Throttle,
    forward and lateral still pass straight through, exactly as in MANUAL.

    Heading behaviour, per mode_stabilize.cpp:41-60:
      - while the pilot is commanding yaw, track the rate and keep
        remembering the current heading
      - for 250 ms after the pilot stops, command zero yaw RATE (decelerate)
        and keep re-latching the heading -- this is what stops the vehicle
        springing back after a fast yaw, since inertia carries it past where
        the stick stopped
      - after that, hold the latched heading as an absolute bearing

    NEEDS ATTITUDE FEEDBACK. Call set_attitude() before run(), or the mode
    raises -- silently stabilizing against a stale or default attitude would
    drive the thrusters on bad data.
    """

    name = "stabilize"

    def __init__(self, channels: Dict, params: Optional[Dict] = None,
                 attitude_defaults: Optional[SubAttitudeDefaults] = None):
        super().__init__(channels, params)
        self.ac = AttitudeControlSub(attitude_defaults)
        self._have_attitude = False
        self._yaw_rad = 0.0
        self._gyro = (0.0, 0.0, 0.0)
        self._last_pilot_heading_rad = 0.0
        self._last_pilot_yaw_input_s = -1e9

    def set_attitude(self, q: Quat, gyro: Tuple[float, float, float]) -> None:
        """Feed the current body-to-NED attitude and body angular rates."""
        self.ac.set_attitude_body(q)
        self._yaw_rad = q.to_euler()[2]
        self._gyro = gyro
        self._have_attitude = True

    def on_enter(self, now_s: float) -> None:
        # ModeStabilize::init() -- latch the heading we are pointing at.
        self._last_pilot_heading_rad = self._yaw_rad
        self._last_pilot_yaw_input_s = -1e9
        self.ac.relax_attitude_controllers()

    def on_disarmed(self, now_s: float) -> None:
        # mode_stabilize.cpp:19-25
        self.ac.relax_attitude_controllers()
        self._last_pilot_heading_rad = self._yaw_rad

    def _run(self, now_s: float, dt: float) -> Demands:
        if not self._have_attitude:
            raise RuntimeError(
                "StabilizeMode.run() called without attitude feedback. "
                "STABILIZE is a closed loop -- call set_attitude() with a "
                "fresh orientation and gyro before running it."
            )

        ch = self.channels
        gain = self.params.get("pilot_gain", JS_GAIN_DEFAULT)
        acro_yaw_p = self.params.get("acro_yaw_p", ACRO_YAW_P)
        lean_max_cd = self.params.get("lean_angle_max_cd", DEFAULT_ANGLE_MAX_CD)

        # mode_stabilize.cpp:32 -- pilot input to target lean angles (cd)
        target_roll_cd, target_pitch_cd = get_pilot_desired_lean_angles(
            ch["roll"].get_control_in(now_s),
            ch["pitch"].get_control_in(now_s),
            lean_max_cd, lean_max_cd)

        # mode_stabilize.cpp:35-36 -- note the deadzone is scaled by the
        # pilot gain here. Unlike MANUAL, gain IS on this path.
        yaw_input = ch["yaw"].pwm_to_angle_dz_trim(
            int(round(ch["yaw"].deadzone_us * gain)), ch["yaw"].pwm_trim, now_s)
        target_yaw_rate_cds = yaw_input * acro_yaw_p

        target_roll = math.radians(target_roll_cd / 100.0)
        target_pitch = math.radians(target_pitch_cd / 100.0)
        target_yaw_rate = math.radians(target_yaw_rate_cds / 100.0)

        if target_yaw_rate != 0.0:
            self.ac.input_euler_angle_roll_pitch_euler_rate_yaw(
                target_roll, target_pitch, target_yaw_rate, dt)
            self._last_pilot_heading_rad = self._yaw_rad
            self._last_pilot_yaw_input_s = now_s
        elif now_s < self._last_pilot_yaw_input_s + YAW_INPUT_GRACE_MS / 1000.0:
            # Decelerate on yaw, keep re-latching the heading.
            self.ac.input_euler_angle_roll_pitch_euler_rate_yaw(
                target_roll, target_pitch, 0.0, dt)
            self._last_pilot_heading_rad = self._yaw_rad
        else:
            self.ac.input_euler_angle_roll_pitch_yaw(
                target_roll, target_pitch, self._last_pilot_heading_rad, dt)

        roll_out, pitch_out, yaw_out = self.ac.rate_controller_run(self._gyro, dt)

        # mode_stabilize.cpp:63-68 -- throttle/forward/lateral unchanged from
        # MANUAL. apply_angle_boost is false, so throttle is untouched.
        return Demands(
            roll=roll_out,
            pitch=pitch_out,
            yaw=yaw_out,
            throttle_0_to_1=ch["throttle"].throttle_0_to_1(now_s),
            forward=ch["forward"].norm_input(now_s),
            lateral=ch["lateral"].norm_input(now_s),
        )


MODES = {ManualMode.name: ManualMode, StabilizeMode.name: StabilizeMode}
