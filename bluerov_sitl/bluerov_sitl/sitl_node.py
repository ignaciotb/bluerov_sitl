"""
sitl_node.py

A software-in-the-loop stand-in for the BlueROV2 Heavy's flight
controller. Subscribes to mavros/rc/override (mavros_msgs/OverrideRCIn) --
exactly what mavros forwards to a real flight controller as a MAVLink
RC_CHANNELS_OVERRIDE message -- and reproduces what ArduSub would do with
it, publishing the per-thruster PWM the real vehicle would drive.

Flight modes live in modes.py, one class per ArduSub mode, selected by the
`flight_mode` parameter and switchable at runtime. The pipeline below is
MANUAL; STABILIZE inserts the AC_AttitudeControl cascade between the RC
channels and the mixer, and needs attitude feedback to close the loop.

    mavros/rc/override
          |  (RC_Channels::set_override() equivalent, with the same
          |   0=ignore / 65535=release sentinels + staleness timeout)
          v
    per-axis RC_Channel::norm_input() -- NO deadzone
          |  (channel 1=pitch, 2=roll, 3=throttle, 4=yaw, 5=forward,
          |   6=lateral by ArduSub's default RCMAP_*; see
          |   ardusub.com/developers/rc-input-and-output.html)
          v
    MANUAL mode: direct pass-through, no stabilization and no gain --
    ArduSub/mode_manual.cpp is literally six set_*() calls
          |
          v
    AP_Motors6DOF weighted-sum mixing for the vectored_6dof frame
    (SixDofMixer, matrix auto-extracted from the real source)
          |
          v
    per-thruster PWM, published as mavros_msgs/RCIn on
    /BlueROV2_Heavy/mavros/rc/in -- channel i is motor i+1, so it lines
    up index-for-index with the real vehicle's mavros/rc/out

Explicitly OUT OF SCOPE for this node (real ArduSub does more than this
in MANUAL mode -- add these yourself if you need them):
  - camera pan/tilt (channels 7-8), lights (9-10), video switch (11)
  - the roll_pitch_toggle joystick-button behaviour
  - any mode other than MANUAL (STABILIZE/DEPTH HOLD/etc. run pilot
    input through attitude/depth PID controllers first -- see
    ArduSub/mode.cpp upstream)
  - RC failsafe / EKF / battery / leak failsafes

A NOTE ON GAIN: ArduSub's pilot "gain" (JS_GAIN / g.gain_default) is
applied in ArduSub/joystick.cpp, where a JOYSTICK is translated into RC
channel values ("rpyScale = 0.4*gain"). It happens UPSTREAM of the RC
override this node consumes, and mode_manual.cpp never touches it.
Applying a gain here would double-count it. This node deliberately has no
gain parameter; if your pilot gain is not 100%, it is already baked into
the PWM values arriving on mavros/rc/override.

A NOTE ON MOTOR DIRECTION: the motor_direction parameter is the one thing
here that CANNOT be derived from the ArduSub source -- it is the
MOT_1_DIRECTION..MOT_8_DIRECTION parameters of your particular vehicle,
recording how that hull's props and ESCs were wired. Read them off the
vehicle (e.g. via QGroundControl's parameter list, or
"ros2 service call /mavros/param/get ...") and set them here. The default
is all +1, matching the firmware default -- which will be WRONG for any
vehicle that has reversed thrusters.
"""
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from mavros_msgs.msg import OverrideRCIn, RCIn
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import SetParametersResult

try:
    from mavros_msgs.msg import State as MavrosState
    _HAVE_MAVROS_STATE = True
except ImportError:  # pragma: no cover
    _HAVE_MAVROS_STATE = False

from std_msgs.msg import Header

from bluerov_sitl.rc_channel import RCChannel, ChannelMode
from bluerov_sitl.motor_mixer_6dof import SixDofMixer, MixerConfig
from bluerov_sitl.attitude_control import (
    Quat, flu_enu_to_frd_ned, flu_to_frd_rates,
)
from bluerov_sitl.modes import MODES, ManualMode, StabilizeMode

# Default channel mapping, per ArduSub's documented RCMAP_* defaults
# (ardusub.com/developers/rc-input-and-output.html). 1-indexed to match
# that documentation; converted to a 0-indexed OverrideRCIn.channels[]
# lookup internally.
DEFAULT_CHANNEL_MAP = {
    "pitch": 1,
    "roll": 2,
    "throttle": 3,
    "yaw": 4,
    "forward": 5,
    "lateral": 6,
}


class BlueRovSitlNode(Node):
    def __init__(self):
        super().__init__("bluerov_sitl")

        self._declare_parameters()
        self._load_parameters()

        matrix_path = self._resolve_motor_matrix_path(
            self.get_parameter("motor_matrix_path").value
        )
        self.mixer = SixDofMixer(
            matrix_yaml_path=matrix_path,
            mixer_config=MixerConfig(
                pwm_min=self.get_parameter("motor_pwm_min").value,
                pwm_trim=self.get_parameter("motor_pwm_trim").value,
                pwm_max=self.get_parameter("motor_pwm_max").value,
            ),
        )
        # Validated against the matrix, so it has to happen after the
        # mixer has loaded it.
        self.mixer.mixer_config.motor_direction = self._resolve_motor_direction()

        reversed_motors = [
            n for n, d in sorted(self.mixer.mixer_config.motor_direction.items())
            if d == -1
        ]
        if reversed_motors:
            self.get_logger().info(f"Reversed motors (MOT_n_DIRECTION=-1): {reversed_motors}")
        else:
            self.get_logger().warn(
                "motor_direction is all +1. That is the ArduSub default, but "
                "most real BlueROV2 builds reverse at least one thruster -- "
                "if you have not read MOT_1_DIRECTION..MOT_8_DIRECTION off "
                "your vehicle, this node's output signs are unverified."
            )

        self.channels = self._build_rc_channels()

        self._armed = False
        self._assume_armed = self.get_parameter("assume_armed_for_testing").value
        if self._assume_armed:
            self.get_logger().warn(
                "assume_armed_for_testing=true -- this node will output "
                "non-neutral PWM without ever seeing mavros/state.armed. "
                "Do not use this against real hardware."
            )

        self.override_sub = self.create_subscription(
            OverrideRCIn, "mavros/rc/override", self._on_override, 10
        )

        if _HAVE_MAVROS_STATE:
            self.state_sub = self.create_subscription(
                MavrosState, "mavros/state", self._on_state, 10
            )
        else:
            self.get_logger().warn(
                "mavros_msgs/State not importable -- arming gate will rely "
                "solely on assume_armed_for_testing."
            )

        self.pwm_pub = self.create_publisher(RCIn, "/BlueROV2_Heavy/mavros/rc/in", 10)

        # --- flight modes -------------------------------------------------
        mode_params = {
            "yaw_scale": self.yaw_scale,
            "acro_yaw_p": self.get_parameter("acro_yaw_p").value,
            "lean_angle_max_cd": self.get_parameter("lean_angle_max_cd").value,
            "pilot_gain": self.get_parameter("pilot_gain").value,
        }
        self.modes = {name: cls(self.channels, mode_params) for name, cls in MODES.items()}
        self._mode_name = None
        self._last_tick_s = None

        # STABILIZE feedback. Subscribed unconditionally so switching modes at
        # runtime does not have to build subscriptions on the fly.
        self._attitude_stamp_s = None
        self._attitude_timeout = float(self.get_parameter("attitude_timeout_s").value)
        imu_topic = self.get_parameter("imu_topic").value
        self._imu_frame = str(self.get_parameter("imu_frame").value).lower()
        if self._imu_frame not in ("ros", "ardupilot"):
            raise ValueError(
                f"imu_frame must be 'ros' or 'ardupilot', got "
                f"{self._imu_frame!r}")
        self.imu_sub = self.create_subscription(Imu, imu_topic, self._on_imu, 10)

        self._set_mode(self.get_parameter("flight_mode").value, initial=True)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        rate_hz = self.get_parameter("publish_rate_hz").value
        self.timer = self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f"bluerov_sitl up. Motors: {self.mixer.motor_numbers()}. "
            f"Mode: {self._mode_name}. Listening on mavros/rc/override "
            f"(+ {imu_topic} for stabilize), publishing "
            f"/BlueROV2_Heavy/mavros/rc/in at {rate_hz} Hz."
        )

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        self.declare_parameter("motor_matrix_path", "")
        self.declare_parameter("motor_pwm_min", 1100)
        self.declare_parameter("motor_pwm_trim", 1500)
        self.declare_parameter("motor_pwm_max", 1900)

        self.declare_parameter("rc_pwm_min", 1100)
        self.declare_parameter("rc_pwm_trim", 1500)
        self.declare_parameter("rc_pwm_max", 1900)
        self.declare_parameter("override_timeout_s", 3.0)

        # Deadzones are NOT used by MANUAL mode (mode_manual.cpp calls
        # norm_input(), not norm_input_dz()). They are still declared and
        # plumbed into the RCChannels so the stabilized modes remain
        # implementable without re-plumbing the parameter surface.
        self.declare_parameter("deadzone_pitch_us", 30)
        self.declare_parameter("deadzone_roll_us", 30)
        self.declare_parameter("deadzone_throttle_us", 30)
        self.declare_parameter("deadzone_yaw_us", 40)
        self.declare_parameter("deadzone_forward_us", 30)
        self.declare_parameter("deadzone_lateral_us", 30)

        self.declare_parameter("channel_pitch", DEFAULT_CHANNEL_MAP["pitch"])
        self.declare_parameter("channel_roll", DEFAULT_CHANNEL_MAP["roll"])
        self.declare_parameter("channel_throttle", DEFAULT_CHANNEL_MAP["throttle"])
        self.declare_parameter("channel_yaw", DEFAULT_CHANNEL_MAP["yaw"])
        self.declare_parameter("channel_forward", DEFAULT_CHANNEL_MAP["forward"])
        self.declare_parameter("channel_lateral", DEFAULT_CHANNEL_MAP["lateral"])

        # Per-motor direction (+1/-1), one entry per motor in the matrix,
        # ordered by motor number. Mirrors AP_Motors6DOF::_motor_reverse[]
        # / MOT_<n>_DIRECTION. See the "A NOTE ON MOTOR DIRECTION" block
        # at the top of this file -- the default is NOT safe to assume.
        self.declare_parameter("motor_direction", [1, 1, 1, 1, 1, 1, 1, 1])

        # ArduSub/mode_manual.cpp scales yaw by g.acro_yaw_p / ACRO_YAW_P.
        # Both default to ACRO_YAW_P (3.375, ArduSub/config.h), so the
        # ratio is 1.0 unless you have retuned ACRO_YAW_P on the vehicle.
        self.declare_parameter("yaw_scale", 1.0)

        # ------------------------------------------------------------------
        # Flight mode. Runtime-settable:
        #     ros2 param set /bluerov_sitl flight_mode stabilize
        # ------------------------------------------------------------------
        self.declare_parameter("flight_mode", ManualMode.name)

        # STABILIZE only -- it is a closed loop and needs the vehicle's
        # attitude. MANUAL ignores all of these.
        self.declare_parameter("imu_topic", "mavros/imu/data")
        self.declare_parameter("attitude_timeout_s", 0.5)

        # Frame of the incoming IMU attitude and rates:
        #   "ros"       -- REP-103: world ENU, body FLU. What mavros
        #                  publishes, for a real vehicle AND for the sim.
        #   "ardupilot" -- world NED, body FRD. Only if something upstream
        #                  has already converted for you.
        # AP_Motors6DOF's matrix is in ArduPilot's convention, so a "ros"
        # attitude MUST be converted or pitch and yaw feedback come in
        # inverted and the vehicle is driven away from level. See
        # flu_enu_to_frd_ned().
        self.declare_parameter("imu_frame", "ros")

        # ArduSub/config.h:113 ACRO_YAW_P. Scales the pilot's yaw stick into
        # a target yaw RATE in STABILIZE (Sub::get_pilot_desired_yaw_rate).
        self.declare_parameter("acro_yaw_p", 3.375)

        # ATC_ANGLE_MAX, centidegrees. ArduSub/config.h:152 DEFAULT_ANGLE_MAX.
        # Caps the lean angle the pilot can command in STABILIZE.
        self.declare_parameter("lean_angle_max_cd", 4500.0)

        # ArduSub/Parameters.cpp:304 JS_GAIN_DEFAULT, clamped 0.1..1.0 in
        # joystick.cpp. READ THE NOTE ON GAIN ABOVE: this does NOT scale any
        # output. Its only effect in this node is shrinking the yaw deadzone
        # in STABILIZE (mode_stabilize.cpp:35). MANUAL never reads it.
        self.declare_parameter("pilot_gain", 0.5)

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("assume_armed_for_testing", False)

    def _load_parameters(self):
        self.yaw_scale = self.get_parameter("yaw_scale").value

    def _resolve_motor_matrix_path(self, configured_path: str) -> str:
        """
        `ros2 launch` runs this node with whatever CWD the terminal
        happened to have, NOT the package's install directory -- so a bare
        relative path like "config/motor_matrix_vectored_6dof.yaml" (the
        default in config/bluerov2_heavy.yaml) will almost never
        resolve on its own. Resolve it properly:

          1. empty string  -> <share>/config/motor_matrix_vectored_6dof.yaml
          2. absolute path -> used as-is
          3. relative path -> tried against CWD first (keeps the
             "run straight from the repo during development" workflow
             working), then against <share>/<relative_path> if that fails
        """
        share_dir = None
        try:
            share_dir = Path(get_package_share_directory("bluerov_sitl"))
        except PackageNotFoundError:
            self.get_logger().warn(
                "Package 'bluerov_sitl' not found via "
                "ament_index -- are you running this outside a sourced "
                "install/setup.bash? Falling back to CWD-relative "
                "resolution only."
            )

        candidates = []

        if not configured_path:
            if share_dir is not None:
                candidates.append(share_dir / "config" / "motor_matrix_vectored_6dof.yaml")
        else:
            p = Path(configured_path)
            if p.is_absolute():
                candidates.append(p)
            else:
                candidates.append(Path.cwd() / p)
                if share_dir is not None:
                    candidates.append(share_dir / p)

        for c in candidates:
            if c.is_file():
                self.get_logger().info(f"Using motor matrix: {c}")
                return str(c)

        tried = "\n  ".join(str(c) for c in candidates) or "(no candidates -- package share dir unavailable)"
        raise FileNotFoundError(
            f"Could not find the motor matrix YAML. Tried:\n  {tried}\n\n"
            f"This file is GENERATED, not shipped -- if you haven't yet, run:\n"
            f"  ./scripts/fetch_ardusub_reference.sh\n"
            f"  python3 scripts/extract_motor_matrix.py --ardupilot-root reference/ardupilot "
            f"--frame SUB_FRAME_VECTORED_6DOF "
            f"--out ros2_ws/src/bluerov_sitl/config/motor_matrix_vectored_6dof.yaml\n\n"
            f"If you already ran that: with ament_python, colcon only copies "
            f"config/*.yaml at BUILD time, so if you generated the file after "
            f"your last build it won't be in the install share dir yet -- "
            f"re-run 'colcon build --packages-select bluerov_sitl' "
            f"and re-source install/setup.bash."
        )

    def _resolve_motor_direction(self):
        """
        motor_direction is a flat list ordered by motor number; the mixer
        wants {motor_number: +-1}. Validate the length against the matrix
        rather than silently zipping a short list, since a wrong-length
        list would quietly leave later thrusters at +1.
        """
        values = list(self.get_parameter("motor_direction").value)
        motor_numbers = self.mixer.motor_numbers()

        if len(values) != len(motor_numbers):
            raise ValueError(
                f"motor_direction has {len(values)} entries but the motor "
                f"matrix defines {len(motor_numbers)} motors "
                f"{motor_numbers}. Give exactly one +1/-1 per motor, in "
                f"motor-number order."
            )

        bad = [(n, v) for n, v in zip(motor_numbers, values) if v not in (1, -1)]
        if bad:
            raise ValueError(
                f"motor_direction entries must be +1 or -1; got {bad} "
                f"(motor, value)."
            )

        return {n: int(v) for n, v in zip(motor_numbers, values)}

    def _build_rc_channels(self):
        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        rc_min = gp("rc_pwm_min")
        rc_trim = gp("rc_pwm_trim")
        rc_max = gp("rc_pwm_max")
        timeout = gp("override_timeout_s")

        def make(name, mode, dz):
            return RCChannel(
                name=name,
                mode=mode,
                pwm_min=rc_min,
                pwm_trim=rc_trim,
                pwm_max=rc_max,
                deadzone_us=dz,
                override_timeout_s=timeout,
            )

        return {
            "pitch": make("pitch", ChannelMode.ANGLE, gp("deadzone_pitch_us")),
            "roll": make("roll", ChannelMode.ANGLE, gp("deadzone_roll_us")),
            # ANGLE, not RANGE: MANUAL mode reads throttle as
            # (norm_input() + 1) / 2, i.e. symmetric about TRIM. See
            # RCChannel.throttle_0_to_1().
            "throttle": make("throttle", ChannelMode.ANGLE, gp("deadzone_throttle_us")),
            "yaw": make("yaw", ChannelMode.ANGLE, gp("deadzone_yaw_us")),
            "forward": make("forward", ChannelMode.ANGLE, gp("deadzone_forward_us")),
            "lateral": make("lateral", ChannelMode.ANGLE, gp("deadzone_lateral_us")),
        }

    def _channel_index_map(self):
        """axis name -> 0-indexed position in OverrideRCIn.channels[]"""
        return {
            "pitch": self.get_parameter("channel_pitch").value - 1,
            "roll": self.get_parameter("channel_roll").value - 1,
            "throttle": self.get_parameter("channel_throttle").value - 1,
            "yaw": self.get_parameter("channel_yaw").value - 1,
            "forward": self.get_parameter("channel_forward").value - 1,
            "lateral": self.get_parameter("channel_lateral").value - 1,
        }

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # Flight mode
    # ------------------------------------------------------------------
    def _set_mode(self, name: str, initial: bool = False) -> None:
        if name not in self.modes:
            raise ValueError(
                f"unknown flight_mode {name!r}; known modes: "
                f"{sorted(self.modes)}"
            )
        if name == self._mode_name:
            return
        self._mode_name = name
        mode = self.modes[name]
        mode.on_enter(time.monotonic())
        if not initial:
            self.get_logger().info(f"flight_mode -> {name}")
        if name == StabilizeMode.name and self._attitude_stamp_s is None:
            self.get_logger().warn(
                "STABILIZE selected but no attitude received yet on "
                f"'{self.get_parameter('imu_topic').value}'. Thrusters stay "
                "neutral until it arrives."
            )

    def _on_set_parameters(self, params):
        """Allows `ros2 param set <node> flight_mode stabilize` at runtime."""
        for p in params:
            if p.name != "flight_mode":
                continue
            if p.value not in self.modes:
                return SetParametersResult(
                    successful=False,
                    reason=f"unknown flight_mode {p.value!r}; "
                           f"known: {sorted(self.modes)}")
            self._set_mode(p.value)
        return SetParametersResult(successful=True)

    def _on_imu(self, msg: Imu):
        """
        Attitude feedback for STABILIZE, converted into the frame the mixer
        and the attitude controller both use (ArduPilot NED/FRD).

        Getting this wrong does not degrade gracefully: ROS and ArduPilot
        agree on roll but have OPPOSITE pitch and yaw, so an unconverted ROS
        attitude makes the controller drive those two axes away from the
        target until the vehicle tumbles.
        """
        q = Quat(msg.orientation.w, msg.orientation.x,
                 msg.orientation.y, msg.orientation.z)
        gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

        if self._imu_frame == "ros":
            q = flu_enu_to_frd_ned(q)
            gyro = flu_to_frd_rates(gyro)

        self.modes[StabilizeMode.name].set_attitude(q, gyro)
        self._attitude_stamp_s = time.monotonic()

    def _attitude_is_fresh(self, now: float) -> bool:
        return (self._attitude_stamp_s is not None
                and (now - self._attitude_stamp_s) <= self._attitude_timeout)

    def _on_override(self, msg: OverrideRCIn):
        now = time.monotonic()
        idx_map = self._channel_index_map()
        for axis, idx in idx_map.items():
            if idx < 0 or idx >= len(msg.channels):
                continue
            self.channels[axis].apply_override(msg.channels[idx], now)

    def _on_state(self, msg):  # msg: mavros_msgs/State
        self._armed = bool(msg.armed)

    def _on_timer(self):
        now = time.monotonic()

        # Real dt, not the nominal timer period: the rate PIDs and the yaw
        # target integrator both need the interval that actually elapsed, and
        # a Python timer does not hit its period exactly.
        dt = 1.0 / self.get_parameter("publish_rate_hz").value
        if self._last_tick_s is not None:
            measured = now - self._last_tick_s
            if 0.0 < measured < 1.0:
                dt = measured
        self._last_tick_s = now

        armed = self._armed or self._assume_armed
        mode = self.modes[self._mode_name]

        # STABILIZE without fresh attitude must not drive the thrusters --
        # stale attitude means the loop is closing on a lie. Fall back to
        # neutral rather than to MANUAL: silently switching modes on a live
        # vehicle would be worse than stopping.
        if isinstance(mode, StabilizeMode) and not self._attitude_is_fresh(now):
            if armed:
                self.get_logger().warn(
                    "STABILIZE: attitude stale or missing -- holding neutral.",
                    throttle_duration_sec=2.0)
            mode.on_disarmed(now)
            armed = False

        demands = mode.run(now, dt, armed)

        pwm_by_motor = self.mixer.mix(
            roll=demands.roll,
            pitch=demands.pitch,
            yaw=demands.yaw,
            throttle_0_to_1=demands.throttle_0_to_1,
            forward=demands.forward,
            lateral=demands.lateral,
            armed=armed,
        )

        out = RCIn()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        max_motor = max(pwm_by_motor.keys())
        channels = [self.mixer.mixer_config.pwm_trim] * max_motor
        for motor_num, pwm in pwm_by_motor.items():
            channels[motor_num - 1] = pwm
        out.channels = channels

        self.pwm_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BlueRovSitlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
