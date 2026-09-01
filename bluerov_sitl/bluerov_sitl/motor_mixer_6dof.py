"""
motor_mixer_6dof.py

Reimplements the thruster-level mixing done by AP_Motors6DOF for a
"vectored_6dof" frame (BlueROV2 Heavy, 8 thrusters).

Pipeline (see libraries/AP_Motors/AP_Motors6DOF.cpp upstream):

  1. Each motor has 6 fixed factors (roll, pitch, yaw, throttle, forward,
     lateral), assigned per-frame in setup_motors() via
     add_motor_raw_6dof(motor_num, roll_fac, pitch_fac, yaw_fac,
     throttle_fac, forward_fac, lat_fac, testing_order). These are loaded
     here from config/motor_matrix_vectored_6dof.yaml, which is
     AUTO-GENERATED from the real source by scripts/extract_motor_matrix.py
     -- see that script and scripts/fetch_ardusub_reference.sh. This
     mixer intentionally does not hardcode the matrix so it can't drift
     from whatever ArduSub version you extracted it from.

  2. The six axes are split into TWO INDEPENDENT GROUPS, which is what
     makes the vectored_6dof frame different from the generic 6dof one.
     Per output_armed_stabilizing_vectored_6dof():

         rpt[i] = roll*roll_factor[i] + pitch*pitch_factor[i]
                                      + throttle*throttle_factor[i]
         yfl[i] = yaw*yaw_factor[i]   + forward*forward_factor[i]
                                      + lateral*lateral_factor[i]

     "rpt" only ever lands on the vertical thrusters and "yfl" only on the
     translational ones, because no motor in this frame has factors in
     both groups. throttle here is the 0..1 throttle stick remapped to
     -1..+1 about its center (AP_Motors_Class::get_throttle_bidirectional()),
     since climb/descend needs a sign but RC throttle input doesn't carry
     one.

  3. Each group is normalized by ITS OWN peak, and only if that peak
     exceeds 1.0 -- upstream initializes both maxima to 1 precisely so a
     non-saturating group is left untouched:

         out[i] = rpt[i]/rpt_max + yfl[i]/yfl_max

     Note this is NOT a single scale factor across all 8 motors: a
     saturated vertical stack does not attenuate the horizontal thrusters
     and vice versa. Getting this wrong only shows up under combined
     full-deflection input, which is why issue #10924
     (github.com/ArduPilot/ardupilot/issues/10924) took a while to
     surface. Diff against your vendored
     libraries/AP_Motors/AP_Motors6DOF.cpp after any re-fetch.

  4. Each motor's result is multiplied by its direction (+1/-1), matching
     _motor_reverse[i] -- the MOT_1_DIRECTION..MOT_8_DIRECTION parameters
     on the real vehicle. These are per-BUILD, not per-frame: they encode
     which way the props/ESCs on THAT hull were actually wired, so they
     are not extractable from the ArduSub source and must be read off the
     vehicle. Leaving them all at +1 (the firmware default) will silently
     invert whichever thrusters your vehicle has reversed.

  5. The result is constrained to -1..+1 and converted to a PWM in
     [pwm_min, pwm_max] around pwm_trim (bidirectional ESC convention:
     1500=stopped, 1100=full reverse, 1900=full forward by default),
     mirroring AP_Motors6DOF::calc_thrust_to_pwm().

  6. If not armed, every motor outputs pwm_trim regardless of demand --
     matching the real vehicle never spinning thrusters while disarmed.
"""
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List

import yaml


@dataclass
class MotorFactors:
    motor: int
    roll_factor: float
    pitch_factor: float
    yaw_factor: float
    throttle_factor: float
    forward_factor: float
    lateral_factor: float


@dataclass
class MixerConfig:
    pwm_min: int = 1100
    pwm_trim: int = 1500
    pwm_max: int = 1900
    # motor number (1-indexed) -> +1 or -1, mirroring AP_Motors6DOF's
    # _motor_reverse[] / the MOT_<n>_DIRECTION parameters. Motors absent
    # from this dict default to +1, the firmware default.
    motor_direction: Dict[int, int] = field(default_factory=dict)

    def direction_for(self, motor_num: int) -> int:
        d = self.motor_direction.get(motor_num, 1)
        if d not in (1, -1):
            raise ValueError(
                f"motor_direction[{motor_num}] must be +1 or -1, got {d!r}"
            )
        return d


class SixDofMixer:
    def __init__(self, matrix_yaml_path: str, mixer_config: MixerConfig = None):
        self.mixer_config = mixer_config or MixerConfig()
        self.motors: List[MotorFactors] = self._load_matrix(matrix_yaml_path)

    @staticmethod
    def _load_matrix(path: str) -> List[MotorFactors]:
        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(
                f"Motor matrix config not found at {p}. Run "
                f"scripts/fetch_ardusub_reference.sh then "
                f"scripts/extract_motor_matrix.py to generate it from the "
                f"real ArduSub source before running this node."
            )
        with open(p) as f:
            doc = yaml.safe_load(f)
        motors = [MotorFactors(**m) for m in doc["motors"]]
        if not motors:
            raise ValueError(f"No motors found in {p}")
        return motors

    @staticmethod
    def throttle_bidirectional(throttle_0_to_1: float) -> float:
        """Mirrors AP_Motors_Class::get_throttle_bidirectional()."""
        val = 2.0 * (throttle_0_to_1 - 0.5)
        return max(-1.0, min(1.0, val))

    def mix(
        self,
        roll: float,
        pitch: float,
        yaw: float,
        throttle_0_to_1: float,
        forward: float,
        lateral: float,
        armed: bool,
    ) -> Dict[int, int]:
        """
        Returns {motor_number: pwm_microseconds} for every configured motor.
        roll/pitch/yaw/forward/lateral are expected in -1..+1.
        throttle_0_to_1 is expected in 0..1 (RC "range" channel convention).
        """
        cfg = self.mixer_config

        if not armed:
            return {m.motor: cfg.pwm_trim for m in self.motors}

        throttle_bidir = self.throttle_bidirectional(throttle_0_to_1)

        # Two independent mixing groups (docstring step 2). Both maxima
        # start at 1.0 so a group that never saturates is passed through
        # untouched -- do not initialize these to 0.
        rpt, yfl = {}, {}
        rpt_max, yfl_max = 1.0, 1.0
        for m in self.motors:
            rpt[m.motor] = (
                m.roll_factor * roll
                + m.pitch_factor * pitch
                + m.throttle_factor * throttle_bidir
            )
            yfl[m.motor] = (
                m.yaw_factor * yaw
                + m.forward_factor * forward
                + m.lateral_factor * lateral
            )
            rpt_max = max(rpt_max, abs(rpt[m.motor]))
            yfl_max = max(yfl_max, abs(yfl[m.motor]))

        pwm_out = {}
        for m in self.motors:
            val = cfg.direction_for(m.motor) * (
                rpt[m.motor] / rpt_max + yfl[m.motor] / yfl_max
            )
            val = max(-1.0, min(1.0, val))
            # AP_Motors6DOF::calc_thrust_to_pwm(): the two sides of trim
            # are scaled by their own span, so an asymmetric PWM range
            # stays continuous through zero.
            if val >= 0:
                pwm = cfg.pwm_trim + val * (cfg.pwm_max - cfg.pwm_trim)
            else:
                pwm = cfg.pwm_trim + val * (cfg.pwm_trim - cfg.pwm_min)
            pwm_out[m.motor] = int(round(pwm))

        return pwm_out

    def motor_numbers(self) -> List[int]:
        return [m.motor for m in self.motors]
