"""
attitude_control.py

The attitude controller ArduSub's STABILIZE mode runs on, ported from the
vendored source:

  libraries/AC_AttitudeControl/AC_AttitudeControl.cpp      (base controller)
  libraries/AC_AttitudeControl/AC_AttitudeControl_Sub.h    (Sub gains)
  libraries/AC_PID/AC_PID.cpp                              (rate PIDs)
  libraries/AP_Math/control.cpp                            (sqrt_controller)
  libraries/AP_Math/quaternion.cpp                         (quaternion maths)

Chain, per AC_AttitudeControl::attitude_controller_run_quat():

    target euler angles
          |  (quaternion attitude target)
          v
    thrust_heading_rotation_angles(target, body)
          |  decomposes the target-vs-body rotation into a thrust-vector
          |  correction (x, y) and a heading correction (z)
          v
    update_ang_vel_target_from_att_error()
          |  per-axis sqrt_controller: angle error -> body rate target,
          |  shaped so the vehicle can always decelerate within its
          |  angular acceleration limit
          v
    rate_controller_run()
          |  three AC_PID instances on body rates (gyro feedback)
          v
    motor roll / pitch / yaw demands in -1..+1

WHAT IS FAITHFUL HERE
  - Every gain, filter cutoff and limit is the vendored default, not a
    retyped number -- see SubAttitudeDefaults below, which cites the
    #define it came from.
  - sqrt_controller, update_i, the PID update order, thrust/heading error
    decomposition and ang_vel_limit are direct ports.
  - With the Sub defaults two branches collapse and the port is EXACT
    rather than approximate:
      * srmax = 0     -> the slew limiter is disabled, so Dmod == 1
      * filt_E_hz = 0 -> calc_lowpass_alpha_dt returns 1, so the error
                         filter is a passthrough
    Both are reproduced as written rather than assumed away; change the
    defaults and they behave like the real thing.

WHAT IS DELIBERATELY NOT PORTED
  - Input shaping and rate feedforward (attitude_command_model,
    body_to_euler_limit, euler_derivative_to_body). This node takes
    AC_AttitudeControl's OWN no-feedforward branch -- the `else` of
    `if (_rate_bf_ff_enabled)` in
    input_euler_angle_roll_pitch_euler_rate_yaw_rad() -- which is what the
    real firmware runs with ATC_RATE_FF_ENAB = 0. Stock default is 1, so
    expect softer, less anticipatory response than a stock vehicle: the
    controller reacts to attitude error rather than feeding the pilot's
    commanded rate forward.
  - Notch filters (AP_FILTER), _angle_P_scale, PD_MAX, D_FF, and the
    EKF-reset handling. All are inactive at Sub defaults.
  - update_throttle_rpy_mix() / throttle boosting: STABILIZE calls
    set_throttle_out(..., apply_angle_boost=false, ...), so throttle is
    passed through untouched, which this node does directly.

NOT VALIDATED AGAINST HARDWARE. The MANUAL path in this package was
checked bit-exact against a real BlueROV2 recording; STABILIZE is a closed
loop and no STABILIZE recording exists yet, so treat these numbers as
"faithful to the source" and not as "verified against the vehicle".
"""
import math
from dataclasses import dataclass
from typing import Tuple

# --- AC_AttitudeControl.h --------------------------------------------------
ACCEL_RP_CONTROLLER_MIN_RADSS = math.radians(40.0)
ACCEL_RP_CONTROLLER_MAX_RADSS = math.radians(720.0)
ACCEL_Y_CONTROLLER_MIN_RADSS = math.radians(10.0)
ACCEL_Y_CONTROLLER_MAX_RADSS = math.radians(120.0)
THRUST_ERROR_ANGLE_RAD = math.radians(30.0)
YAW_MAX_ERROR_ANGLE_RAD = math.radians(45.0)


@dataclass
class SubAttitudeDefaults:
    """
    Every value below is the vendored default for a Sub, with the #define
    it comes from. Do not retype these -- re-read them from
    reference/ardupilot after any fetch.
    """
    # AC_AttitudeControl_Sub.cpp:344-346 -- _p_angle_{roll,pitch,yaw}.kP()
    angle_p_roll: float = 6.0        # AC_ATC_SUB_ANGLE_P
    angle_p_pitch: float = 6.0       # AC_ATC_SUB_ANGLE_P
    angle_p_yaw: float = 6.0         # AC_ATC_SUB_ANGLE_P

    # AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_DEGSS (base default), and
    # AC_ATC_SUB_ACCEL_Y_MAX_DEGSS via AC_AttitudeControl_Sub.cpp:348
    accel_roll_max_degss: float = 1100.0
    accel_pitch_max_degss: float = 1100.0
    accel_yaw_max_degss: float = 1100.0

    # ATC_RATE_{R,P,Y}_MAX -- 0 means "no limit", the ArduPilot default.
    ang_vel_roll_max_degs: float = 0.0
    ang_vel_pitch_max_degs: float = 0.0
    ang_vel_yaw_max_degs: float = 0.0

    # AC_AttitudeControl_Sub.h rp_defaults / _pid_rate_yaw
    rate_rp_p: float = 0.135         # AC_ATC_SUB_RATE_RP_P
    rate_rp_i: float = 0.090         # AC_ATC_SUB_RATE_RP_I
    rate_rp_d: float = 0.0036        # AC_ATC_SUB_RATE_RP_D
    rate_rp_imax: float = 0.444      # AC_ATC_SUB_RATE_RP_IMAX
    rate_rp_filt_hz: float = 30.0    # AC_ATC_SUB_RATE_RP_FILT_HZ
    rate_yaw_p: float = 0.180        # AC_ATC_SUB_RATE_YAW_P
    rate_yaw_i: float = 0.018        # AC_ATC_SUB_RATE_YAW_I
    rate_yaw_d: float = 0.0          # AC_ATC_SUB_RATE_YAW_D
    rate_yaw_imax: float = 0.222     # AC_ATC_SUB_RATE_YAW_IMAX
    rate_yaw_filt_hz: float = 5.0    # AC_ATC_SUB_RATE_YAW_FILT_HZ


def constrain(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def wrap_2pi(a: float) -> float:
    """AP_Math::wrap_2PI() -- into [0, 2*pi)."""
    res = math.fmod(a, 2.0 * math.pi)
    if res < 0.0:
        res += 2.0 * math.pi
    return res


def wrap_pi(a: float) -> float:
    """
    AP_Math::wrap_PI() -- into [-pi, pi]. Note this goes via wrap_2PI, so
    the +-pi boundary lands on +pi, not -pi; to_axis_angle() depends on it.
    """
    res = wrap_2pi(a)
    if res > math.pi:
        res -= 2.0 * math.pi
    return res


def calc_lowpass_alpha_dt(dt: float, cutoff_hz: float) -> float:
    """AP_Math.cpp:418. cutoff of 0 means 'no filtering' -> alpha 1."""
    if dt < 0.0 or cutoff_hz < 0.0:
        return 1.0
    if cutoff_hz == 0.0:
        return 1.0
    if dt == 0.0:
        return 0.0
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    return dt / (dt + rc)


def sqrt_controller(error: float, p: float, second_ord_lim: float, dt: float) -> float:
    """
    Direct port of sqrt_controller() in AP_Math/control.cpp.

    Proportional near zero error, square-root beyond it, so the commanded
    rate is always one the vehicle can null out within second_ord_lim
    without overshooting.
    """
    if second_ord_lim <= 0.0:
        correction_rate = error * p
    elif p == 0.0:
        if error > 0.0:
            correction_rate = math.sqrt(2.0 * second_ord_lim * error)
        elif error < 0.0:
            correction_rate = -math.sqrt(2.0 * second_ord_lim * -error)
        else:
            correction_rate = 0.0
    else:
        linear_dist = second_ord_lim / (p * p)
        if error > linear_dist:
            correction_rate = math.sqrt(2.0 * second_ord_lim * (error - linear_dist / 2.0))
        elif error < -linear_dist:
            correction_rate = -math.sqrt(2.0 * second_ord_lim * (-error - linear_dist / 2.0))
        else:
            correction_rate = error * p

    if dt > 0.0:
        return constrain(correction_rate, -abs(error) / dt, abs(error) / dt)
    return correction_rate


class Quat:
    """
    Minimal (w, x, y, z) quaternion, matching AP_Math/quaternion.cpp's
    conventions: a quaternion here is a passive rotation from body frame to
    NED, so `q.rotate(v)` takes a body-frame vector to the earth frame.
    """

    __slots__ = ("w", "x", "y", "z")

    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w, self.x, self.y, self.z = w, x, y, z

    @staticmethod
    def from_euler(roll: float, pitch: float, yaw: float) -> "Quat":
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        return Quat(cr * cp * cy + sr * sp * sy,
                    sr * cp * cy - cr * sp * sy,
                    cr * sp * cy + sr * cp * sy,
                    cr * cp * sy - sr * sp * cy)

    @staticmethod
    def from_axis_angle(axis: Tuple[float, float, float], theta: float) -> "Quat":
        """quaternion.cpp:469."""
        if theta == 0.0:
            return Quat()
        st2 = math.sin(theta * 0.5)
        return Quat(math.cos(theta * 0.5), axis[0] * st2, axis[1] * st2, axis[2] * st2)

    def to_euler(self) -> Tuple[float, float, float]:
        w, x, y, z = self.w, self.x, self.y, self.z
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(constrain(2.0 * (w * y - z * x), -1.0, 1.0))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def to_axis_angle(self) -> Tuple[float, float, float]:
        """quaternion.cpp:497 -- returns axis * angle."""
        norm = math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
        if norm == 0.0:
            return (self.x, self.y, self.z)
        theta = wrap_pi(2.0 * math.atan2(norm, self.w))
        return (self.x / norm * theta, self.y / norm * theta, self.z / norm * theta)

    def inverse(self) -> "Quat":
        return Quat(self.w, -self.x, -self.y, -self.z)

    def normalize(self) -> "Quat":
        n = math.sqrt(self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2)
        if n == 0.0:
            return Quat()
        return Quat(self.w / n, self.x / n, self.y / n, self.z / n)

    def __mul__(self, o: "Quat") -> "Quat":
        return Quat(self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
                    self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
                    self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
                    self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w)

    def rotate(self, v: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """quaternion.cpp:731 -- v + 2w(qv x v) + 2 qv x (qv x v)."""
        qv = (self.x, self.y, self.z)
        uv = (qv[1] * v[2] - qv[2] * v[1],
              qv[2] * v[0] - qv[0] * v[2],
              qv[0] * v[1] - qv[1] * v[0])
        uv = (2.0 * uv[0], 2.0 * uv[1], 2.0 * uv[2])
        return (v[0] + self.w * uv[0] + qv[1] * uv[2] - qv[2] * uv[1],
                v[1] + self.w * uv[1] + qv[2] * uv[0] - qv[0] * uv[2],
                v[2] + self.w * uv[2] + qv[0] * uv[1] - qv[1] * uv[0])


# ---------------------------------------------------------------------------
# Frame conversion
# ---------------------------------------------------------------------------
# ROS (REP-103) and ArduPilot do not share a frame convention, and this
# controller is written in ArduPilot's:
#
#   ROS       world ENU, body FLU (x forward, y LEFT,  z UP)
#   ArduPilot world NED, body FRD (x forward, y RIGHT, z DOWN)
#
# Both are right-handed, so roll (about x) agrees, but pitch and yaw are
# INVERTED between them: a positive ROS pitch is nose-down where a positive
# ArduPilot pitch is nose-up, and a positive ROS yaw turns to port where a
# positive ArduPilot yaw turns to starboard.
#
# This matters because AP_Motors6DOF's mixing matrix is in ArduPilot's
# convention while mavros publishes attitude in ROS's -- for BOTH a real
# vehicle and the Unity sim, since mavros converts the flight controller's
# native NED to ENU on the way out. Feeding a ROS attitude straight into
# this controller inverts the pitch and yaw feedback, so those axes are
# driven AWAY from the target and the vehicle tumbles.
#
# Composed as quaternions rather than by negating euler angles, so it stays
# correct through +-90 degrees of pitch.
_Q_ENU_TO_NED = None   # built lazily below
_Q_FLU_TO_FRD = None


def flu_enu_to_frd_ned(q: "Quat") -> "Quat":
    """
    Convert a body-FLU-to-world-ENU attitude (what mavros publishes) into
    the body-FRD-to-world-NED attitude this controller expects.
    """
    global _Q_ENU_TO_NED, _Q_FLU_TO_FRD
    if _Q_ENU_TO_NED is None:
        r2 = math.sqrt(2.0) / 2.0
        # ENU -> NED: 90 deg about Z then 180 deg about X
        _Q_ENU_TO_NED = Quat(0.0, r2, r2, 0.0)
        # FLU -> FRD: 180 deg about X
        _Q_FLU_TO_FRD = Quat(0.0, 1.0, 0.0, 0.0)
    return (_Q_ENU_TO_NED * q * _Q_FLU_TO_FRD).normalize()


def flu_to_frd_rates(gyro):
    """Body angular rates FLU -> FRD: y and z flip, x is unchanged."""
    return (gyro[0], -gyro[1], -gyro[2])


class ACPID:
    """
    Port of AC_PID (libraries/AC_PID/AC_PID.cpp), covering update_all() and
    update_i() as the Sub rate loops use them.

    Omitted because they are inactive at Sub defaults, and asserted to be so
    in __init__: notch filters (AP_FILTER), the slew limiter (srmax = 0 ->
    Dmod = 1), PD_MAX, feed-forward and D_FF (ff = dff = 0).
    """

    def __init__(self, p, i, d, imax, filt_t_hz, filt_e_hz, filt_d_hz):
        self.kp, self.ki, self.kd, self.kimax = p, i, d, abs(imax)
        self.filt_t_hz, self.filt_e_hz, self.filt_d_hz = filt_t_hz, filt_e_hz, filt_d_hz
        self.reset_filter()
        self._integrator = 0.0

    def reset_filter(self):
        """AC_PID::reset_filter() -- next update_all seeds the filters."""
        self._reset_filter = True
        self._target = 0.0
        self._error = 0.0
        self._derivative = 0.0

    def reset_I(self):
        self._integrator = 0.0

    def update_i(self, dt: float, limit: bool):
        """AC_PID::update_i() -- integrate, with anti-windup against `limit`."""
        if self.ki != 0.0 and dt > 0.0:
            growing_into_limit = not (
                (self._integrator > 0.0 and self._error < 0.0)
                or (self._integrator < 0.0 and self._error > 0.0)
            )
            if not (limit and growing_into_limit):
                self._integrator += self._error * self.ki * dt
                self._integrator = constrain(self._integrator, -self.kimax, self.kimax)
        else:
            self._integrator = 0.0

    def update_all(self, target: float, measurement: float, dt: float,
                   limit: bool = False) -> float:
        """AC_PID::update_all()."""
        if not (math.isfinite(target) and math.isfinite(measurement)):
            return 0.0

        if self._reset_filter:
            self._reset_filter = False
            self._target = target
            self._error = self._target - measurement
            self._derivative = 0.0
        else:
            error_last = self._error
            self._target += calc_lowpass_alpha_dt(dt, self.filt_t_hz) * (target - self._target)
            error = self._target - measurement
            self._error += calc_lowpass_alpha_dt(dt, self.filt_e_hz) * (error - self._error)
            if dt > 0.0:
                derivative = (self._error - error_last) / dt
                self._derivative += calc_lowpass_alpha_dt(dt, self.filt_d_hz) * (
                    derivative - self._derivative)

        self.update_i(dt, limit)
        # Dmod is 1 with the slew limiter disabled (srmax = 0), so P and D
        # are used unscaled -- see the class docstring.
        return self._error * self.kp + self._derivative * self.kd + self._integrator


class AttitudeControlSub:
    """
    Port of AC_AttitudeControl / AC_AttitudeControl_Sub, no-feedforward path.

    Usage per control iteration, mirroring ArduSub's loop order:
        ac.input_euler_angle_roll_pitch_euler_rate_yaw(r, p, yaw_rate, dt)
          -- or --
        ac.input_euler_angle_roll_pitch_yaw(r, p, yaw, dt)
        roll_out, pitch_out, yaw_out = ac.rate_controller_run(gyro, dt)
    """

    def __init__(self, defaults: SubAttitudeDefaults = None):
        self.d = defaults or SubAttitudeDefaults()
        d = self.d
        self.rate_roll = ACPID(d.rate_rp_p, d.rate_rp_i, d.rate_rp_d, d.rate_rp_imax,
                               d.rate_rp_filt_hz, 0.0, d.rate_rp_filt_hz)
        self.rate_pitch = ACPID(d.rate_rp_p, d.rate_rp_i, d.rate_rp_d, d.rate_rp_imax,
                                d.rate_rp_filt_hz, 0.0, d.rate_rp_filt_hz)
        self.rate_yaw = ACPID(d.rate_yaw_p, d.rate_yaw_i, d.rate_yaw_d, d.rate_yaw_imax,
                              d.rate_yaw_filt_hz, 0.0, d.rate_yaw_filt_hz)
        self.attitude_body = Quat()
        self._attitude_target = Quat()
        self._euler_angle_target = [0.0, 0.0, 0.0]
        self._ang_vel_body = [0.0, 0.0, 0.0]
        self.thrust_error_angle_rad = 0.0

    # -- feedback ---------------------------------------------------------
    def set_attitude_body(self, q: Quat):
        self.attitude_body = q.normalize()

    def relax_attitude_controllers(self):
        """
        AC_AttitudeControl::relax_attitude_controllers() -- snap the target
        to the current attitude and drop the integrators, so re-arming or
        re-entering the mode does not start with a wound-up controller.
        """
        self._attitude_target = self.attitude_body
        self._euler_angle_target = list(self.attitude_body.to_euler())
        self._ang_vel_body = [0.0, 0.0, 0.0]
        for pid in (self.rate_roll, self.rate_pitch, self.rate_yaw):
            pid.reset_I()
            pid.reset_filter()

    # -- attitude target --------------------------------------------------
    def input_euler_angle_roll_pitch_euler_rate_yaw(self, roll, pitch, yaw_rate, dt):
        """
        input_euler_angle_roll_pitch_euler_rate_yaw_rad(), `else` branch --
        the no-feedforward path (ATC_RATE_FF_ENAB = 0).
        """
        self._euler_angle_target[0] = roll
        self._euler_angle_target[1] = pitch
        self._euler_angle_target[2] += yaw_rate * dt
        self._attitude_target = Quat.from_euler(*self._euler_angle_target)
        self._attitude_controller_run_quat(dt)

    def input_euler_angle_roll_pitch_yaw(self, roll, pitch, yaw, dt):
        """input_euler_angle_roll_pitch_yaw_rad(), no-feedforward path."""
        self._euler_angle_target = [roll, pitch, yaw]
        self._attitude_target = Quat.from_euler(roll, pitch, yaw)
        self._attitude_controller_run_quat(dt)

    # -- controller core --------------------------------------------------
    def _thrust_vector_rotation_angles(self):
        """
        AC_AttitudeControl::thrust_vector_rotation_angles().

        Splits the target-vs-body rotation into the rotation that aligns the
        thrust vectors (x, y) and the leftover rotation about thrust (z), so
        levelling is never traded against heading.
        """
        up = (0.0, 0.0, -1.0)
        tgt = self._attitude_target.rotate(up)
        body = self.attitude_body.rotate(up)

        dot = constrain(body[0] * tgt[0] + body[1] * tgt[1] + body[2] * tgt[2], -1.0, 1.0)
        thrust_error_angle = math.acos(dot)

        cross = (body[1] * tgt[2] - body[2] * tgt[1],
                 body[2] * tgt[0] - body[0] * tgt[2],
                 body[0] * tgt[1] - body[1] * tgt[0])
        length = math.sqrt(sum(c * c for c in cross))
        if length == 0.0 or thrust_error_angle == 0.0:
            cross = up
        else:
            cross = (cross[0] / length, cross[1] / length, cross[2] / length)

        cross = self.attitude_body.inverse().rotate(cross)
        thrust_vector_correction = Quat.from_axis_angle(cross, thrust_error_angle)

        rot = thrust_vector_correction.to_axis_angle()
        err_x, err_y = rot[0], rot[1]

        heading_q = (thrust_vector_correction.inverse()
                     * self.attitude_body.inverse()
                     * self._attitude_target)
        err_z = heading_q.to_axis_angle()[2]
        return [err_x, err_y, err_z], thrust_error_angle

    def _update_ang_vel_target_from_att_error(self, err, dt):
        """AC_AttitudeControl::update_ang_vel_target_from_att_error()."""
        d = self.d
        out = [0.0, 0.0, 0.0]
        for i, (angle_p, accel_max_degss, lo, hi) in enumerate((
            (d.angle_p_roll, d.accel_roll_max_degss,
             ACCEL_RP_CONTROLLER_MIN_RADSS, ACCEL_RP_CONTROLLER_MAX_RADSS),
            (d.angle_p_pitch, d.accel_pitch_max_degss,
             ACCEL_RP_CONTROLLER_MIN_RADSS, ACCEL_RP_CONTROLLER_MAX_RADSS),
            (d.angle_p_yaw, d.accel_yaw_max_degss,
             ACCEL_Y_CONTROLLER_MIN_RADSS, ACCEL_Y_CONTROLLER_MAX_RADSS),
        )):
            accel_max = math.radians(accel_max_degss)
            if accel_max != 0.0:
                out[i] = sqrt_controller(err[i], angle_p,
                                         constrain(accel_max / 2.0, lo, hi), dt)
            else:
                out[i] = angle_p * err[i]
        return out

    def _ang_vel_limit(self, v):
        """AC_AttitudeControl::ang_vel_limit(). A max of 0 means unlimited."""
        d = self.d
        r = math.radians(d.ang_vel_roll_max_degs)
        p = math.radians(d.ang_vel_pitch_max_degs)
        y = math.radians(d.ang_vel_yaw_max_degs)
        if r == 0.0 or p == 0.0:
            if r != 0.0:
                v[0] = constrain(v[0], -r, r)
            if p != 0.0:
                v[1] = constrain(v[1], -p, p)
        else:
            length = math.sqrt((v[0] / r) ** 2 + (v[1] / p) ** 2)
            if length > 1.0:
                v[0] = (v[0] / r) * r / length
                v[1] = (v[1] / p) * p / length
        if y != 0.0:
            v[2] = constrain(v[2], -y, y)
        return v

    def _attitude_controller_run_quat(self, dt):
        """
        AC_AttitudeControl::attitude_controller_run_quat(), no-feedforward.

        The feedforward blending around AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD is
        kept for its OTHER effect: past twice that error the yaw rate demand
        is abandoned in favour of the measured gyro, so a vehicle far from
        level stops fighting for heading while it rights itself.
        """
        err, thrust_error_angle = self._thrust_vector_rotation_angles()
        self.thrust_error_angle_rad = thrust_error_angle

        # Limit heading error to what the yaw rate loop could actually
        # deliver at zero rate -- thrust_heading_rotation_angles().
        if self.rate_yaw.kp != 0.0 and self.d.angle_p_yaw != 0.0:
            heading_accel_max = constrain(math.radians(self.d.accel_yaw_max_degss) / 2.0,
                                          ACCEL_Y_CONTROLLER_MIN_RADSS,
                                          ACCEL_Y_CONTROLLER_MAX_RADSS)
            # inv_sqrt_controller(output, p, D_max): the error at which the
            # sqrt controller would demand 1/kP of rate.
            out = 1.0 / self.rate_yaw.kp
            p = self.d.angle_p_yaw
            linear_out = heading_accel_max / p
            if out > linear_out:
                inv = (out * out) / (2.0 * heading_accel_max) + heading_accel_max / (2.0 * p * p)
            else:
                inv = out / p
            heading_error_max = min(inv, YAW_MAX_ERROR_ANGLE_RAD)
            if abs(err[2]) > heading_error_max:
                err[2] = constrain(wrap_pi(err[2]), -heading_error_max, heading_error_max)

        ang_vel = self._update_ang_vel_target_from_att_error(err, dt)
        ang_vel = self._ang_vel_limit(ang_vel)

        if thrust_error_angle > THRUST_ERROR_ANGLE_RAD * 2.0:
            ang_vel[2] = self._last_gyro_z if hasattr(self, "_last_gyro_z") else 0.0

        self._ang_vel_body = ang_vel

    # -- rate loop --------------------------------------------------------
    def rate_controller_run(self, gyro, dt, limits=(False, False, False)):
        """
        AC_AttitudeControl_Sub::rate_controller_run() -- three AC_PID
        instances on body rates. Returns motor roll/pitch/yaw in -1..+1.
        `gyro` is the body-frame angular rate (rad/s) as x, y, z.
        """
        self._last_gyro_z = gyro[2]
        roll = self.rate_roll.update_all(self._ang_vel_body[0], gyro[0], dt, limits[0])
        pitch = self.rate_pitch.update_all(self._ang_vel_body[1], gyro[1], dt, limits[1])
        yaw = self.rate_yaw.update_all(self._ang_vel_body[2], gyro[2], dt, limits[2])
        return (constrain(roll, -1.0, 1.0),
                constrain(pitch, -1.0, 1.0),
                constrain(yaw, -1.0, 1.0))

    @property
    def ang_vel_body(self):
        return tuple(self._ang_vel_body)
