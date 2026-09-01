import math

import pytest

from bluerov_sitl.attitude_control import (
    ACPID, AttitudeControlSub, Quat, SubAttitudeDefaults,
    calc_lowpass_alpha_dt, sqrt_controller, wrap_pi,
)


# --- sqrt_controller (AP_Math/control.cpp) ---------------------------------

def test_sqrt_controller_is_linear_near_zero():
    """Inside linear_dist = lim/p^2 the response is plain proportional."""
    p, lim = 6.0, 10.0
    linear_dist = lim / (p * p)          # 0.2778
    err = linear_dist * 0.5
    assert sqrt_controller(err, p, lim, 0.0) == pytest.approx(err * p)


def test_sqrt_controller_is_sqrt_shaped_far_out():
    """Beyond the linear region it grows as sqrt, i.e. slower than p*error."""
    p, lim = 6.0, 10.0
    err = 5.0
    out = sqrt_controller(err, p, lim, 0.0)
    assert out < err * p
    assert out == pytest.approx(math.sqrt(2.0 * lim * (err - (lim / (p * p)) / 2.0)))


def test_sqrt_controller_never_overshoots_in_one_step():
    """The dt clamp stops it commanding more than error/dt."""
    out = sqrt_controller(0.01, 100.0, 1000.0, 0.02)
    assert abs(out) <= 0.01 / 0.02


def test_sqrt_controller_no_accel_limit_is_pure_p():
    assert sqrt_controller(0.5, 6.0, 0.0, 0.0) == pytest.approx(3.0)


def test_sqrt_controller_is_odd():
    for e in (0.05, 0.5, 5.0):
        assert sqrt_controller(-e, 6.0, 10.0, 0.0) == pytest.approx(
            -sqrt_controller(e, 6.0, 10.0, 0.0))


# --- AC_PID ----------------------------------------------------------------

def test_lowpass_alpha_zero_cutoff_is_passthrough():
    """filt_E_hz = 0 on the Sub PIDs -- alpha 1 means no error filtering."""
    assert calc_lowpass_alpha_dt(0.02, 0.0) == 1.0


def test_pid_first_call_seeds_filters_without_a_derivative_spike():
    pid = ACPID(0.135, 0.0, 0.0036, 0.444, 30.0, 0.0, 30.0)
    out = pid.update_all(target=1.0, measurement=0.0, dt=0.02)
    assert pid._derivative == 0.0          # no spike on the first sample
    assert out == pytest.approx(0.135)     # pure P on a unit error


def test_pid_integrator_is_clamped_to_imax():
    pid = ACPID(0.0, 10.0, 0.0, 0.444, 0.0, 0.0, 0.0)
    pid.update_all(1.0, 0.0, 0.02)
    for _ in range(500):
        pid.update_all(1.0, 0.0, 0.02)
    assert pid._integrator == pytest.approx(0.444)


def test_pid_anti_windup_blocks_growth_while_limited():
    pid = ACPID(0.0, 10.0, 0.0, 0.444, 0.0, 0.0, 0.0)
    pid.update_all(1.0, 0.0, 0.02)
    for _ in range(10):
        pid.update_all(1.0, 0.0, 0.02)
    wound = pid._integrator
    assert wound > 0.0
    for _ in range(10):
        pid.update_all(1.0, 0.0, 0.02, limit=True)
    assert pid._integrator == pytest.approx(wound)   # frozen, not grown


def test_pid_anti_windup_still_allows_unwinding():
    """A limited axis may still shrink its integrator when error reverses."""
    pid = ACPID(0.0, 10.0, 0.0, 0.444, 0.0, 0.0, 0.0)
    pid.update_all(1.0, 0.0, 0.02)
    for _ in range(10):
        pid.update_all(1.0, 0.0, 0.02)
    wound = pid._integrator
    for _ in range(5):
        pid.update_all(-1.0, 0.0, 0.02, limit=True)
    assert pid._integrator < wound


# --- quaternions -----------------------------------------------------------

@pytest.mark.parametrize("rpy", [(0.0, 0.0, 0.0), (0.3, -0.2, 1.1), (-0.9, 0.4, -2.5)])
def test_quat_euler_roundtrip(rpy):
    r, p, y = Quat.from_euler(*rpy).to_euler()
    assert (r, p, y) == pytest.approx(rpy, abs=1e-9)


def test_quat_rotate_matches_known_yaw():
    """90 deg yaw takes body +x (nose) to earth +y."""
    q = Quat.from_euler(0.0, 0.0, math.pi / 2)
    assert q.rotate((1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)


def test_quat_axis_angle_roundtrip():
    axis, theta = (0.0, 0.0, 1.0), 0.7
    assert Quat.from_axis_angle(axis, theta).to_axis_angle() == pytest.approx(
        (0.0, 0.0, theta), abs=1e-9)


def test_wrap_pi():
    assert wrap_pi(3 * math.pi) == pytest.approx(math.pi, abs=1e-9)
    assert wrap_pi(-3 * math.pi) == pytest.approx(math.pi, abs=1e-9)


# --- the assembled controller ---------------------------------------------

def _ac_at(roll=0.0, pitch=0.0, yaw=0.0):
    ac = AttitudeControlSub()
    ac.set_attitude_body(Quat.from_euler(roll, pitch, yaw))
    ac.relax_attitude_controllers()
    ac.set_attitude_body(Quat.from_euler(roll, pitch, yaw))
    return ac


def test_level_vehicle_level_target_demands_nothing():
    ac = _ac_at()
    ac.input_euler_angle_roll_pitch_euler_rate_yaw(0.0, 0.0, 0.0, 0.02)
    assert ac.ang_vel_body == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    assert ac.rate_controller_run((0.0, 0.0, 0.0), 0.02) == pytest.approx(
        (0.0, 0.0, 0.0), abs=1e-9)


def test_roll_error_commands_opposing_roll_rate():
    """Rolled +0.2 rad with a level target -> negative roll rate demand."""
    ac = _ac_at(roll=0.2)
    ac.input_euler_angle_roll_pitch_euler_rate_yaw(0.0, 0.0, 0.0, 0.02)
    assert ac.ang_vel_body[0] < 0.0
    assert ac.ang_vel_body[1] == pytest.approx(0.0, abs=1e-6)
    assert ac.rate_controller_run((0.0, 0.0, 0.0), 0.02)[0] < 0.0


def test_pitch_error_commands_opposing_pitch_rate():
    ac = _ac_at(pitch=0.2)
    ac.input_euler_angle_roll_pitch_euler_rate_yaw(0.0, 0.0, 0.0, 0.02)
    assert ac.ang_vel_body[1] < 0.0
    assert ac.ang_vel_body[0] == pytest.approx(0.0, abs=1e-6)


def test_heading_error_commands_yaw_but_not_roll_pitch():
    """Level but yawed off target: correction is pure heading."""
    ac = _ac_at(yaw=0.3)
    ac.input_euler_angle_roll_pitch_yaw(0.0, 0.0, 0.0, 0.02)
    assert ac.ang_vel_body[2] < 0.0
    assert ac.ang_vel_body[0] == pytest.approx(0.0, abs=1e-6)
    assert ac.ang_vel_body[1] == pytest.approx(0.0, abs=1e-6)


def test_yaw_rate_target_integrates_the_heading():
    ac = _ac_at()
    ac.input_euler_angle_roll_pitch_euler_rate_yaw(0.0, 0.0, 0.5, 0.02)
    assert ac._euler_angle_target[2] == pytest.approx(0.01)


def test_relax_clears_a_wound_up_integrator():
    ac = _ac_at(roll=0.4)
    for _ in range(50):
        ac.input_euler_angle_roll_pitch_euler_rate_yaw(0.0, 0.0, 0.0, 0.02)
        ac.rate_controller_run((0.0, 0.0, 0.0), 0.02)
    assert abs(ac.rate_roll._integrator) > 0.0
    ac.relax_attitude_controllers()
    assert ac.rate_roll._integrator == 0.0


def test_relax_snaps_target_to_current_attitude():
    """Re-entering the mode must not inherit a stale target."""
    ac = _ac_at(roll=0.4, yaw=1.0)
    ac.relax_attitude_controllers()
    ac.input_euler_angle_roll_pitch_yaw(0.4, 0.0, 1.0, 0.02)
    assert ac.ang_vel_body == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)


def test_gains_are_the_vendored_sub_defaults():
    """Guards against someone 'tidying' these into round numbers."""
    d = SubAttitudeDefaults()
    assert (d.angle_p_roll, d.angle_p_pitch, d.angle_p_yaw) == (6.0, 6.0, 6.0)
    assert (d.rate_rp_p, d.rate_rp_i, d.rate_rp_d, d.rate_rp_imax) == (
        0.135, 0.090, 0.0036, 0.444)
    assert (d.rate_yaw_p, d.rate_yaw_i, d.rate_yaw_d, d.rate_yaw_imax) == (
        0.180, 0.018, 0.0, 0.222)
    assert (d.rate_rp_filt_hz, d.rate_yaw_filt_hz) == (30.0, 5.0)


# --- ROS <-> ArduPilot frame conversion ------------------------------------

def test_frame_conversion_preserves_level():
    from bluerov_sitl.attitude_control import flu_enu_to_frd_ned
    r, p, _ = flu_enu_to_frd_ned(Quat()).to_euler()
    assert (r, p) == pytest.approx((0.0, 0.0), abs=1e-9)


def test_frame_conversion_keeps_roll_and_inverts_pitch():
    """
    The bug that tumbled the sim: ROS and ArduPilot agree on roll but have
    opposite pitch (ROS positive pitch is nose-DOWN).
    """
    from bluerov_sitl.attitude_control import flu_enu_to_frd_ned
    r, p, _ = flu_enu_to_frd_ned(Quat.from_euler(0.3, 0.0, 0.0)).to_euler()
    assert r == pytest.approx(0.3, abs=1e-9)
    r, p, _ = flu_enu_to_frd_ned(Quat.from_euler(0.0, 0.3, 0.0)).to_euler()
    assert p == pytest.approx(-0.3, abs=1e-9)


def test_frame_conversion_inverts_yaw():
    """ROS positive yaw turns to port; ArduPilot positive yaw to starboard."""
    from bluerov_sitl.attitude_control import flu_enu_to_frd_ned
    y0 = flu_enu_to_frd_ned(Quat.from_euler(0.0, 0.0, 0.0)).to_euler()[2]
    y1 = flu_enu_to_frd_ned(Quat.from_euler(0.0, 0.0, 0.3)).to_euler()[2]
    assert y1 - y0 == pytest.approx(-0.3, abs=1e-9)


def test_frame_conversion_survives_steep_pitch():
    """Composed as quaternions, so it does not blow up near +-90 deg."""
    from bluerov_sitl.attitude_control import flu_enu_to_frd_ned
    q = flu_enu_to_frd_ned(Quat.from_euler(0.1, math.radians(89.0), 0.5))
    assert all(math.isfinite(v) for v in (q.w, q.x, q.y, q.z))
    assert q.to_euler()[1] == pytest.approx(math.radians(-89.0), abs=1e-6)


def test_gyro_frame_conversion():
    from bluerov_sitl.attitude_control import flu_to_frd_rates
    assert flu_to_frd_rates((1.0, 2.0, 3.0)) == (1.0, -2.0, -3.0)
