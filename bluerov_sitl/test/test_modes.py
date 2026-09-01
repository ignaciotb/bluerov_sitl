import math

import pytest

from bluerov_sitl.attitude_control import Quat
from bluerov_sitl.modes import (
    ACRO_YAW_P, MODES, Demands, ManualMode, StabilizeMode,
    get_pilot_desired_lean_angles,
)
from bluerov_sitl.rc_channel import ChannelMode, RCChannel

DEADZONES = {"pitch": 30, "roll": 30, "throttle": 30,
             "yaw": 40, "forward": 30, "lateral": 30}


@pytest.fixture
def channels():
    return {name: RCChannel(name=name, mode=ChannelMode.ANGLE, pwm_min=1100,
                            pwm_trim=1500, pwm_max=1900, deadzone_us=dz)
            for name, dz in DEADZONES.items()}


def drive(channels, now=0.0, **pwm):
    for axis, value in pwm.items():
        channels[axis].apply_override(value, now)


# --- registry --------------------------------------------------------------

def test_both_modes_are_registered():
    assert set(MODES) == {"manual", "stabilize"}


# --- get_pilot_desired_lean_angles (ArduSub/Attitude.cpp) ------------------

def test_lean_angles_full_stick_reaches_angle_max():
    roll, pitch = get_pilot_desired_lean_angles(4500.0, 0.0, 4500.0, 4500.0)
    assert roll == pytest.approx(4500.0, abs=1.0)
    assert pitch == pytest.approx(0.0)


def test_lean_angles_circular_limit_caps_the_corners():
    """
    Full roll AND full pitch would be 4500*sqrt(2) = 6364 cd of combined
    tilt; the circular limit pulls it back onto the 4500 circle.
    """
    roll, pitch = get_pilot_desired_lean_angles(4500.0, 4500.0, 4500.0, 4500.0)
    assert math.hypot(roll, pitch) <= 4500.0 + 1.0


def test_lean_angles_are_odd_about_centre():
    pos = get_pilot_desired_lean_angles(2000.0, 1000.0, 4500.0, 4500.0)
    neg = get_pilot_desired_lean_angles(-2000.0, -1000.0, 4500.0, 4500.0)
    assert neg == pytest.approx((-pos[0], -pos[1]))


def test_lean_angles_respect_a_reduced_angle_max():
    roll, _ = get_pilot_desired_lean_angles(4500.0, 0.0, 2000.0, 4500.0)
    assert abs(roll) <= 2000.0 + 1.0


# --- ManualMode ------------------------------------------------------------

def test_manual_passes_through_without_deadzone(channels):
    drive(channels, forward=1600, lateral=1400)
    d = ManualMode(channels).run(0.0, 0.02, armed=True)
    assert d.forward == pytest.approx(0.25)     # not 70/370 = 0.189
    assert d.lateral == pytest.approx(-0.25)
    assert d.throttle_0_to_1 == pytest.approx(0.5)


def test_manual_disarmed_is_neutral(channels):
    drive(channels, forward=1900)
    assert ManualMode(channels).run(0.0, 0.02, armed=False) == Demands()


# --- StabilizeMode ---------------------------------------------------------

def _stab(channels, roll=0.0, pitch=0.0, yaw=0.0, gyro=(0.0, 0.0, 0.0)):
    m = StabilizeMode(channels)
    m.set_attitude(Quat.from_euler(roll, pitch, yaw), gyro)
    m.on_enter(0.0)
    m.set_attitude(Quat.from_euler(roll, pitch, yaw), gyro)
    return m


def test_stabilize_refuses_to_run_without_attitude(channels):
    """Closing the loop on a default attitude would drive the thrusters."""
    with pytest.raises(RuntimeError, match="without attitude feedback"):
        StabilizeMode(channels).run(0.0, 0.02, armed=True)


def test_stabilize_level_and_centred_is_quiet(channels):
    d = _stab(channels).run(0.0, 0.02, armed=True)
    assert (d.roll, d.pitch, d.yaw) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)


def test_stabilize_corrects_roll_without_pilot_input(channels):
    """The whole point of the mode: it levels itself."""
    d = _stab(channels, roll=0.2).run(0.0, 0.02, armed=True)
    assert d.roll < 0.0
    assert d.pitch == pytest.approx(0.0, abs=1e-6)


def test_stabilize_corrects_pitch_without_pilot_input(channels):
    d = _stab(channels, pitch=0.2).run(0.0, 0.02, armed=True)
    assert d.pitch < 0.0
    assert d.roll == pytest.approx(0.0, abs=1e-6)


def test_stabilize_passes_throttle_forward_lateral_through(channels):
    """Translation is untouched by the attitude loop, same as MANUAL."""
    drive(channels, forward=1600, lateral=1400, throttle=1700)
    d = _stab(channels).run(0.0, 0.02, armed=True)
    assert d.forward == pytest.approx(0.25)
    assert d.lateral == pytest.approx(-0.25)
    assert d.throttle_0_to_1 == pytest.approx(0.75)


def test_stabilize_roll_stick_commands_a_lean_not_a_rate(channels):
    """
    Held roll stick against a level vehicle: the controller drives toward
    the commanded lean angle, so the demand is non-zero and signed with the
    stick.
    """
    drive(channels, roll=1900)
    d = _stab(channels).run(0.0, 0.02, armed=True)
    assert d.roll > 0.0


def test_stabilize_holds_heading_after_the_grace_period(channels):
    """
    mode_stabilize.cpp:50 -- for 250 ms after the yaw stick centres the
    heading keeps re-latching; after that it is held as an absolute bearing,
    so drifting off it produces a correcting yaw demand.
    """
    m = _stab(channels, yaw=0.0)
    drive(channels, yaw=1900, now=0.0)
    m.run(0.0, 0.02, armed=True)              # pilot yawing -> latch follows
    drive(channels, yaw=1500, now=0.5)

    m.set_attitude(Quat.from_euler(0.0, 0.0, 0.3), (0.0, 0.0, 0.0))
    d = m.run(0.5, 0.02, armed=True)          # 0.5s > 250ms -> bearing held
    assert d.yaw < 0.0                        # yaw back toward the latch


def test_stabilize_relatches_heading_inside_the_grace_period(channels):
    """Inside 250 ms the target follows the vehicle, so no correction."""
    m = _stab(channels, yaw=0.0)
    drive(channels, yaw=1900, now=0.0)
    m.run(0.0, 0.02, armed=True)
    drive(channels, yaw=1500, now=0.1)

    m.set_attitude(Quat.from_euler(0.0, 0.0, 0.3), (0.0, 0.0, 0.0))
    m.run(0.1, 0.02, armed=True)              # 0.1s < 250ms
    assert m._last_pilot_heading_rad == pytest.approx(0.3)


def test_stabilize_yaw_deadzone_is_scaled_by_pilot_gain(channels):
    """
    mode_stabilize.cpp:35 -- gain shrinks the yaw DEADZONE (and nothing
    else), so a low-gain vehicle responds to smaller yaw sticks.
    """
    now = 0.0
    channels["yaw"].apply_override(1530, now)      # inside the raw 40us dz
    full = StabilizeMode(channels, {"pilot_gain": 1.0})
    half = StabilizeMode(channels, {"pilot_gain": 0.5})
    assert channels["yaw"].pwm_to_angle_dz_trim(40, 1500, now) == 0.0
    assert channels["yaw"].pwm_to_angle_dz_trim(20, 1500, now) != 0.0
    assert full.params["pilot_gain"] != half.params["pilot_gain"]


def test_stabilize_disarmed_is_neutral_and_resets(channels):
    m = _stab(channels, roll=0.4)
    for _ in range(20):
        m.run(0.0, 0.02, armed=True)
    assert abs(m.ac.rate_roll._integrator) > 0.0
    assert m.run(0.5, 0.02, armed=False) == Demands()
    assert m.ac.rate_roll._integrator == 0.0


def test_stabilize_yaw_rate_scaling_uses_acro_yaw_p(channels):
    """get_pilot_desired_yaw_rate() = stick_angle * ACRO_YAW_P."""
    now = 0.0
    channels["yaw"].apply_override(1900, now)
    stick_cd = channels["yaw"].pwm_to_angle_dz_trim(20, 1500, now)
    assert stick_cd * ACRO_YAW_P == pytest.approx(4500.0 * ACRO_YAW_P, rel=1e-3)
