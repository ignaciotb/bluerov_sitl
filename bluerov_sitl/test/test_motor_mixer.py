from pathlib import Path

import pytest

from bluerov_sitl.motor_mixer_6dof import SixDofMixer, MixerConfig

FIXTURE = Path(__file__).parent / "fixtures" / "synthetic_test_matrix.yaml"


@pytest.fixture
def mixer():
    return SixDofMixer(str(FIXTURE), MixerConfig(pwm_min=1100, pwm_trim=1500, pwm_max=1900))


def test_disarmed_outputs_trim(mixer):
    out = mixer.mix(roll=1.0, pitch=1.0, yaw=1.0, throttle_0_to_1=1.0,
                    forward=1.0, lateral=1.0, armed=False)
    assert all(pwm == 1500 for pwm in out.values())


def test_zero_input_outputs_trim_when_armed(mixer):
    out = mixer.mix(roll=0.0, pitch=0.0, yaw=0.0, throttle_0_to_1=0.5,
                    forward=0.0, lateral=0.0, armed=True)
    # throttle_0_to_1=0.5 -> bidirectional 0.0 -> motor 4 (throttle) also neutral
    assert all(pwm == 1500 for pwm in out.values())


def test_full_positive_roll_only_drives_roll_motor(mixer):
    out = mixer.mix(roll=1.0, pitch=0.0, yaw=0.0, throttle_0_to_1=0.5,
                    forward=0.0, lateral=0.0, armed=True)
    assert out[1] == 1900          # motor 1 has roll_factor=1.0 -> full forward PWM
    assert all(pwm == 1500 for m, pwm in out.items() if m != 1)


def test_full_negative_pitch_only_drives_pitch_motor_reverse(mixer):
    out = mixer.mix(roll=0.0, pitch=-1.0, yaw=0.0, throttle_0_to_1=0.5,
                    forward=0.0, lateral=0.0, armed=True)
    assert out[2] == 1100          # motor 2 has pitch_factor=1.0, -1.0 input -> full reverse
    assert all(pwm == 1500 for m, pwm in out.items() if m != 2)


def test_full_up_throttle_drives_throttle_motor_forward(mixer):
    out = mixer.mix(roll=0.0, pitch=0.0, yaw=0.0, throttle_0_to_1=1.0,
                    forward=0.0, lateral=0.0, armed=True)
    # throttle_0_to_1=1.0 -> bidirectional +1.0
    assert out[4] == 1900
    assert all(pwm == 1500 for m, pwm in out.items() if m != 4)


def test_saturating_group_is_normalized_by_its_own_peak(mixer):
    # Drive the roll motor past 1.0: rpt_max becomes 2.0, so it scales
    # back to exactly 1.0 -> full forward PWM.
    out = mixer.mix(roll=2.0, pitch=0.0, yaw=0.0, throttle_0_to_1=0.5,
                    forward=0.0, lateral=0.0, armed=True)
    assert out[1] == 1900
    assert all(pwm == 1500 for m, pwm in out.items() if m != 1)


def test_saturation_does_not_leak_between_groups(mixer):
    """
    The vectored_6dof frame normalizes {roll,pitch,throttle} and
    {yaw,forward,lateral} INDEPENDENTLY (rpt_max / yfl_max), so a
    saturated vertical stack must not attenuate the horizontal thrusters.

    A single global peak scale -- which is what a naive port does -- would
    scale motor 5 down to 0.5 (1700us) here. Upstream leaves it at 1.0.
    """
    out = mixer.mix(roll=2.0, pitch=0.0, yaw=0.0, throttle_0_to_1=0.5,
                    forward=1.0, lateral=0.0, armed=True)
    assert out[1] == 1900   # roll motor: 2.0 / rpt_max(2.0) = 1.0
    assert out[5] == 1900   # forward motor: 1.0 / yfl_max(1.0) = 1.0, untouched
    assert all(pwm == 1500 for m, pwm in out.items() if m not in (1, 5))


def test_non_saturating_group_is_not_rescaled(mixer):
    """Both maxima start at 1.0, so a half-deflection stays a half."""
    out = mixer.mix(roll=0.5, pitch=0.0, yaw=0.0, throttle_0_to_1=0.5,
                    forward=0.0, lateral=0.0, armed=True)
    assert out[1] == 1700   # 1500 + 0.5 * 400


def test_motor_direction_inverts_that_motor_only():
    cfg = MixerConfig(pwm_min=1100, pwm_trim=1500, pwm_max=1900,
                      motor_direction={1: -1})
    mixer = SixDofMixer(str(FIXTURE), cfg)
    out = mixer.mix(roll=1.0, pitch=1.0, yaw=0.0, throttle_0_to_1=0.5,
                    forward=0.0, lateral=0.0, armed=True)
    assert out[1] == 1100   # reversed
    assert out[2] == 1900   # untouched
    assert all(pwm == 1500 for m, pwm in out.items() if m not in (1, 2))


def test_motor_direction_defaults_to_forward(mixer):
    """An unlisted motor is +1, matching the ArduSub parameter default."""
    assert mixer.mixer_config.direction_for(1) == 1
    assert mixer.mixer_config.direction_for(99) == 1


def test_motor_direction_rejects_non_unit_values():
    cfg = MixerConfig(motor_direction={1: 0})
    with pytest.raises(ValueError, match="must be \\+1 or -1"):
        cfg.direction_for(1)


def test_motor_numbers(mixer):
    assert mixer.motor_numbers() == [1, 2, 3, 4, 5, 6]
