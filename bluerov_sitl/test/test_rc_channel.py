from bluerov_sitl.rc_channel import RCChannel, ChannelMode


def make_angle_channel(deadzone=30, timeout=3.0):
    return RCChannel(
        name="test", mode=ChannelMode.ANGLE,
        pwm_min=1100, pwm_trim=1500, pwm_max=1900,
        deadzone_us=deadzone, override_timeout_s=timeout,
    )


def make_range_channel(deadzone=30, timeout=3.0):
    return RCChannel(
        name="throttle", mode=ChannelMode.RANGE,
        pwm_min=1100, pwm_trim=1500, pwm_max=1900,
        deadzone_us=deadzone, override_timeout_s=timeout,
    )


def test_no_override_returns_trim():
    ch = make_angle_channel()
    assert ch.get_pwm(now_s=0.0) == 1500
    assert ch.norm_input_dz(now_s=0.0) == 0.0


def test_override_ignore_sentinel_is_ignored():
    ch = make_angle_channel()
    ch.apply_override(0, now_s=1.0)  # OVERRIDE_IGNORE_VALUE
    assert ch.get_pwm(now_s=1.0) == 1500


def test_override_release_sentinel_reverts_to_trim():
    ch = make_angle_channel()
    ch.apply_override(1900, now_s=1.0)
    assert ch.get_pwm(now_s=1.0) == 1900
    ch.apply_override(65535, now_s=1.1)  # OVERRIDE_RELEASE_VALUE
    assert ch.get_pwm(now_s=1.1) == 1500


def test_override_full_deflection_normalizes_to_plus_minus_one():
    ch = make_angle_channel()
    ch.apply_override(1900, now_s=0.0)
    assert ch.norm_input_dz(now_s=0.0) == 1.0
    ch.apply_override(1100, now_s=0.0)
    assert ch.norm_input_dz(now_s=0.0) == -1.0


def test_deadzone_suppresses_small_deflection():
    ch = make_angle_channel(deadzone=30)
    ch.apply_override(1515, now_s=0.0)  # +15us, inside 30us deadzone
    assert ch.norm_input_dz(now_s=0.0) == 0.0


def test_override_goes_stale_after_timeout():
    ch = make_angle_channel(timeout=1.0)
    ch.apply_override(1900, now_s=0.0)
    assert ch.get_pwm(now_s=0.5) == 1900
    assert ch.get_pwm(now_s=1.5) == 1500  # stale -> reverted to trim
    assert ch.is_override_stale(now_s=1.5) is True


def test_range_channel_center_is_half():
    ch = make_range_channel()
    assert ch.norm_input_dz(now_s=0.0) == 0.5  # no override yet -> trim -> 0.5


def test_range_channel_full_up_is_one():
    ch = make_range_channel()
    ch.apply_override(1900, now_s=0.0)
    assert round(ch.norm_input_dz(now_s=0.0), 3) == 1.0


def test_range_channel_full_down_is_zero():
    ch = make_range_channel()
    ch.apply_override(1100, now_s=0.0)
    assert round(ch.norm_input_dz(now_s=0.0), 3) == 0.0


# ---------------------------------------------------------------------
# norm_input() -- the deadzone-free path MANUAL mode actually uses
# ---------------------------------------------------------------------


def test_norm_input_ignores_deadzone():
    """
    The whole point of norm_input() vs norm_input_dz(): a deflection
    inside the deadzone still produces output, and a deflection outside
    it is NOT rescaled to recover the lost span.
    """
    ch = make_angle_channel(deadzone=30)
    ch.apply_override(1515, now_s=0.0)          # inside the 30us deadzone
    assert ch.norm_input_dz(now_s=0.0) == 0.0   # suppressed
    assert ch.norm_input(now_s=0.0) == 0.0375   # 15/400, passed straight through


def test_norm_input_matches_real_vehicle_scaling():
    """
    +100us of stick on a 1100/1500/1900 channel drives the thruster
    +100us -- the 1:1 passthrough observed on the real BlueROV2 Heavy.
    norm_input_dz() would give 70/370 = 0.189 instead.
    """
    ch = make_angle_channel(deadzone=30)
    ch.apply_override(1600, now_s=0.0)
    assert ch.norm_input(now_s=0.0) == 0.25
    assert round(ch.norm_input_dz(now_s=0.0), 4) == 0.1892


def test_norm_input_full_deflection():
    ch = make_angle_channel()
    ch.apply_override(1900, now_s=0.0)
    assert ch.norm_input(now_s=0.0) == 1.0
    ch.apply_override(1100, now_s=0.0)
    assert ch.norm_input(now_s=0.0) == -1.0


def test_norm_input_scales_each_side_by_its_own_span():
    """
    Asymmetric range: the two sides of trim use different denominators,
    so full deflection is +-1.0 on both sides with no discontinuity.
    """
    ch = RCChannel(
        name="asym", mode=ChannelMode.ANGLE,
        pwm_min=1000, pwm_trim=1500, pwm_max=1700, deadzone_us=0,
    )
    ch.apply_override(1600, now_s=0.0)
    assert ch.norm_input(now_s=0.0) == 0.5      # 100/200
    ch.apply_override(1400, now_s=0.0)
    assert ch.norm_input(now_s=0.0) == -0.2     # -100/500
    ch.apply_override(1700, now_s=0.0)
    assert ch.norm_input(now_s=0.0) == 1.0
    ch.apply_override(1000, now_s=0.0)
    assert ch.norm_input(now_s=0.0) == -1.0


def test_throttle_0_to_1_is_centred_on_trim():
    ch = make_angle_channel(deadzone=30)
    assert ch.throttle_0_to_1(now_s=0.0) == 0.5     # no override -> trim
    ch.apply_override(1400, now_s=0.0)
    assert ch.throttle_0_to_1(now_s=0.0) == 0.375   # (-0.25 + 1) / 2
    ch.apply_override(1900, now_s=0.0)
    assert ch.throttle_0_to_1(now_s=0.0) == 1.0
    ch.apply_override(1100, now_s=0.0)
    assert ch.throttle_0_to_1(now_s=0.0) == 0.0


def test_throttle_0_to_1_diverges_from_range_mode_when_trim_is_off_centre():
    """
    With trim at the midpoint the two agree, which is why the old RANGE
    path went unnoticed on a 1100/1500/1900 channel. Move trim and they
    part company -- throttle_0_to_1() is the one that matches ArduSub.
    """
    kwargs = dict(pwm_min=1100, pwm_trim=1300, pwm_max=1900, deadzone_us=0)
    angle = RCChannel(name="t", mode=ChannelMode.ANGLE, **kwargs)
    rng = RCChannel(name="t", mode=ChannelMode.RANGE, **kwargs)
    for ch in (angle, rng):
        ch.apply_override(1300, now_s=0.0)       # sitting exactly on trim
    assert angle.throttle_0_to_1(now_s=0.0) == 0.5       # neutral, correct
    assert rng.norm_input_dz(now_s=0.0) == 0.25          # not neutral
