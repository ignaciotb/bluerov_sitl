"""
rc_channel.py

A small, from-scratch reimplementation of the pieces of ArduPilot's
RC_Channel / RC_Channels classes that ArduSub's MANUAL mode actually
depends on:

  - storing an "override" PWM value with a timestamp and a timeout
    (this is exactly what happens when mavros forwards your
    mavros_msgs/OverrideRCIn publish as a MAVLink RC_CHANNELS_OVERRIDE
    message: RC_Channels::set_override(chan, value, tnow) in the real
    firmware)
  - the sentinel values used by the real override handler:
        0      -> "don't touch this channel" (ignored)
        65535  -> "release the override" (revert to trim / other source)
  - normalization, in the two forms the real RC_Channel offers:
        * norm_input()    -- PWM mapped symmetrically about trim to
          -1..+1, with NO deadzone
        * norm_input_dz() -- the same, but with the channel deadzone
          subtracted and the remaining span rescaled

    ArduSub's MANUAL mode uses norm_input(), NOT norm_input_dz():
    ArduSub/mode_manual.cpp calls channel_*->norm_input() on every axis
    and channel_throttle->norm_input() for throttle. There is no
    deadzone anywhere in the MANUAL path. Use norm_input() to reproduce
    MANUAL mode; norm_input_dz() is here because the stabilized modes do
    use it.

  - the "range" channel scaling (ArduSub/radio.cpp:
    channel_throttle->set_range(1000)). NOTE this is NOT how MANUAL mode
    reads throttle -- see throttle_0_to_1() below.

Defaults for pwm_min/trim/max and deadzone match ArduSub/radio.cpp's
Sub::init_rc_in(); the deadzone values are auto-extracted into
config/motor_matrix_vectored_6dof.yaml by scripts/extract_motor_matrix.py
-- pass them in explicitly rather than relying on the fallback defaults
here when you want guaranteed parity with a specific ArduSub version.
"""
from dataclasses import dataclass
from enum import Enum


class ChannelMode(Enum):
    ANGLE = "angle"   # symmetric about trim: -1..+1
    RANGE = "range"   # 0..1 from radio_min..radio_max (throttle only)


# ArduSub RC_CHANNELS_OVERRIDE sentinel values (see RC_Channels::set_override
# handling in libraries/RC_Channel/RC_Channels.cpp upstream).
OVERRIDE_IGNORE_VALUE = 0
OVERRIDE_RELEASE_VALUE = 65535

# Default override staleness timeout. ArduPilot's RC_Channels enforces a
# timeout on overrides so a dropped ROS node/mavros link doesn't leave a
# thruster pinned at its last commanded value forever.
#
# VERIFY THIS against your vendored copy: grep for override timeout
# handling in libraries/RC_Channel/RC_Channels.cpp (e.g.
# "_gcs_overrides_enabled" / the millis() comparison guarding
# rc_channel->set_override()) for the exact current constant -- it has
# moved between a hardcoded value and a parameter across ArduSub releases.
DEFAULT_OVERRIDE_TIMEOUT_S = 3.0


@dataclass
class RCChannel:
    name: str
    mode: ChannelMode
    pwm_min: int = 1100
    pwm_trim: int = 1500
    pwm_max: int = 1900
    deadzone_us: int = 30
    range_max: float = 1000.0  # only used in RANGE mode (throttle)
    # high_in for an ANGLE channel: ArduSub/radio.cpp Sub::init_rc_in() calls
    # set_angle(ROLL_PITCH_INPUT_MAX) on roll/pitch/yaw/forward/lateral, and
    # ROLL_PITCH_INPUT_MAX is 4500 (ArduSub/config.h:149). get_control_in()
    # therefore returns centidegrees in -4500..+4500.
    angle_max_cd: int = 4500
    override_timeout_s: float = DEFAULT_OVERRIDE_TIMEOUT_S

    def __post_init__(self):
        self._override_pwm: int = self.pwm_trim
        self._override_time_s: float = 0.0
        self._has_override: bool = False

    def apply_override(self, raw_value: int, now_s: float) -> None:
        """
        Mirrors RC_Channels' handling of one element of a
        RC_CHANNELS_OVERRIDE message / mavros_msgs/OverrideRCIn.channels[i].
        """
        if raw_value == OVERRIDE_IGNORE_VALUE:
            return  # "don't touch this channel"
        if raw_value == OVERRIDE_RELEASE_VALUE:
            self._has_override = False
            return  # "release the override"
        self._override_pwm = int(raw_value)
        self._override_time_s = now_s
        self._has_override = True

    def get_pwm(self, now_s: float) -> int:
        """
        Current effective PWM for this channel: the last override value if
        it hasn't gone stale, otherwise trim (this is the safety behaviour:
        a dropped ROS node results in that axis going neutral, not frozen).
        """
        if self._has_override and (now_s - self._override_time_s) <= self.override_timeout_s:
            return self._override_pwm
        return self.pwm_trim

    def is_override_stale(self, now_s: float) -> bool:
        if not self._has_override:
            return True
        return (now_s - self._override_time_s) > self.override_timeout_s

    def norm_input_dz(self, now_s: float) -> float:
        """
        Deadzone + normalize, matching RC_Channel::norm_input_dz() semantics.
        ANGLE channels  -> -1.0 .. +1.0 about pwm_trim
        RANGE channel   ->  0.0 .. +1.0 from pwm_min .. pwm_max
        """
        pwm = self.get_pwm(now_s)

        if self.mode is ChannelMode.ANGLE:
            # Direct port of RC_Channel::norm_input_dz(), guards included.
            dz_min = self.pwm_trim - self.deadzone_us
            dz_max = self.pwm_trim + self.deadzone_us
            if pwm < dz_min and dz_min > self.pwm_min:
                ret = (pwm - dz_min) / (dz_min - self.pwm_min)
            elif pwm > dz_max and self.pwm_max > dz_max:
                ret = (pwm - dz_max) / (self.pwm_max - dz_max)
            else:
                ret = 0.0
            return max(-1.0, min(1.0, ret))

        # RANGE (throttle): deadzone centered on the channel's own midpoint,
        # normalized to 0..1 across the full pwm_min..pwm_max span.
        # This mirrors set_range()/get_control_in(), which the STABILIZED
        # modes use -- MANUAL mode does NOT go through here.
        mid = (self.pwm_min + self.pwm_max) / 2.0
        offset = pwm - mid
        if abs(offset) <= self.deadzone_us:
            offset = 0.0
        span = max((self.pwm_max - self.pwm_min) / 2.0 - self.deadzone_us, 1)
        return max(0.0, min(1.0, 0.5 + (offset / span) * 0.5))

    def norm_input(self, now_s: float) -> float:
        """
        Deadzone-free normalization about trim, a direct port of
        RC_Channel::norm_input() (libraries/RC_Channel/RC_Channel.cpp):

            radio_in < radio_trim -> (radio_in - radio_trim) / (radio_trim - radio_min)
            otherwise             -> (radio_in - radio_trim) / (radio_max  - radio_trim)

        THIS is what ArduSub's MANUAL mode calls on every axis. Note the
        asymmetric denominators: the deflection either side of trim is
        scaled by that side's own span, so an off-center trim does not
        produce a discontinuity.
        """
        pwm = self.get_pwm(now_s)

        if pwm < self.pwm_trim:
            if self.pwm_min >= self.pwm_trim:
                return 0.0
            ret = (pwm - self.pwm_trim) / (self.pwm_trim - self.pwm_min)
        else:
            if self.pwm_max <= self.pwm_trim:
                return 0.0
            ret = (pwm - self.pwm_trim) / (self.pwm_max - self.pwm_trim)

        return max(-1.0, min(1.0, ret))

    def throttle_0_to_1(self, now_s: float) -> float:
        """
        Throttle as MANUAL mode computes it (ArduSub/mode_manual.cpp):

            motors.set_throttle((channel_throttle->norm_input() + 1) / 2)

        i.e. the deadzone-free -1..+1 about TRIM, shifted into 0..1 --
        not the set_range() 0..1 across [pwm_min, pwm_max] that
        norm_input_dz() returns for a RANGE channel. The two agree only
        when trim happens to sit exactly at the midpoint of min/max.
        """
        return (self.norm_input(now_s) + 1.0) / 2.0

    def pwm_to_angle_dz_trim(self, dead_zone: int, trim: int, now_s: float) -> float:
        """
        Direct port of RC_Channel::pwm_to_angle_dz_trim().

        Like norm_input_dz() but scaled to +-angle_max_cd rather than +-1,
        and with the deadzone and trim passed in rather than taken from the
        channel -- STABILIZE uses that to shrink the yaw deadzone by the
        pilot gain (mode_stabilize.cpp:35).
        """
        pwm = self.get_pwm(now_s)
        trim_high = trim + dead_zone
        trim_low = trim - dead_zone
        r_in = max(self.pwm_min, min(self.pwm_max, pwm))

        if r_in > trim_high and self.pwm_max != trim_high:
            return self.angle_max_cd * (r_in - trim_high) / (self.pwm_max - trim_high)
        if r_in < trim_low and trim_low != self.pwm_min:
            return self.angle_max_cd * (r_in - trim_low) / (trim_low - self.pwm_min)
        return 0.0

    def get_control_in(self, now_s: float) -> float:
        """
        RC_Channel::get_control_in() for an ANGLE channel, which is what
        RC_Channel::update() caches via pwm_to_angle() ->
        pwm_to_angle_dz(dead_zone). Centidegrees, -4500..+4500.

        Note this DOES apply the channel deadzone -- unlike MANUAL mode,
        STABILIZE reads roll/pitch through here, so the deadzone is live on
        those axes.
        """
        return self.pwm_to_angle_dz_trim(self.deadzone_us, self.pwm_trim, now_s)
