#!/usr/bin/env python3
"""
extract_motor_matrix.py

Parses the vendored ArduSub source (see fetch_ardusub_reference.sh) and
regenerates the ROS2 package's YAML config directly from it:

  - libraries/AP_Motors/AP_Motors6DOF.cpp
        -> the add_motor_raw_6dof(...) calls for the requested frame
           (default: SUB_FRAME_VECTORED_6DOF, i.e. the BlueROV2 Heavy)
  - ArduSub/defines.h
        -> ROLL_PITCH_INPUT_MAX (the +/- range used for angle-type RC
           input channels: roll, pitch, yaw, forward, lateral)
  - ArduSub/radio.cpp
        -> the default per-channel deadzones set in Sub::init_rc_in()

This is intentionally a *parser*, not a hand-maintained lookup table:
re-run it after re-running fetch_ardusub_reference.sh against a newer
ArduSub tag and the mixing matrix used by the ROS2 node updates itself,
instead of silently drifting from upstream.

Usage:
    python3 extract_motor_matrix.py \
        --ardupilot-root ../reference/ardupilot \
        --frame SUB_FRAME_VECTORED_6DOF \
        --out ../ros2_ws/src/bluerov_sitl/config/motor_matrix_vectored_6dof.yaml
"""
import argparse
import re
import sys
from pathlib import Path

import yaml

ADD_MOTOR_CALL_RE = re.compile(
    r"add_motor_raw_6dof\s*\(\s*(?P<args>[^;]+?)\s*\)\s*;",
    re.MULTILINE,
)

# Matches "case SUB_FRAME_VECTORED_6DOF:" or "case AP_Motors6DOF::SUB_FRAME_VECTORED_6DOF:"
# (upstream has renamed the enum's qualification a few times over the years).
CASE_RE_TEMPLATE = r"case\s+(?:AP_Motors6DOF::)?{frame}\s*:"


def strip_comments(src: str) -> str:
    """Remove // and /* */ comments so they don't confuse the arg parser."""
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.DOTALL)
    src = re.sub(r"//[^\n]*", "", src)
    return src


def find_case_block(src: str, frame_enum: str) -> str:
    """
    Return the source text of the `case <frame_enum>: ... break;`
    (or up to the next `case`) block inside setup_motors()/init().
    """
    case_re = re.compile(CASE_RE_TEMPLATE.format(frame=re.escape(frame_enum)))
    m = case_re.search(src)
    if not m:
        raise ValueError(
            f"Could not find 'case {frame_enum}:' in AP_Motors6DOF.cpp. "
            f"Upstream may have renamed the enum -- open the file and check "
            f"the 'Supported frame types' switch inside setup_motors()."
        )
    start = m.end()
    # Block ends at the next top-level `case` or `break;`, whichever comes first.
    next_case = re.search(r"\bcase\s+", src[start:])
    next_break = re.search(r"\bbreak\s*;", src[start:])
    ends = [x.start() for x in (next_case, next_break) if x]
    if not ends:
        raise ValueError(f"Could not find end of 'case {frame_enum}:' block")
    end = start + min(ends)
    return src[start:end]


def parse_motor_calls(block: str):
    """
    Each add_motor_raw_6dof() call is:
        add_motor_raw_6dof(motor_num, roll_fac, pitch_fac, yaw_fac,
                            throttle_fac, forward_fac, lat_fac, testing_order)
    Motor numbers are macros like AP_MOTORS_MOT_1 .. AP_MOTORS_MOT_8.
    """
    motors = []
    for call in ADD_MOTOR_CALL_RE.finditer(block):
        raw_args = [a.strip() for a in call.group("args").split(",")]
        # add_motor_raw_6dof(motor_num, roll_fac, pitch_fac, yaw_fac,
        #                     throttle_fac, forward_fac, lat_fac, testing_order)
        if len(raw_args) != 8:
            raise ValueError(f"Unexpected add_motor_raw_6dof arity ({len(raw_args)} "
                              f"args, expected 8): {raw_args}")
        motor_macro, roll, pitch, yaw, throttle, forward, lateral, _testing_order = raw_args
        # motor_macro looks like "AP_MOTORS_MOT_5" -> we want the trailing int
        mnum_match = re.search(r"(\d+)\s*$", motor_macro)
        if not mnum_match:
            raise ValueError(f"Could not parse motor number from '{motor_macro}'")
        motor_index = int(mnum_match.group(1))

        def as_float(tok: str) -> float:
            tok = tok.replace("f", "").replace("F", "").strip()
            return float(tok)

        motors.append(
            {
                "motor": motor_index,
                # NOTE: field order in the C++ call is
                # (roll, pitch, yaw, throttle, forward, lateral)
                "roll_factor": as_float(roll),
                "pitch_factor": as_float(pitch),
                "yaw_factor": as_float(yaw),
                "throttle_factor": as_float(throttle),
                "forward_factor": as_float(forward),
                "lateral_factor": as_float(lateral),
            }
        )
    motors.sort(key=lambda m: m["motor"])
    return motors


def extract_roll_pitch_input_max(ardusub_dir: Path) -> int:
    defines = ardusub_dir / "defines.h"
    if not defines.exists():
        print(f"[warn] {defines} not found, defaulting ROLL_PITCH_INPUT_MAX to 4500",
              file=sys.stderr)
        return 4500
    text = strip_comments(defines.read_text())
    m = re.search(r"#define\s+ROLL_PITCH_INPUT_MAX\s+(\d+)", text)
    if not m:
        print(f"[warn] ROLL_PITCH_INPUT_MAX not found in {defines}, defaulting to 4500",
              file=sys.stderr)
        return 4500
    return int(m.group(1))


def extract_deadzones(ardusub_dir: Path) -> dict:
    """
    Pulls the six set_default_dead_zone(N) calls out of Sub::init_rc_in()
    in ArduSub/radio.cpp, in the order they're declared there:
    roll, pitch, throttle, yaw, forward, lateral.
    """
    radio_cpp = ardusub_dir / "radio.cpp"
    defaults = {"roll": 30, "pitch": 30, "throttle": 30, "yaw": 40, "forward": 30, "lateral": 30}
    if not radio_cpp.exists():
        print(f"[warn] {radio_cpp} not found, using known-good defaults {defaults}",
              file=sys.stderr)
        return defaults

    text = strip_comments(radio_cpp.read_text())
    result = {}
    for axis in defaults:
        m = re.search(
            rf"channel_{axis}\s*->\s*set_default_dead_zone\s*\(\s*(\d+)\s*\)", text
        )
        if m:
            result[axis] = int(m.group(1))
        else:
            print(f"[warn] deadzone for '{axis}' not found in radio.cpp, "
                  f"using default {defaults[axis]}", file=sys.stderr)
            result[axis] = defaults[axis]
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ardupilot-root", required=True, type=Path,
                     help="Path to the vendored ardupilot checkout "
                          "(reference/ardupilot from fetch_ardusub_reference.sh)")
    ap.add_argument("--frame", default="SUB_FRAME_VECTORED_6DOF",
                     help="Frame enum to extract (BlueROV2 Heavy = SUB_FRAME_VECTORED_6DOF)")
    ap.add_argument("--out", required=True, type=Path,
                     help="Where to write the generated YAML")
    args = ap.parse_args()

    motors_cpp = args.ardupilot_root / "libraries/AP_Motors/AP_Motors6DOF.cpp"
    ardusub_dir = args.ardupilot_root / "ArduSub"

    if not motors_cpp.exists():
        print(f"ERROR: {motors_cpp} not found. Did you run "
              f"fetch_ardusub_reference.sh first?", file=sys.stderr)
        sys.exit(1)

    src = strip_comments(motors_cpp.read_text())
    block = find_case_block(src, args.frame)
    motors = parse_motor_calls(block)

    if not motors:
        print(f"ERROR: found the '{args.frame}' case but no add_motor_raw_6dof() "
              f"calls inside it. The upstream implementation may have changed "
              f"structure -- inspect {motors_cpp} manually.", file=sys.stderr)
        sys.exit(1)

    roll_pitch_input_max = extract_roll_pitch_input_max(ardusub_dir)
    deadzones = extract_deadzones(ardusub_dir)

    out_doc = {
        "_source": {
            "note": "AUTO-GENERATED by scripts/extract_motor_matrix.py -- do not hand-edit. "
                    "Re-run the extractor after re-fetching the reference source instead.",
            "frame_enum": args.frame,
            "extracted_from": "libraries/AP_Motors/AP_Motors6DOF.cpp",
        },
        "frame_class": args.frame,
        "num_motors": len(motors),
        "motors": motors,
        "rc_input": {
            "roll_pitch_input_max": roll_pitch_input_max,
            "deadzone_us": deadzones,
        },
    }

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, "w") as f:
        yaml.safe_dump(out_doc, f, sort_keys=False, default_flow_style=False)

    print(f"==> Wrote {len(motors)} motors for frame '{args.frame}' to {args.out}")
    for m in motors:
        print(f"    motor {m['motor']}: roll={m['roll_factor']:+.3f} "
              f"pitch={m['pitch_factor']:+.3f} yaw={m['yaw_factor']:+.3f} "
              f"throttle={m['throttle_factor']:+.3f} forward={m['forward_factor']:+.3f} "
              f"lateral={m['lateral_factor']:+.3f}")
    print(f"==> ROLL_PITCH_INPUT_MAX = {roll_pitch_input_max}")
    print(f"==> deadzones (us) = {deadzones}")


if __name__ == "__main__":
    main()
