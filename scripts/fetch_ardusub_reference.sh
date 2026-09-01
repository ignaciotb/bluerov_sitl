#!/usr/bin/env bash
#
# fetch_ardusub_reference.sh
#
# Pulls the REAL ArduSub source out of ArduPilot/ardupilot into ./reference/
# so that every constant/matrix used by this package can be extracted
# straight from upstream instead of hand-copied (and therefore always
# in sync with "the latest ArduSub version", which is what was asked for).
#
# We sparse-checkout only the directories we actually need:
#   - ArduSub/                       (radio.cpp, mode.cpp, defines.h, Parameters.cpp, ...)
#   - libraries/AP_Motors/           (AP_Motors6DOF.cpp/.h - the thruster mixing matrix)
#   - libraries/RC_Channel/          (RC_Channel.cpp/.h, RC_Channels.cpp - override handling)
#   - libraries/AC_AttitudeControl/  (AC_AttitudeControl_Sub.h - STABILIZE gains)
#   - libraries/AC_PID/              (AC_PID.cpp - the rate loops)
#   - libraries/AP_Math/             (control.cpp sqrt_controller, quaternion.cpp)
#
# Usage:
#   ./fetch_ardusub_reference.sh [git-ref]
#
# git-ref defaults to "ArduSub-stable" (the tag BlueOS/BlueROV2 firmware
# builds are actually cut from). Pass "master" to track bleeding edge,
# or a specific tag/commit for reproducibility.

set -euo pipefail

REF="${1:-ArduSub-stable}"
DEST="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/reference/ardupilot"
REPO_URL="https://github.com/ArduPilot/ardupilot.git"

echo "==> Fetching ArduPilot/ardupilot @ ${REF} into ${DEST}"

if [ -d "${DEST}/.git" ]; then
    echo "==> Existing checkout found, updating in place"
    cd "${DEST}"
    git fetch --depth 1 origin "${REF}"
    git checkout FETCH_HEAD
else
    mkdir -p "${DEST}"
    cd "${DEST}"
    git init -q
    git remote add origin "${REPO_URL}"
    git config core.sparseCheckout true
    mkdir -p .git/info
    cat > .git/info/sparse-checkout <<'EOF'
ArduSub/*
libraries/AP_Motors/*
libraries/RC_Channel/*
libraries/AC_AttitudeControl/*
libraries/AC_PID/*
libraries/AP_Math/*
EOF
    git fetch --depth 1 origin "${REF}"
    git checkout FETCH_HEAD
fi

echo "==> Done. Vendored source lives under:"
echo "    ${DEST}"
echo ""
echo "Key files this package depends on (verify these exist):"
for f in \
    "ArduSub/radio.cpp" \
    "ArduSub/defines.h" \
    "ArduSub/mode.h" \
    "ArduSub/mode.cpp" \
    "ArduSub/Parameters.cpp" \
    "libraries/AP_Motors/AP_Motors6DOF.cpp" \
    "libraries/AP_Motors/AP_Motors6DOF.h" \
    "libraries/RC_Channel/RC_Channels.cpp"
do
    if [ -f "${DEST}/${f}" ]; then
        echo "    [ok]      ${f}"
    else
        echo "    [MISSING] ${f}  <-- extraction will fail on this, check the path upstream"
    fi
done

echo ""
echo "Next: run scripts/extract_motor_matrix.py to regenerate config/*.yaml from this checkout."
