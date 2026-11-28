#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/noetic/setup.bash
set -u

BUNDLE_DIR="${HUMAN_FALL_BUNDLE_DIR:-/opt/lidar-human-fall/bundle}"
if [[ ! -f "${BUNDLE_DIR}/MANIFEST.json" || ! -f "${BUNDLE_DIR}/RELEASE.json" ]]; then
    echo "ERROR: Missing immutable bundle (MANIFEST.json / RELEASE.json) in ${BUNDLE_DIR}" >&2
    exit 2
fi

# Verify every payload file against MANIFEST.json and extract release metadata.
eval "$(python3 -B - "${BUNDLE_DIR}" <<'PY'
import hashlib
import json
import shlex
import sys
from pathlib import Path

root = Path(sys.argv[1])
manifest = json.loads((root / "MANIFEST.json").read_text(encoding="utf-8"))
for rel, info in sorted(manifest.items()):
    path = root / rel
    raw = path.read_bytes()
    if len(raw) != info["bytes"] or hashlib.sha256(raw).hexdigest() != info["sha256"]:
        raise SystemExit(f"MANIFEST mismatch for {rel}")

release = json.loads((root / "RELEASE.json").read_text(encoding="utf-8"))
print(f"RELEASE_ID={shlex.quote(str(release['release_id']))}")
print(f"ENTRY_REL={shlex.quote(str(release['entry']))}")
print(f"CONFIG_REL={shlex.quote(str(release['config']))}")
print(f"STATE_TOPIC={shlex.quote(str(release.get('canonical_state_topic', '/single_target_live/state')))}")
PY
)"

echo "HUMAN_FALL_BUNDLE_VERIFIED release_id=${RELEASE_ID} dir=${BUNDLE_DIR}"

# Wait for existing slam-localization roscore before launching recognition node.
WAIT_SEC="${ROS_MASTER_WAIT_SEC:-60}"
python3 -B - "${WAIT_SEC}" <<'PY'
import sys
import time
import rosgraph

deadline = time.monotonic() + float(sys.argv[1])
while True:
    try:
        rosgraph.Master("/human_fall_container_wait").getSystemState()
        print("ROS_MASTER_READY", flush=True)
        break
    except Exception as exc:
        if time.monotonic() >= deadline:
            raise SystemExit(f"ROS Master unreachable after {sys.argv[1]}s: {exc}")
        time.sleep(1.0)
PY

CMD=(
    python3 -u -B "${BUNDLE_DIR}/${ENTRY_REL}"
    --config "${BUNDLE_DIR}/${CONFIG_REL}"
    --release-id "${RELEASE_ID}"
)

MODE="${HUMAN_FALL_MODE:-instant_start}"
case "${MODE}" in
    instant_start)
        CMD+=(
            --instant-start
            --mount-pitch-deg "${HUMAN_FALL_MOUNT_PITCH_DEG:-26.0}"
            --mount-roll-deg "${HUMAN_FALL_MOUNT_ROLL_DEG:-0.0}"
        )
        if [[ "${HUMAN_FALL_DISABLE_IMU:-1}" == "1" ]]; then
            CMD+=(--disable-imu)
        elif [[ -n "${HUMAN_FALL_IMU_TOPIC:-}" ]]; then
            CMD+=(--imu-topic "${HUMAN_FALL_IMU_TOPIC}")
        fi
        ;;
    empty_prefix)
        CMD+=(--declare-empty-prefix)
        if [[ -n "${HUMAN_FALL_FIT_GROUND_OUT:-}" && -n "${HUMAN_FALL_SITE_ID:-}" ]]; then
            CMD+=(--fit-ground-out "${HUMAN_FALL_FIT_GROUND_OUT}" --site-id "${HUMAN_FALL_SITE_ID}")
        fi
        ;;
    *)
        echo "ERROR: Unsupported HUMAN_FALL_MODE=${MODE} (use instant_start or empty_prefix)" >&2
        exit 2
        ;;
esac

if [[ "${HUMAN_FALL_PHYSICAL_V1:-0}" == "1" ]]; then
    CMD+=(--human-physical-v1)
fi
if [[ "${HUMAN_FALL_SINGLE_PERSON_LOCK:-0}" == "1" ]]; then
    CMD+=(--single-person-lock)
fi
if [[ "${HUMAN_FALL_DIAGNOSTICS:-0}" == "1" ]]; then
    CMD+=(--diagnostics)
fi

echo "Launching lidar-human-fall: ${CMD[*]}"
exec "${CMD[@]}"
