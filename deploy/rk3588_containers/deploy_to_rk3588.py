"""Stage and deploy two isolated RK3588 Docker containers (lidar-human-fall + lidar-bird).

Safety invariants:
- Never modifies or restarts `slam-localization` driver components (roscore, inno_lidar_node,
  foxglove_bridge :8765, capture :8766) or any recording/calibration data outside its deploy root.
- Builds both Docker images before stopping legacy in-container range_service_node.py.
- Supports 100% offline RK3588 builds by auto-detecting `slam-localization`'s local base image.
- Works offline locally with `--stage-only` to produce a checksummed staging archive ready to push.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_HUMAN_FALL_TAR = Path(
    r"D:\Code\ldiar\docs\evidence\2026-11-25_board_instant_deploy\range_service_instant_v1.tar"
)
DEFAULT_REMOTE_ROOT = "/home/wel/.local/opt/lidar-rk3588"

STAGE_REPO_FILES = [
    "firmware/esp32s3_bird_core/bird_tracker.hpp",
    "firmware/esp32s3_bird_core/grid_algorithm.hpp",
    "firmware/esp32s3_bird_core/grid_algorithm.cpp",
    "firmware/esp32s3_bird_core/tracker.cpp",
    "firmware/esp32s3_bird_core/geometry.cpp",
    "deploy/rk3588_containers/docker-compose.yml",
    "deploy/rk3588_containers/.env.example",
    "deploy/rk3588_containers/bird/Dockerfile",
    "deploy/rk3588_containers/bird/bird_core_c_api.cpp",
    "deploy/rk3588_containers/bird/native_tracker.py",
    "deploy/rk3588_containers/bird/pointcloud_grid.py",
    "deploy/rk3588_containers/bird/foxglove_client.py",
    "deploy/rk3588_containers/bird/bird_service.py",
    "deploy/rk3588_containers/human_fall/Dockerfile",
    "deploy/rk3588_containers/human_fall/entrypoint.sh",
    "deploy/rk3588_containers/human_fall/healthcheck.py",
    "web/s3_live_server.py",
    "web/s3_live.html",
    "bird_audit/raw_capture.py",
]


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inspect_human_fall_bundle(tar_path):
    """Validate immutable human_fall bundle tar and extract RELEASE + IFW192S CSV tables."""
    tar_path = Path(tar_path)
    if not tar_path.is_file():
        raise FileNotFoundError(f"Human-fall bundle tar not found: {tar_path}")

    with tarfile.open(tar_path, "r:*") as tf:
        members = {m.name.lstrip("./"): m for m in tf.getmembers() if m.isfile()}
        if "MANIFEST.json" not in members or "RELEASE.json" not in members:
            raise ValueError(f"Bundle missing MANIFEST.json or RELEASE.json: {tar_path}")

        manifest = json.loads(tf.extractfile(members["MANIFEST.json"]).read().decode("utf-8"))
        release = json.loads(tf.extractfile(members["RELEASE.json"]).read().decode("utf-8"))

        for rel, info in sorted(manifest.items()):
            if rel not in members:
                raise ValueError(f"MANIFEST entry missing from tar: {rel}")
            raw = tf.extractfile(members[rel]).read()
            if len(raw) != info["bytes"] or sha256_bytes(raw) != info["sha256"]:
                raise ValueError(f"MANIFEST checksum mismatch inside tar for {rel}")

        az_key = "src/inno_lidar_ros/config/ifw192s/ifw192s_azimuth_192x256.csv"
        el_key = "src/inno_lidar_ros/config/ifw192s/ifw192s_elevation_192x256.csv"
        if az_key not in members or el_key not in members:
            raise ValueError("IFW192S azimuth/elevation CSV tables missing from human-fall bundle")

        az_bytes = tf.extractfile(members[az_key]).read()
        el_bytes = tf.extractfile(members[el_key]).read()

    return {
        "release": release,
        "manifest_count": len(manifest),
        "tar_sha256": sha256_file(tar_path),
        "azimuth_csv_bytes": az_bytes,
        "elevation_csv_bytes": el_bytes,
    }


def build_remote_deploy_script(config):
    """Generate LF-only bash script executed on RK3588 to build and launch both containers."""
    lines = [
        "#!/usr/bin/env bash",
        "set -euo pipefail",
        "",
        f"DEPLOY_ROOT={json.dumps(config['remote_root'])}",
        f"RELEASE_ID={json.dumps(config['release_id'])}",
        f"HUMAN_FALL_TAR_SHA={json.dumps(config['human_fall_tar_sha'])}",
        f"BIRD_HTTP_PORT={json.dumps(str(config['bird_http_port']))}",
        f"HUMAN_FALL_CPUSET={json.dumps(config['human_fall_cpuset'])}",
        f"BIRD_CPUSET={json.dumps(config['bird_cpuset'])}",
        f"HUMAN_FALL_MODE={json.dumps(config['human_fall_mode'])}",
        f"MOUNT_PITCH_DEG={json.dumps(str(config['mount_pitch_deg']))}",
        f"MOUNT_ROLL_DEG={json.dumps(str(config['mount_roll_deg']))}",
        f"DISABLE_IMU={json.dumps('1' if config['disable_imu'] else '0')}",
        f"PHYSICAL_V1={json.dumps('1' if config['with_human_physical_v1'] else '0')}",
        f"SINGLE_LOCK={json.dumps('1' if config['single_person_lock'] else '0')}",
        f"OVERRIDE_HF_BASE={json.dumps(config['human_fall_base_image'] or '')}",
        f"OVERRIDE_BIRD_BASE={json.dumps(config['bird_base_image'] or '')}",
        f"USE_CAL_BIN={json.dumps('1' if config['has_calibration_bin'] else '0')}",
        "",
        'STAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"',
        'echo "=== [1/7] Verify slam-localization driver container ==="',
        'if ! docker inspect -f "{{.State.Running}}" slam-localization 2>/dev/null | grep -q "^true$"; then',
        '    echo "ERROR: slam-localization container is not running; refusing to deploy." >&2',
        "    exit 2",
        "fi",
        'SLAM_IMAGE="$(docker inspect -f "{{.Config.Image}}" slam-localization)"',
        'SLAM_ROS_MASTER="$(docker inspect -f "{{range .Config.Env}}{{println .}}{{end}}" slam-localization | awk -F= \'$1=="ROS_MASTER_URI"{print $2; exit}\')"',
        'ROS_MASTER_URI="${SLAM_ROS_MASTER:-http://127.0.0.1:11311}"',
        'HF_BASE="${OVERRIDE_HF_BASE:-$SLAM_IMAGE}"',
        'BIRD_BASE="${OVERRIDE_BIRD_BASE:-$SLAM_IMAGE}"',
        'echo "Detected slam-localization image=${SLAM_IMAGE} ROS_MASTER_URI=${ROS_MASTER_URI}"',
        "",
        'echo "=== [2/7] Install immutable human-fall bundle & shared calibration ==="',
        'echo "${HUMAN_FALL_TAR_SHA}  ${STAGE_DIR}/human_fall_bundle.tar" | sha256sum -c -',
        'REL_DIR="${DEPLOY_ROOT}/releases/${RELEASE_ID}/bundle"',
        'CAL_DIR="${DEPLOY_ROOT}/calibration"',
        'REPO_DIR="${DEPLOY_ROOT}/repo"',
        'mkdir -p "${REL_DIR}" "${CAL_DIR}" "${DEPLOY_ROOT}/bundles"',
        'if [[ ! -f "${REL_DIR}/MANIFEST.json" ]]; then',
        '    tar -xf "${STAGE_DIR}/human_fall_bundle.tar" -C "${REL_DIR}"',
        "fi",
        'ln -sfn "${REL_DIR}" "${DEPLOY_ROOT}/bundles/human_fall"',
        'cp -f "${STAGE_DIR}/calibration/ifw192s_azimuth_192x256.csv" "${CAL_DIR}/"',
        'cp -f "${STAGE_DIR}/calibration/ifw192s_elevation_192x256.csv" "${CAL_DIR}/"',
        'if [[ "${USE_CAL_BIN}" == "1" && -f "${STAGE_DIR}/calibration/calibration_sdk_geometry.bin" ]]; then',
        '    cp -f "${STAGE_DIR}/calibration/calibration_sdk_geometry.bin" "${CAL_DIR}/"',
        "fi",
        'rm -rf "${REPO_DIR}"',
        'cp -a "${STAGE_DIR}/repo" "${REPO_DIR}"',
        "",
        'echo "=== [3/7] Write .env for compose stack ==="',
        'BIRD_CAL_ENV=""',
        'if [[ "${USE_CAL_BIN}" == "1" ]]; then',
        '    BIRD_CAL_ENV="/config/calibration/calibration_sdk_geometry.bin"',
        "fi",
        'cat > "${REPO_DIR}/deploy/rk3588_containers/.env" <<EOF',
        'HUMAN_FALL_BASE_IMAGE=${HF_BASE}',
        'BIRD_BASE_IMAGE=${BIRD_BASE}',
        'ROS_MASTER_URI=${ROS_MASTER_URI}',
        'ROS_IP=127.0.0.1',
        'BIRD_FOXGLOVE_URL=ws://127.0.0.1:8765',
        'BIRD_INPUT_TOPIC=/innolidar_points',
        'HUMAN_FALL_BUNDLE_HOST_DIR=${REL_DIR}',
        'HUMAN_FALL_CPUSET=${HUMAN_FALL_CPUSET}',
        'HUMAN_FALL_MEM_LIMIT=1536m',
        'HUMAN_FALL_MODE=${HUMAN_FALL_MODE}',
        'HUMAN_FALL_MOUNT_PITCH_DEG=${MOUNT_PITCH_DEG}',
        'HUMAN_FALL_MOUNT_ROLL_DEG=${MOUNT_ROLL_DEG}',
        'HUMAN_FALL_DISABLE_IMU=${DISABLE_IMU}',
        'HUMAN_FALL_PHYSICAL_V1=${PHYSICAL_V1}',
        'HUMAN_FALL_SINGLE_PERSON_LOCK=${SINGLE_LOCK}',
        'HUMAN_FALL_DIAGNOSTICS=0',
        'BIRD_CALIBRATION_HOST_DIR=${CAL_DIR}',
        'BIRD_CPUSET=${BIRD_CPUSET}',
        'BIRD_MEM_LIMIT=512m',
        'BIRD_HTTP_HOST=0.0.0.0',
        'BIRD_HTTP_PORT=${BIRD_HTTP_PORT}',
        'BIRD_AZIMUTH_CSV=/config/calibration/ifw192s_azimuth_192x256.csv',
        'BIRD_ELEVATION_CSV=/config/calibration/ifw192s_elevation_192x256.csv',
        'BIRD_CALIBRATION=${BIRD_CAL_ENV}',
        "EOF",
        "",
        'echo "=== [4/7] Build lidar-human-fall and lidar-bird images before cutover ==="',
        'docker build --build-arg BASE_IMAGE="${HF_BASE}" \\',
        '    -f "${REPO_DIR}/deploy/rk3588_containers/human_fall/Dockerfile" \\',
        '    -t lidar-human-fall:latest "${REPO_DIR}/deploy/rk3588_containers/human_fall"',
        'docker build --build-arg BASE_IMAGE="${BIRD_BASE}" \\',
        '    -f "${REPO_DIR}/deploy/rk3588_containers/bird/Dockerfile" \\',
        '    -t lidar-bird:latest "${REPO_DIR}"',
        "",
        'echo "=== [5/7] Stop ONLY legacy range_service_node.py & conflicting port-8895 viewer ==="',
        'docker exec slam-localization bash -lc \'',
        'OLD_PIDS="$(pgrep -f "[r]ange_service_node.py" || true)"',
        'echo "OLD_RANGE_PIDS=${OLD_PIDS}"',
        'if [[ -n "${OLD_PIDS}" ]]; then',
        '    kill -INT ${OLD_PIDS} || true',
        '    for _ in $(seq 1 40); do',
        '        pgrep -f "[r]ange_service_node.py" >/dev/null 2>&1 || break',
        "        sleep 0.2",
        "    done",
        '    if pgrep -f "[r]ange_service_node.py" >/dev/null 2>&1; then',
        '        kill -TERM $(pgrep -f "[r]ange_service_node.py") || true',
        "        sleep 1",
        "    fi",
        "fi",
        "'",
        'if [[ "${BIRD_HTTP_PORT}" == "8895" ]] && command -v systemctl >/dev/null 2>&1; then',
        '    if systemctl --user is-active --quiet lidar-bird-s3.service 2>/dev/null; then',
        '        echo "Stopping legacy ESP32-S3 viewer lidar-bird-s3.service to free port 8895 (files preserved)"',
        "        systemctl --user disable --now lidar-bird-s3.service || true",
        "    fi",
        "fi",
        "",
        'echo "=== [6/7] Start two independent containers ==="',
        'COMPOSE_DIR="${REPO_DIR}/deploy/rk3588_containers"',
        'if docker compose version >/dev/null 2>&1; then',
        '    (cd "${COMPOSE_DIR}" && docker compose up -d --force-recreate)',
        'elif command -v docker-compose >/dev/null 2>&1; then',
        '    (cd "${COMPOSE_DIR}" && docker-compose up -d --force-recreate)',
        "else",
        '    echo "docker compose unavailable; falling back to docker run"',
        "    docker rm -f lidar-human-fall lidar-bird >/dev/null 2>&1 || true",
        "    docker run -d --name lidar-human-fall \\",
        "        --network host \\",
        '        --cpuset-cpus "${HUMAN_FALL_CPUSET}" \\',
        "        --memory 1536m \\",
        "        --restart unless-stopped \\",
        '        --env-file "${COMPOSE_DIR}/.env" \\',
        '        -v "${REL_DIR}:/opt/lidar-human-fall/bundle:ro" \\',
        "        lidar-human-fall:latest",
        "    docker run -d --name lidar-bird \\",
        "        --network host \\",
        '        --cpuset-cpus "${BIRD_CPUSET}" \\',
        "        --memory 512m \\",
        "        --restart unless-stopped \\",
        '        --env-file "${COMPOSE_DIR}/.env" \\',
        '        -v "${CAL_DIR}:/config/calibration:ro" \\',
        "        lidar-bird:latest",
        "fi",
        "",
        'echo "=== [7/7] Verify both containers ==="',
        'docker ps --filter "name=slam-localization" --filter "name=lidar-human-fall" --filter "name=lidar-bird" --format "table {{.Names}}\\t{{.Status}}\\t{{.Image}}"',
        'echo "--- wait for lidar-bird /healthz ---"',
        'BIRD_OK=0',
        'for _ in $(seq 1 15); do',
        '    if python3 - "${BIRD_HTTP_PORT}" <<\'PY\'',
        'import sys, urllib.request',
        'url = f"http://127.0.0.1:{sys.argv[1]}/healthz"',
        'print(urllib.request.urlopen(url, timeout=3).read().decode())',
        "PY",
        "    then",
        "        BIRD_OK=1",
        "        break",
        "    fi",
        "    sleep 1",
        "done",
        'if [[ "${BIRD_OK}" != "1" ]]; then',
        '    docker logs --tail 50 lidar-bird >&2 || true',
        '    echo "ERROR: lidar-bird /healthz failed" >&2',
        "    exit 1",
        "fi",
        'echo "--- wait for lidar-human-fall healthcheck ---"',
        'HF_OK=0',
        'for _ in $(seq 1 20); do',
        '    if docker exec lidar-human-fall bash -lc "source /opt/ros/noetic/setup.bash && python3 -B /opt/lidar-human-fall/healthcheck.py" >/dev/null 2>&1; then',
        "        HF_OK=1",
        '        echo "HUMAN_FALL_HEALTHY"',
        "        break",
        "    fi",
        "    sleep 1",
        "done",
        'if [[ "${HF_OK}" != "1" ]]; then',
        '    docker logs --tail 50 lidar-human-fall >&2 || true',
        '    echo "ERROR: lidar-human-fall healthcheck failed" >&2',
        "    exit 1",
        "fi",
        'echo "TWO_CONTAINER_DEPLOY_OK"',
        "",
    ]
    return "\n".join(lines)


def _add_bytes_to_tar(tf, arcname, raw_bytes, mode=0o644):
    info = tarfile.TarInfo(name=arcname)
    info.size = len(raw_bytes)
    info.mode = mode
    tf.addfile(info, io.BytesIO(raw_bytes))


def create_staging_bundle(out_dir, *, human_fall_tar, calibration_bin=None, config):
    """Create a self-contained checksummed staging archive for RK3588 deployment."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    archive_path = out_dir / "rk3588_two_containers_stage.tar.gz"
    manifest_path = out_dir / "STAGE_MANIFEST.json"
    if archive_path.exists():
        raise FileExistsError(f"Choose a fresh output directory; already exists: {archive_path}")

    bundle_info = inspect_human_fall_bundle(human_fall_tar)
    config = dict(config)
    config["release_id"] = bundle_info["release"]["release_id"]
    config["human_fall_tar_sha"] = bundle_info["tar_sha256"]
    config["has_calibration_bin"] = calibration_bin is not None

    remote_script = build_remote_deploy_script(config).encode("utf-8")

    staged_files = {}
    with tarfile.open(archive_path, "w:gz") as tf:
        for rel in STAGE_REPO_FILES:
            src = REPO_ROOT / rel
            if not src.is_file():
                raise FileNotFoundError(f"Required repository file missing: {src}")
            raw = src.read_bytes()
            if rel.endswith(".sh"):
                raw = raw.replace(b"\r\n", b"\n")
            mode = 0o755 if rel.endswith(".sh") else 0o644
            arc = f"repo/{rel}"
            _add_bytes_to_tar(tf, arc, raw, mode=mode)
            staged_files[arc] = {"bytes": len(raw), "sha256": sha256_bytes(raw)}

        hf_raw = Path(human_fall_tar).read_bytes()
        _add_bytes_to_tar(tf, "human_fall_bundle.tar", hf_raw, mode=0o644)
        staged_files["human_fall_bundle.tar"] = {
            "bytes": len(hf_raw),
            "sha256": bundle_info["tar_sha256"],
        }

        _add_bytes_to_tar(
            tf, "calibration/ifw192s_azimuth_192x256.csv", bundle_info["azimuth_csv_bytes"]
        )
        _add_bytes_to_tar(
            tf, "calibration/ifw192s_elevation_192x256.csv", bundle_info["elevation_csv_bytes"]
        )
        staged_files["calibration/ifw192s_azimuth_192x256.csv"] = {
            "bytes": len(bundle_info["azimuth_csv_bytes"]),
            "sha256": sha256_bytes(bundle_info["azimuth_csv_bytes"]),
        }
        staged_files["calibration/ifw192s_elevation_192x256.csv"] = {
            "bytes": len(bundle_info["elevation_csv_bytes"]),
            "sha256": sha256_bytes(bundle_info["elevation_csv_bytes"]),
        }

        if calibration_bin is not None:
            cal_raw = Path(calibration_bin).read_bytes()
            _add_bytes_to_tar(tf, "calibration/calibration_sdk_geometry.bin", cal_raw)
            staged_files["calibration/calibration_sdk_geometry.bin"] = {
                "bytes": len(cal_raw),
                "sha256": sha256_bytes(cal_raw),
            }

        _add_bytes_to_tar(tf, "remote_deploy.sh", remote_script, mode=0o755)
        staged_files["remote_deploy.sh"] = {
            "bytes": len(remote_script),
            "sha256": sha256_bytes(remote_script),
        }

    archive_sha = sha256_file(archive_path)
    summary = {
        "archive": str(archive_path.resolve()),
        "archive_sha256": archive_sha,
        "archive_bytes": archive_path.stat().st_size,
        "human_fall_release": bundle_info["release"],
        "human_fall_manifest_files": bundle_info["manifest_count"],
        "deploy_config": config,
        "staged_files": staged_files,
    }
    manifest_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return archive_path, manifest_path, summary


def run_ssh(host, user, remote_cmd, timeout=60):
    return subprocess.run(
        ["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", f"{user}@{host}", remote_cmd],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def deploy_remote(host, user, archive_path, archive_sha):
    probe = run_ssh(host, user, "hostname && docker ps --format '{{.Names}}\\t{{.Status}}'", timeout=15)
    if probe.returncode != 0:
        raise RuntimeError(f"Board {user}@{host} unreachable: {(probe.stderr or probe.stdout).strip()}")
    print("Board reachable:\n" + probe.stdout.strip(), flush=True)

    remote_tar = f"/tmp/rk3588_two_containers_{archive_sha[:12]}.tar.gz"
    remote_dir = f"/tmp/rk3588_two_containers_{archive_sha[:12]}"
    subprocess.run(
        ["scp", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", str(archive_path), f"{user}@{host}:{remote_tar}"],
        check=True,
    )
    remote_cmd = (
        f"set -euo pipefail; "
        f'echo "{archive_sha}  {remote_tar}" | sha256sum -c -; '
        f'rm -rf "{remote_dir}" && mkdir -p "{remote_dir}"; '
        f'tar -xzf "{remote_tar}" -C "{remote_dir}"; '
        f'rm -f "{remote_tar}"; '
        f'bash "{remote_dir}/remote_deploy.sh"; '
        f'rm -rf "{remote_dir}"'
    )
    res = run_ssh(host, user, remote_cmd, timeout=300)
    print(res.stdout, flush=True)
    if res.returncode != 0:
        print(res.stderr, file=sys.stderr, flush=True)
        raise RuntimeError(f"Remote deployment failed with exit code {res.returncode}")


def rollback_remote(host, user):
    cmd = (
        "set -euo pipefail; "
        "docker rm -f lidar-human-fall lidar-bird >/dev/null 2>&1 || true; "
        "docker ps --format 'table {{.Names}}\\t{{.Status}}'; "
        "echo ROLLBACK_CONTAINERS_STOPPED"
    )
    res = run_ssh(host, user, cmd, timeout=30)
    print(res.stdout, flush=True)
    if res.returncode != 0:
        print(res.stderr, file=sys.stderr, flush=True)
        raise RuntimeError(f"Rollback failed with exit code {res.returncode}")


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.31.58")
    parser.add_argument("--user", default="wel")
    parser.add_argument("--remote-root", default=DEFAULT_REMOTE_ROOT)
    parser.add_argument("--human-fall-bundle", type=Path, default=DEFAULT_HUMAN_FALL_TAR)
    parser.add_argument("--calibration-bin", type=Path, default=None,
                        help="Optional device-specific calibration_sdk_geometry.bin override for lidar-bird")
    parser.add_argument("--bird-http-port", type=int, default=8895)
    parser.add_argument("--human-fall-cpuset", default="4,5")
    parser.add_argument("--bird-cpuset", default="6,7")
    parser.add_argument("--human-fall-mode", choices=("instant_start", "empty_prefix"), default="instant_start")
    parser.add_argument("--mount-pitch-deg", type=float, default=26.0)
    parser.add_argument("--mount-roll-deg", type=float, default=0.0)
    parser.add_argument("--enable-imu", action="store_true")
    parser.add_argument("--with-human-physical-v1", action="store_true")
    parser.add_argument("--single-person-lock", action="store_true")
    parser.add_argument("--human-fall-base-image", default="")
    parser.add_argument("--bird-base-image", default="")
    parser.add_argument("--stage-only", action="store_true",
                        help="Build and verify the staging archive locally without SSHing to RK3588")
    parser.add_argument("--out", type=Path, default=None,
                        help="Output directory for --stage-only archive (required with --stage-only)")
    parser.add_argument("--rollback", action="store_true",
                        help="Stop and remove lidar-human-fall and lidar-bird containers without touching slam-localization")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.rollback:
        rollback_remote(args.host, args.user)
        return 0

    config = {
        "remote_root": args.remote_root,
        "bird_http_port": args.bird_http_port,
        "human_fall_cpuset": args.human_fall_cpuset,
        "bird_cpuset": args.bird_cpuset,
        "human_fall_mode": args.human_fall_mode,
        "mount_pitch_deg": args.mount_pitch_deg,
        "mount_roll_deg": args.mount_roll_deg,
        "disable_imu": not args.enable_imu,
        "with_human_physical_v1": args.with_human_physical_v1,
        "single_person_lock": args.single_person_lock,
        "human_fall_base_image": args.human_fall_base_image,
        "bird_base_image": args.bird_base_image,
    }

    if args.stage_only:
        if args.out is None:
            raise SystemExit("--out is required when using --stage-only")
        archive_path, manifest_path, summary = create_staging_bundle(
            args.out,
            human_fall_tar=args.human_fall_bundle,
            calibration_bin=args.calibration_bin,
            config=config,
        )
        print(json.dumps({
            "ok": True,
            "mode": "stage_only",
            "archive": str(archive_path),
            "archive_sha256": summary["archive_sha256"],
            "archive_bytes": summary["archive_bytes"],
            "manifest": str(manifest_path),
            "human_fall_release_id": summary["human_fall_release"]["release_id"],
        }, ensure_ascii=False, indent=2))
        return 0

    with tempfile.TemporaryDirectory(prefix="rk3588_stage_") as tmp:
        archive_path, _manifest, summary = create_staging_bundle(
            Path(tmp),
            human_fall_tar=args.human_fall_bundle,
            calibration_bin=args.calibration_bin,
            config=config,
        )
        deploy_remote(args.host, args.user, archive_path, summary["archive_sha256"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
