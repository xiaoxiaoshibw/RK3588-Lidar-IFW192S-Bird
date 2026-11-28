"""Docker HEALTHCHECK for the lidar-human-fall container."""
import os
from pathlib import Path
import sys


def _has_node_process():
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            cmdline = (entry / "cmdline").read_bytes().decode("utf-8", errors="ignore")
        except OSError:
            continue
        if "range_service_node.py" in cmdline:
            return True
    return False


def main():
    if not _has_node_process():
        print("range_service_node.py not running", file=sys.stderr)
        return 1
    state_topic = os.environ.get("HUMAN_FALL_STATE_TOPIC", "/single_target_live/state")
    import rosgraph

    publishers = rosgraph.Master("/human_fall_healthcheck").getSystemState()[0]
    active = [names for topic, names in publishers if topic == state_topic and names]
    if not active:
        print(f"no active publisher for {state_topic}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
