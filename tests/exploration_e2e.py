#!/usr/bin/env python3
"""
End-to-end automated test for warehouse exploration system.

Launches the full demo, verifies node health, checks /cmd_vel publisher
uniqueness, and runs for a fixed monitoring duration.

Usage:
  source /opt/ros/noetic/setup.bash
  python3 tests/exploration_e2e.py [--duration 60] [--headless]

Exit codes: 0=pass, 1=fail
"""
import argparse
import os
import signal
import subprocess
import sys
import time


# ── Expected nodes ──────────────────────────────────────────
EXPECTED_NODES = {
    "gazebo", "spawn_robot", "hector_mapping",
    "mission_node", "motion_controller",
    "move_base", "navigation_manager",
    "exploration_node", "coverage_monitor",
    "recovery", "rviz",
}

# Required topics that must exist
REQUIRED_TOPICS = [
    "/cmd_vel",
    "/map",
    "/scan",
    "/odom",
    "/mission_state",
    "/exploration_state",
    "/coverage",
]

# /cmd_vel must have exactly one publisher
CMD_VEL_PUBLISHER = "motion_controller"


def run(cmd, timeout=30):
    """Run a shell command, return stdout and exit code."""
    try:
        p = subprocess.run(cmd, shell=True, capture_output=True,
                           text=True, timeout=timeout)
        return p.stdout.strip(), p.returncode
    except subprocess.TimeoutExpired:
        return "", -1


def source_ros():
    """Set up ROS environment."""
    return ("source /opt/ros/noetic/setup.bash && "
            "source ~/catkin_ws/devel/setup.bash && ")


def kill_ros():
    """Kill all ROS/Gazebo processes."""
    run("killall -9 gzserver gzclient roscore rosmaster 2>/dev/null")
    time.sleep(2)


def count_nodes():
    """Count alive ROS nodes."""
    out, _ = run(source_ros() + "rosnode list 2>/dev/null", timeout=20)
    if not out:
        return set()
    return set(out.strip().split("\n"))


def get_cmd_vel_publishers():
    """Get /cmd_vel topic info."""
    out, _ = run(source_ros() + "rostopic info /cmd_vel 2>/dev/null", timeout=20)
    if not out:
        return []
    pubs = []
    for line in out.split("\n"):
        if "Publishers:" in line:
            continue
        line = line.strip()
        if line.startswith("* "):
            node = line[2:].split("/", 1)[1] if "/" in line[2:] else line[2:]
            pubs.append(node.split("(")[0].strip())
    return pubs


def topic_exists(topic):
    """Check if topic has publishers."""
    out, _ = run(source_ros() + f"rostopic info {topic} 2>/dev/null", timeout=20)
    return "Publishers: None" not in out if out else False


def main():
    parser = argparse.ArgumentParser(description="E2E exploration test")
    parser.add_argument("--duration", type=int, default=90,
                        help="Monitoring duration in seconds (default: 90)")
    parser.add_argument("--headless", action="store_true",
                        help="Don't show RViz")
    parser.add_argument("--no-launch", action="store_true",
                        help="Skip launch (assume already running)")
    args = parser.parse_args()

    os.chdir(os.path.expanduser("~/catkin_ws"))
    passed = True

    # ── Phase 1: Launch ─────────────────────────────────
    if not args.no_launch:
        print("[E2E] Phase 1: Launching demo...")
        kill_ros()

        # Start roscore
        subprocess.Popen(
            ["bash", "-c", source_ros() + "roscore"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(3)

        # Start demo in background
        rviz_arg = "--rviz_required:=false" if args.headless else ""
        subprocess.Popen(
            ["bash", "-c",
             source_ros() +
             f"roslaunch warehouse_utils demo_gazebo.launch gui:=true {rviz_arg}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # Wait for Gazebo and nodes to initialize
        print("[E2E] Waiting 40s for Gazebo + nodes to initialize...")
        time.sleep(40)

    # ── Phase 2: Node health check ──────────────────────
    print("[E2E] Phase 2: Node health check...")
    nodes = count_nodes()
    missing = EXPECTED_NODES - nodes
    extra = nodes - EXPECTED_NODES

    print(f"  Found {len(nodes)} nodes")

    if missing:
        print(f"  WARNING: Missing nodes: {missing}")
        # Non-critical: rviz may fail in headless mode
        if missing != {"rviz"}:
            passed = False

    # ── Phase 3: Topic verification ─────────────────────
    print("[E2E] Phase 3: Topic verification...")
    for topic in REQUIRED_TOPICS:
        exists = topic_exists(topic)
        status = "OK" if exists else "MISSING"
        print(f"  {status}: {topic}")
        if not exists:
            passed = False

    # Check /cmd_vel publisher uniqueness (ADR-001)
    pubs = get_cmd_vel_publishers()
    print(f"  /cmd_vel publishers: {pubs}")
    if len(pubs) != 1:
        print(f"  FAIL: /cmd_vel has {len(pubs)} publishers, expected 1")
        passed = False
    elif CMD_VEL_PUBLISHER not in pubs[0]:
        print(f"  FAIL: /cmd_vel publisher is '{pubs[0]}', expected '{CMD_VEL_PUBLISHER}'")
        passed = False
    else:
        print(f"  OK: /cmd_vel sole publisher is {CMD_VEL_PUBLISHER}")

    # ── Phase 4: Stability monitoring ───────────────────
    print(f"[E2E] Phase 4: Monitoring for {args.duration}s...")
    start = time.time()
    last_count = len(nodes)
    while time.time() - start < args.duration:
        time.sleep(10)
        elapsed = int(time.time() - start)
        nodes = count_nodes()
        count = len(nodes)
        if count != last_count:
            delta = count - last_count
            direction = "+" if delta > 0 else ""
            print(f"  t={elapsed}s: node count {last_count} → {count} ({direction}{delta})")
            last_count = count
        else:
            print(f"  t={elapsed}s: {count} nodes (stable)")

        # Periodic cmd_vel check
        pubs = get_cmd_vel_publishers()
        if CMD_VEL_PUBLISHER not in (pubs[0] if pubs else ""):
            print(f"  WARNING: /cmd_vel publisher changed: {pubs}")

    # ── Phase 5: Final summary ──────────────────────────
    print(f"\n[E2E] Final: {len(nodes)} nodes alive")
    for node in sorted(nodes):
        print(f"  - {node}")

    if passed:
        print("\n[E2E] RESULT: PASS")
    else:
        print("\n[E2E] RESULT: FAIL")

    if not args.no_launch:
        kill_ros()

    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
