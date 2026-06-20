#!/usr/bin/env python3
"""ROS2 MCP server — gives Claude direct access to robot state and control.

Run via: docker exec -i mark_five_robot python3 /root/mark_five_amr/tools/ros2_mcp/server.py
Registered in .claude/settings.local.json as the "ros2" MCP server.
"""
import shlex
import subprocess

from mcp.server.fastmcp import FastMCP

mcp = FastMCP("ros2")

# ── Config ────────────────────────────────────────────────────────────────────
JETSON_USER      = "jetson"
JETSON_HOST      = "jetson@jarvis.local"
JETSON_CONTAINER = "mark_five_robot"
SSH_KEY          = "/root/.ssh/jetson_key"

ROS_WS     = "/root/mark_five_amr"
ROS_BASE   = "/opt/ros/jazzy/setup.bash"
ROS_SOURCE = (
    f"source {ROS_BASE} && "
    f"if [ -f {ROS_WS}/install/setup.bash ]; then "
    f"  source {ROS_WS}/install/setup.bash; "
    f"fi"
)

_SSH_OPTS = [
    "-i", SSH_KEY,
    "-o", "StrictHostKeyChecking=no",
    "-o", "ConnectTimeout=10",
    "-o", "BatchMode=yes",
]

# ── Low-level helpers ─────────────────────────────────────────────────────────

def _run(cmd: str, timeout: int = 15) -> str:
    try:
        r = subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True, text=True, timeout=timeout,
        )
        out = r.stdout.strip()
        err = r.stderr.strip()
        if r.returncode != 0 and err:
            out = (out + f"\n[stderr] {err}").strip()
        return out or "(no output)"
    except subprocess.TimeoutExpired:
        return f"[timeout after {timeout}s — topic may not be publishing]"
    except Exception as e:
        return f"[error] {e}"


def _ros2(*args: str, timeout: int = 15) -> str:
    cmd = f"{ROS_SOURCE} && ros2 {shlex.join(list(args))}"
    return _run(cmd, timeout)


def _ssh(cmd: str, timeout: int = 30) -> str:
    try:
        r = subprocess.run(
            ["ssh", *_SSH_OPTS, JETSON_HOST, cmd],
            capture_output=True, text=True, timeout=timeout,
        )
        out = r.stdout.strip()
        err = r.stderr.strip()
        if r.returncode != 0 and err:
            out = (out + f"\n[stderr] {err}").strip()
        return out or "(no output)"
    except subprocess.TimeoutExpired:
        return f"[ssh timeout after {timeout}s]"
    except Exception as e:
        return f"[ssh error] {e}"


def _jetson_docker(cmd: str, timeout: int = 30) -> str:
    """Run a command inside the Jetson's Docker container with ROS2 sourced."""
    escaped = cmd.replace("'", r"'\''")
    full_cmd = f"docker exec {JETSON_CONTAINER} bash -c '{ROS_SOURCE} && {escaped}'"
    return _ssh(full_cmd, timeout)


# ── Tier 1: Diagnostics ───────────────────────────────────────────────────────

@mcp.tool()
def list_nodes() -> str:
    """List all active ROS2 nodes currently running on the robot."""
    return _ros2("node", "list")


@mcp.tool()
def list_topics() -> str:
    """List all ROS2 topics with their message types."""
    return _ros2("topic", "list", "-t")


@mcp.tool()
def echo_topic(topic: str, count: int = 3) -> str:
    """
    Read N messages from a ROS2 topic.

    Useful for inspecting: /odom, /scan, /tf, /cmd_vel, /imu/data,
    /left_ticks, /right_ticks, /map, /robot_description, etc.
    Use count=1 for a quick snapshot, higher values to check consistency.
    Warning: /scan produces large output — use count=1 for laser data.
    """
    if count == 1:
        return _ros2("topic", "echo", "--once", topic, timeout=20)
    return _ros2("topic", "echo", "--times", str(count), topic, timeout=20)


@mcp.tool()
def topic_hz(topic: str, duration: int = 5) -> str:
    """
    Measure the publish rate of a topic over N seconds.
    Returns average Hz, min/max period, standard deviation.
    """
    cmd = (
        f"{ROS_SOURCE} && "
        f"timeout {duration} ros2 topic hz {shlex.quote(topic)} 2>&1 || true"
    )
    return _run(cmd, timeout=duration + 5)


@mcp.tool()
def topic_info(topic: str) -> str:
    """Get type, publisher count, and subscriber count for a topic."""
    return _ros2("topic", "info", "-v", topic)


@mcp.tool()
def node_info(node: str) -> str:
    """Get publishers, subscribers, and services for a specific node."""
    return _ros2("node", "info", node)


@mcp.tool()
def list_services() -> str:
    """List all available ROS2 services with their types."""
    return _ros2("service", "list", "-t")


# ── Tier 2: Parameters & Services ─────────────────────────────────────────────

@mcp.tool()
def list_params(node: str) -> str:
    """List all parameters for a ROS2 node."""
    return _ros2("param", "list", node)


@mcp.tool()
def get_param(node: str, param: str) -> str:
    """Get the current value of a ROS2 parameter on a live node."""
    return _ros2("param", "get", node, param)


@mcp.tool()
def set_param(node: str, param: str, value: str) -> str:
    """
    Set a ROS2 parameter on a live node (takes effect immediately, not persisted).
    value examples: "0.5", "true", "[1.0, 2.0, 3.0]"
    To persist, also update the relevant YAML config file.
    """
    return _ros2("param", "set", node, param, value)


@mcp.tool()
def call_service(service: str, srv_type: str, args: str = "{}") -> str:
    """
    Call a ROS2 service.
    args: YAML string, e.g. '{}' for Trigger, '{data: true}' for SetBool.

    Common services on this robot:
      /mission/start   std_srvs/srv/Trigger   {}
      /mission/stop    std_srvs/srv/Trigger   {}
      /mission/pause   std_srvs/srv/Trigger   {}
      /mission/resume  std_srvs/srv/Trigger   {}
      /arm/home        std_srvs/srv/Trigger   {}
      /arm/relax       std_srvs/srv/Trigger   {}
    """
    return _ros2("service", "call", service, srv_type, args, timeout=15)


# ── Tier 3: Jetson Operations ─────────────────────────────────────────────────

@mcp.tool()
def jetson_cmd(cmd: str) -> str:
    """
    Run a shell command on the Jetson host (not inside Docker).
    Use for: checking Docker status, disk space, system logs, network info.
    Examples: "docker ps", "df -h", "free -h", "ip addr"
    """
    return _ssh(cmd)


@mcp.tool()
def jetson_docker_cmd(cmd: str) -> str:
    """
    Run a command inside the Jetson's ROS2 Docker container (with ROS2 sourced).
    Use for: ros2 commands from Jetson perspective, checking node state on robot side.
    Examples: "ros2 node list", "ls /root/mark_five_amr/src"
    """
    return _jetson_docker(cmd)


@mcp.tool()
def read_ros_log(lines: int = 100) -> str:
    """
    Read the most recent ROS2 log file from the Jetson container.
    Useful for spotting errors, warnings, and node startup issues.
    """
    cmd = (
        "LOG_DIR=$(ls -dt /root/.ros/log/*/  2>/dev/null | head -1); "
        "if [ -z \"$LOG_DIR\" ]; then echo 'No ROS2 logs found'; "
        "else find \"$LOG_DIR\" -name '*.log' | "
        f"  xargs tail -n {lines} 2>/dev/null; fi"
    )
    return _jetson_docker(cmd, timeout=15)


@mcp.tool()
def rebuild_on_jetson(packages: str = "") -> str:
    """
    Run colcon build on the Jetson (inside its Docker container).
    packages: space-separated list to build selectively, empty = build all.
    Build runs in background; use get_build_log() to check progress.
    """
    pkg_flag = f"--packages-select {packages}" if packages.strip() else ""
    cmd = (
        f"cd {ROS_WS} && "
        f"colcon build {pkg_flag} "
        f"> /tmp/colcon_build.log 2>&1"
    )
    # Run detached so SSH doesn't block waiting for build to finish
    ssh_cmd = (
        f"docker exec -d {JETSON_CONTAINER} bash -c "
        f"'source {ROS_BASE} && {cmd}' && echo 'Build started in background'"
    )
    return _ssh(ssh_cmd, timeout=15)


@mcp.tool()
def get_build_log(lines: int = 60) -> str:
    """Read output from the last rebuild_on_jetson() call."""
    return _jetson_docker(
        f"tail -n {lines} /tmp/colcon_build.log 2>/dev/null || echo 'No build log found'",
        timeout=10,
    )


@mcp.tool()
def restart_jetson_container() -> str:
    """
    Restart the ROS2 Docker container on the Jetson.
    Use after a rebuild or when nodes are in a bad state.
    Takes ~10-15 seconds.
    """
    cmd = (
        f"cd {ROS_WS}/docker && "
        f"docker stop {JETSON_CONTAINER} 2>/dev/null; sleep 2; "
        f"./start.sh && echo 'Container restarted successfully'"
    )
    return _ssh(cmd, timeout=60)


if __name__ == "__main__":
    mcp.run()
