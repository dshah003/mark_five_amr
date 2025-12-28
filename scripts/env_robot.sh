#!/usr/bin/env bash
#
# Mark Five AMR - Robot (Jetson Nano) ROS Environment Setup
#
# This machine runs robot-side nodes and connects to workstation master:
#   - rosserial (Arduino communication)
#   - odometry_node
#   - robot_state_publisher
#   - RealSense camera driver
#   - depthimage_to_laserscan
#
# Usage (inside Docker container):
#   source ~/mark_five_amr/scripts/env_robot.sh
#
# Note: Do NOT run roscore on this machine - workstation is the master.
#

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Load network configuration
if [ -f "$SCRIPT_DIR/network.conf" ]; then
    source "$SCRIPT_DIR/network.conf"
else
    echo "ERROR: network.conf not found!"
    echo "Please copy network.conf.template to network.conf and configure IPs:"
    echo "  cp $SCRIPT_DIR/network.conf.template $SCRIPT_DIR/network.conf"
    return 1
fi

# Network Configuration
export ROS_MASTER_URI=http://${WORKSTATION_IP}:11311
export ROS_IP=${JETSON_IP}
export ROS_HOSTNAME=${JETSON_IP}

# Verify network settings
echo "=============================================="
echo "  Mark Five AMR - Robot Configuration"
echo "=============================================="
echo "  ROS_MASTER_URI: $ROS_MASTER_URI (Workstation)"
echo "  ROS_IP:         $ROS_IP"
echo "  Role:           Robot (Sensing + Actuation)"
echo "=============================================="
echo ""
echo "Next steps:"
echo "  1. Ensure workstation roscore is running"
echo "  2. Test connection:  rostopic list"
echo "  3. Launch robot:     roslaunch mark_five_bot robot.launch"
echo ""
