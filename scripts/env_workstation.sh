#!/usr/bin/env bash
#
# Mark Five AMR - Workstation ROS Environment Setup
#
# This machine runs as ROS Master and handles heavy processing:
#   - roscore
#   - RViz visualization
#   - RTAB-Map SLAM (future)
#   - move_base navigation (future)
#   - YOLO object detection (future)
#   - MoveIt! arm planning (future)
#
# Usage:
#   source scripts/env_workstation.sh
#
# After sourcing, start roscore:
#   roscore
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
export ROS_IP=${WORKSTATION_IP}
export ROS_HOSTNAME=${WORKSTATION_IP}

# Verify network settings
echo "=============================================="
echo "  Mark Five AMR - Workstation Configuration"
echo "=============================================="
echo "  ROS_MASTER_URI: $ROS_MASTER_URI"
echo "  ROS_IP:         $ROS_IP"
echo "  Role:           ROS Master + Heavy Processing"
echo "=============================================="
echo ""
echo "Next steps:"
echo "  1. Start roscore:  roscore"
echo "  2. Launch nodes:   roslaunch mark_five_bot workstation.launch"
echo ""
