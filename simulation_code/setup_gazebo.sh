#!/usr/bin/env bash
# setup_gazebo.sh
# ================
# Run ONCE before using Gazebo replay.
# From simulation_code/:
#   bash setup_gazebo.sh
#
# What it does:
#   1. Converts the MuJoCo model to URDF/XACRO (model/so101.urdf[.xacro])
#   2. Copies STL meshes into the ROS2 package for Gazebo to find them
#   3. Copies the URDF into the ROS2 package urdf/ directory

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_PKG="$SCRIPT_DIR/../ros2_ws/src/gazebo_mujoco_bridge"

echo "=== Step 1: Generate URDF/XACRO from MuJoCo model ==="
cd "$SCRIPT_DIR"
python3 mjcf_to_urdf.py

echo ""
echo "=== Step 2: Copy URDF into ROS2 package ==="
cp "$SCRIPT_DIR/model/so101.urdf"       "$ROS2_PKG/urdf/so101.urdf"
cp "$SCRIPT_DIR/model/so101.urdf.xacro" "$ROS2_PKG/urdf/so101.urdf.xacro"
echo "  Copied so101.urdf and so101.urdf.xacro → $ROS2_PKG/urdf/"

echo ""
echo "=== Step 3: Copy STL mesh assets ==="
cp "$SCRIPT_DIR/model/assets/"*.stl "$ROS2_PKG/meshes/" 2>/dev/null || true
STL_COUNT=$(ls "$ROS2_PKG/meshes/"*.stl 2>/dev/null | wc -l)
echo "  Copied $STL_COUNT STL files → $ROS2_PKG/meshes/"

echo ""
echo "=== Done. Now build the ROS2 workspace: ==="
echo "  cd ../ros2_ws && colcon build --symlink-install"
echo "  source install/setup.bash"
echo ""
echo "Then export the trajectory:"
echo "  python3 export_trajectory.py"
echo ""
echo "Then replay in Gazebo:"
echo "  ros2 launch gazebo_mujoco_bridge so101_replay.launch.py \\"
echo "      csv_in:=\$(pwd)/trajectory_data/mujoco_trajectory.csv"
