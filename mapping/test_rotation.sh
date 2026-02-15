#!/bin/bash
# Test script to verify rotation parameter is loaded

cd /home/lorenzo/autonomy-ws-25-26/mapping
source install/setup.bash

echo "======================================"
echo "Testing lidar_range_filter rotation"
echo "======================================"

# Start node in background
ros2 run pointcloud_filters lidar_range_filter --ros-args -p rotate_cw_deg:=22.5 &
NODE_PID=$!

echo "Started node with PID: $NODE_PID"
sleep 3

echo ""
echo "Checking parameters:"
ros2 param list /lidar_range_filter

echo ""
echo "Getting rotate_cw_deg value:"
ros2 param get /lidar_range_filter rotate_cw_deg

echo ""
echo "Killing test node..."
kill $NODE_PID 2>/dev/null
wait $NODE_PID 2>/dev/null

echo ""
echo "Test complete!"
