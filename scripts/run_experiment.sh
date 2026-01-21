#!/bin/bash
# Run experiment sequence: move_to_pose -> switch controller -> record video

CASE=${1:-default}
VIDEO_FILENAME=${2:-realsense_color.mp4}
FPS=${3:-30}

echo "=========================================="
echo "Starting experiment with case: $CASE"
echo "Video filename: $VIDEO_FILENAME"
echo "FPS: $FPS"
echo "=========================================="

# Step 1: Move to initial pose
echo "[Step 1] Moving robot to initial pose..."
rosrun linearmpc_panda move_to_pose _case:=$CASE

# Wait for the trajectory to complete
echo "[Step 1] Waiting for trajectory to complete..."
sleep 5 # sleep for 5 seconds to ensure the robot reaches the pose, move_to_pose takes 4s

# Step 2: Switch controller
echo "[Step 2] Switching controller..."
rosservice call /controller_manager/switch_controller "{start_controllers: ['panda_torque_pd_controller_simpson'], stop_controllers: ['position_joint_trajectory_controller'], strictness: 2}"

if [ $? -ne 0 ]; then # check if the last command ($?) is successful (-ne 0, not equal to 0)
    echo "[ERROR] Failed to switch controller"
    exit 1
fi

echo "[Step 2] Controller switched successfully"

# Step 3: Start video recording in background
echo "[Step 3] Starting video recording..."
rosrun image_view video_recorder image:=/camera/color/image_raw _filename:=$VIDEO_FILENAME _fps:=$FPS &
VIDEO_PID=$!

echo "[Step 3] Video recording started (PID: $VIDEO_PID)"

# Wait for trajectory completion signal
echo "[Step 4] Waiting for trajectory completion..."
while true; do
    result=$(rostopic echo -n 1 /trajectory_completion/data)
    if [ "$result" == "True" ]; then
        echo "[step 4] Trajectory completed!"
        break
    fi
    sleep 0.1
done

# Stop video recording
echo "[Step 5] Stopping video recording..."
kill $VIDEO_PID 2>/dev/null

# Switch back to position controller
echo "[Step 6] Switching back to position controller..."
rosservice call /controller_manager/switch_controller "{start_controllers: ['position_joint_trajectory_controller'], stop_controllers: ['panda_torque_pd_controller_simpson'], strictness: 2}"

echo "=========================================="
echo "Experiment complete!"
echo "=========================================="

# usage:
# # chmod +x run_experiment.sh
# # Run with default case
# roslaunch linearmpc_panda run_experiment.launch

# # Run with specific case
# roslaunch linearmpc_panda run_experiment.launch case:=extra_case1

# # Run with custom video settings
# roslaunch linearmpc_panda run_experiment.launch case:=extra_case1 video_filename:=exp1.mp4 fps:=60
 

