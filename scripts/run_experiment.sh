#!/bin/bash
# Run experiment sequence: move_to_pose -> switch controller -> record video

CASE=${1:-default}
VIDEO_FILENAME=${2:-realsense_color.mp4}
FPS=${3:-30}
WIDTH=${4:-640}
HEIGHT=${5:-480}

echo "=========================================="
echo "Starting experiment with case: $CASE"
echo "Video filename: $VIDEO_FILENAME"
echo "FPS: $FPS"
# echo "Resolution: ${WIDTH}x${HEIGHT}"
echo "=========================================="

# Load pose parameters to parameter server
echo "[Step 0] Loading tracking case and q_init parameters..."
# rosparam load $(rospack find linearmpc_panda)/config/init_poses.yaml /move_to_pose (/move_to_pose namespace is private)
rosparam load $(rospack find linearmpc_panda)/config/init_poses.yaml

# no needed anymore but as a exmpale of using || true: or true, so if what's before || fails,
# the script continues without exiting
# echo "[Step 0.5] Loading controller..."
# rosservice call /controller_manager/load_controller "name: 'panda_torque_pd_controller_simpson'" || true

# Step 1: Move to initial pose
echo "[Step 1] Moving robot to initial pose..."

MAX_RETRIES=10
RETRY_COUNT=0

while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    rosrun linearmpc_panda move_to_pose _case:=$CASE # _case is a private parameter
    
    echo "[Step 1] Waiting for arm to reach pose..."
    sleep 5 # sleep for 5 seconds to ensure the robot reaches the pose
    
    # Ask user to confirm robot reached the pose
    echo ""
    read -p "[Step 1] Did the robot reach the initial pose? (y/n): " CONFIRM
    if [ "$CONFIRM" = "y" ] || [ "$CONFIRM" = "Y" ]; then
        echo "[Step 1] Initial pose confirmed!"
        break
    else
        RETRY_COUNT=$((RETRY_COUNT + 1))
        if [ $RETRY_COUNT -lt $MAX_RETRIES ]; then
            echo "[Step 1] Retrying move_to_pose... (attempt $((RETRY_COUNT + 1))/$MAX_RETRIES)"
        else
            echo "[ERROR] Failed to reach initial pose after $MAX_RETRIES attempts"
            exit 1
        fi
    fi
done

# Step 2: Switch controller
echo ""
read -p "[Step 2] Press ENTER to switch controller and start experiment..."
echo "[Step 2] Switching controller..."
rosservice call /controller_manager/switch_controller "{start_controllers: ['panda_torque_pd_controller_simpson'], stop_controllers: ['position_joint_trajectory_controller'], strictness: 2}"

if [ $? -ne 0 ]; then # check if the last command ($?) is successful (-ne 0, not equal to 0)
    echo "[ERROR] Failed to switch controller"
    exit 1
fi

echo "[Step 2] Controller switched successfully"

# Step 3: Start video recording in background
echo "[Step 3] Starting video recording..."
# Note: video_recorder records at source resolution. To resize, either:
#   - Use ffmpeg after: ffmpeg -i input.mp4 -vf scale=${WIDTH}:${HEIGHT} output.mp4
#   - Or launch a resize node before this line
rosrun image_view video_recorder image:=/camera/color/image_raw _filename:=$VIDEO_FILENAME _fps:=$FPS &
VIDEO_PID=$!

echo "[Step 3] Video recording started (PID: $VIDEO_PID)"

# Wait for trajectory completion signal
echo "[Step 4] Waiting for trajectory completion..."
while true; do
    RESULT=$(rostopic echo -n 1 /trajectory_completion)
    if echo "$RESULT" | grep -q "data: True"; then
        break
    fi
    sleep 0.5
done
echo "[Step 4] Trajectory completed!"

# Stop video recording
echo "[Step 5] Stopping video recording..."
kill -SIGINT $VIDEO_PID 2>/dev/null
sleep 2  # Give video_recorder time to finalize

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