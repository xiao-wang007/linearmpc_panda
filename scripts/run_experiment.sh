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

# Step 2: Prepare for experiment
echo ""
read -p "[Step 2] Press ENTER to start experiment..."

# Capture initial object pose BEFORE starting
POSE_INIT_FILE="${VIDEO_FILENAME%.mp4}_pose_init.csv"
echo "[Step 2.1] Capturing initial object pose..."
rostopic echo -p -n 1 /mocap/rigid_bodies/Tomato_soup/pose > $POSE_INIT_FILE
echo "[Step 2.1] Initial pose saved to $POSE_INIT_FILE"

# Start video recording BEFORE switching controller
echo "[Step 2.2] Starting video recording..."
# & means to run the line in the background, > /dev/null throws output away, 2>&1 throws errors away
rosrun image_view video_recorder image:=/camera/color/image_raw _filename:=$VIDEO_FILENAME _fps:=$FPS > /dev/null 2>&1 &
VIDEO_PID=$!
echo "[Step 2.2] Video recording started (PID: $VIDEO_PID)"
sleep 1  # Give video recorder time to initialize

# NOW switch controller (trajectory starts)
echo "[Step 2.3] Switching controller..."
rosservice call /controller_manager/switch_controller "{start_controllers: ['panda_torque_pd_controller_simpson'], stop_controllers: ['position_joint_trajectory_controller'], strictness: 2}"

if [ $? -ne 0 ]; then # check if the last command ($?) is successful (-ne 0, not equal to 0)
    echo "[ERROR] Failed to switch controller"
    kill -SIGINT $VIDEO_PID 2>/dev/null
    exit 1
fi

echo "[Step 2.3] Controller switched - experiment running!"

# Wait for trajectory completion signal
echo "[Step 3] Waiting for trajectory completion..."
while true; do
    RESULT=$(rostopic echo -n 1 /trajectory_completion)
    if echo "$RESULT" | grep -q "data: True"; then
        break
    fi
    sleep 0.5
done
echo "[Step 3] Trajectory completed!"

# Capture final object pose
POSE_END_FILE="${VIDEO_FILENAME%.mp4}_pose_end.csv"
echo "[Step 3.1] Capturing final object pose..."
rostopic echo -p -n 1 /mocap/rigid_bodies/Tomato_soup/pose > $POSE_END_FILE
echo "[Step 3.1] Final pose saved to $POSE_END_FILE"

# Stop video recording
echo "[Step 3.2] Stopping video recording..."
kill -SIGINT $VIDEO_PID 2>/dev/null
sleep 2  # Give video_recorder time to finalize

# Switch back to position controller
echo "[Step 4] Switching back to position controller..."
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