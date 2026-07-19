#!/bin/bash
# start_full_stack.sh
# 
# Usage:
#   bash start_full_stack.sh
#   tmux attach -t go2_stack (to re-attach)

SESSION="go2_stack"

# Check if session exists; if so, attach
tmux has-session -t $SESSION 2>/dev/null

if [ $? != 0 ]; then

  echo "Ensuring containers are running..."
  docker start go2nav
  docker start go2vision
  sleep 2

  # --- Window 0: nav ---
  tmux new-session -d -s $SESSION -n "nav"
  tmux send-keys -t $SESSION:0 "1" C-m
  sleep 1
  tmux send-keys -t $SESSION:0 \
    "docker exec -it -w /workspace go2nav bash -c 'source /nav_build/install/setup.bash && ros2 launch go2_nav explore_slam.launch.py; bash'" C-m

  # --- Window 1: camera ---
  tmux new-window -t $SESSION -n "camera"
  tmux send-keys -t $SESSION:1 "1" C-m
  sleep 1
  tmux send-keys -t $SESSION:1 \
    "docker exec -it -w /workspace go2nav bash -c 'sleep 5 && source /nav_build/install/setup.bash && RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run go2_nav cameraimg; bash'" C-m

  # --- Window 2: yolo ---
  tmux new-window -t $SESSION -n "yolo"
  tmux send-keys -t $SESSION:2 "1" C-m
  sleep 1
  tmux send-keys -t $SESSION:2 \
    "docker exec -it -w /workspace go2vision bash -c 'sleep 10 && ros2 run go2_vision yolo_detector; bash'" C-m

  # --- Window 3: pursuit ---
  tmux new-window -t $SESSION -n "pursuit"
  tmux send-keys -t $SESSION:3 "1" C-m
  sleep 1
  tmux send-keys -t $SESSION:3 \
    "docker exec -it -w /workspace go2nav bash -c 'sleep 35 && source /nav_build/install/setup.bash && ros2 launch go2_nav object_pursuit.launch.py; bash'" C-m

  # --- Window 4: scratch ---
  tmux new-window -t $SESSION -n "scratch"
  tmux send-keys -t $SESSION:4 "1" C-m
  sleep 1
  # Drops you directly into an interactive bash shell in /workspace
  tmux send-keys -t $SESSION:4 "docker exec -it -w /workspace go2nav bash" C-m
fi

# Attach to the session
tmux attach-session -t $SESSION