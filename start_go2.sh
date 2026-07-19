#!/bin/bash
# start_full_stack.sh
#
# Launches the full go2nav/go2vision stack across a single tmux session,
# with staggered startup timing so dependent nodes don't start before
# what they need is actually up.
#
# Usage:
#   bash start_full_stack.sh      # create (or attach to) the session
#   tmux attach -t go2_stack       # reattach later from any terminal
#
# Window layout:
#   0: nav      - SLAM + Nav2 + explore_lite (starts immediately, takes
#                 longest to fully come up -- everything else waits on it)
#   1: camera   - cameraimg (go2_vision package, go2nav container, fastrtps)
#   2: yolo     - yolo_detector (go2vision container)
#   3: pursuit  - camera_info + static TFs + object_pursuit_node (go2nav) --
#                 starts LAST since it depends on nav's TF/map frame AND
#                 yolo's detections both being up already
#   4: scratch  - free terminal in go2nav for ad-hoc topic/tf checks

SESSION="go2_stack"

tmux has-session -t $SESSION 2>/dev/null
if [ $? != 0 ]; then

  # Make sure both containers are actually running FIRST -- docker exec
  # fails immediately (and silently, from tmux's perspective) if the
  # target container is stopped. docker start on an already-running
  # container is a harmless no-op, safe to always run.
  echo "Ensuring containers are running..."
  docker start go2nav
  docker start go2vision
  sleep 2

  # --- Window 0: nav (start immediately, no delay) ---
  tmux new-session -d -s $SESSION -n "nav"
  # The host's login shell sometimes prompts "ros:foxy(1) noetic(2) ?" on a
  # fresh shell. Auto-answer "1" (foxy) so the script doesn't stall waiting
  # for manual input. Harmless no-op if the prompt doesn't appear.
  tmux send-keys -t $SESSION:0 "1" C-m
  sleep 1
  tmux send-keys -t $SESSION:0 \
    "docker exec -it go2nav bash -c 'source /nav_build/install/setup.bash && ros2 launch go2_nav explore_slam.launch.py; bash'" C-m

  # --- Window 1: camera (small delay, just to avoid all containers
  #     hammering CPU at the exact same instant during startup) ---
  tmux new-window -t $SESSION -n "camera"
  tmux send-keys -t $SESSION:1 "1" C-m
  sleep 1
  tmux send-keys -t $SESSION:1 \
    "docker exec -it go2nav bash -c 'sleep 5 && source /nav_build/install/setup.bash && RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run go2_nav cameraimg; bash'" C-m

  # --- Window 2: yolo (waits for camera to actually be publishing first) ---
  tmux new-window -t $SESSION -n "yolo"
  tmux send-keys -t $SESSION:2 "1" C-m
  sleep 1
  tmux send-keys -t $SESSION:2 \
    "docker exec -it go2vision bash -c 'sleep 10 && ros2 run go2_vision yolo_detector; bash'" C-m

  # --- Window 3: pursuit (starts LAST -- needs nav's TF/map frame AND
  #     yolo's detections both already flowing; explore_slam.launch.py
  #     itself has ~25-30s of internal startup delays before SLAM/Nav2
  #     are fully up, so this waits generously past that) ---
  tmux new-window -t $SESSION -n "pursuit"
  tmux send-keys -t $SESSION:3 "1" C-m
  sleep 1
  tmux send-keys -t $SESSION:3 \
    "docker exec -it go2nav bash -c 'sleep 35 && source /nav_build/install/setup.bash && ros2 launch go2_nav object_pursuit.launch.py; bash'" C-m

  # --- Window 4: scratch terminal for ad-hoc checks (topic hz, tf2_echo, etc.) ---
  tmux new-window -t $SESSION -n "scratch"
  tmux send-keys -t $SESSION:4 "1" C-m
  sleep 1
  tmux send-keys -t $SESSION:4 \
    "docker exec -it go2nav bash -c 'source /nav_build/install/setup.bash; bash'" C-m

fi

tmux attach-session -t $SESSION