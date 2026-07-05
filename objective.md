# Go2 Vision-Guided Autonomous Exploration — Architecture Reference

## 1. Goal
Extend an existing SLAM + `explore_lite` frontier-exploration stack with a vision layer that detects objects (person, football, stairs, day-to-day objects) while mapping, and responds per-class: log it, mark it as a hazard, or navigate to it — without disrupting the ongoing map or exploration state.

---

## 2. High-Level Data Flow

```
Camera (RGB)              Depth/Pointcloud            SLAM (existing)
     │                          │                            │
     ▼                          │                            │
[Vision Node]                   │                            │
  YOLO11n (TensorRT, INT8)      │                            │
     │ 2D detections            │                            │
     ▼                          ▼                            │
[2D→3D Lifting Node] ◄──────────┘                             │
     │ object pose (camera frame)                             │
     ▼                                                        │
[TF Transform] ◄───────────────────────────────────────────────┘
     │ object pose (map frame)
     ▼
[Semantic Memory Node]
     │ dedupe / track / confidence-filter
     ▼
[Per-Class Policy Router]  ◄── policy.yaml (config, not code)
     │
     ├── log_only        → semantic memory only, done
     ├── costmap_hazard   → inject cost/keepout layer, done
     └── navigate_to      → notify Supervisor
                                  │
                                  ▼
                          [Mission Supervisor]
                     (state machine: EXPLORING / PAUSING /
                      NAVIGATING / DWELLING / RESUMING)
                                  │
                    publish False/True ──────► explore/resume (explore_lite)
                    send goal ────────────────► Nav2 navigate_to_pose action server
```

---

## 3. Components

| # | Component | Purpose | Tech / Package |
|---|-----------|---------|-----------------|
| 1 | **SLAM** | Builds/maintains the map continuously. Untouched by everything below — this was already running and stays fully decoupled from navigation goal churn. | Your existing SLAM stack (unchanged) |
| 2 | **Frontier Exploration** | Autonomously picks unexplored frontiers and drives there via Nav2, when active. | `explore_lite` (m-explore-ros2 port) |
| 3 | **Nav2 stack** | Path planning, control, execution of any `navigate_to_pose` goal — whether from `explore_lite` or the Supervisor. | Your existing `go2_nav` (BT navigator, planner server, controller server) |
| 4 | **Vision Node** | Runs real-time object detection on the camera feed at ≥30 FPS. | YOLO11n → ONNX → TensorRT (INT8), on Jetson Orin NX |
| 5 | **2D→3D Lifting** | Converts a 2D bbox + depth into a 3D point in camera frame. | Depth camera / stereo + simple projection math |
| 6 | **TF Transform** | Converts camera-frame object pose into map-frame pose using the current robot pose from SLAM. | ROS 2 `tf2` |
| 7 | **Semantic Memory** | Persistent store of detected objects: class, map pose, confidence, first/last seen, visited flag. Dedupes repeat detections of the same physical object. | Lightweight node (dict/SQLite backing) |
| 8 | **Per-Class Policy Router** | Reads `policy.yaml`, decides which behavior path a detected class takes. Keeps behavior *configurable*, not hardcoded. | Config-driven logic inside/alongside Semantic Memory |
| 9 | **Costmap Hazard Injection** | For hazard classes (e.g. stairs): marks the object's map location as high-cost/keepout so Nav2's planner (and `explore_lite`'s frontier choices) naturally avoid it — no goal preemption needed. | Nav2 costmap layer (e.g. custom keepout layer) |
| 10 | **Mission Supervisor** | The only node that owns "go-to" arbitration. Pauses exploration, sends one-off goal, dwells, resumes exploration. State machine, not hardcoded per-class logic. | Custom ROS 2 node |

---

## 4. Mission Supervisor — State Machine

```
EXPLORING  (default; explore_lite active, driving frontier goals)
   │
   │  "navigate_to" class detected, confidence ≥ threshold, not yet visited
   ▼
PAUSING     → publish False → explore/resume   (explore_lite halts, robot stops)
   ▼
NAVIGATING  → send goal → Nav2 navigate_to_pose action server (target pose, with approach offset)
   ▼
   ├─ success → DWELLING → wait 5s → mark visited = True
   └─ failure/timeout → mark attempt failed (no retry logic yet)
   ▼
RESUMING    → publish True → explore/resume   (explore_lite resumes, map continues unchanged)
   ▼
EXPLORING  (loop continues)
```

Only "navigate_to" classes ever touch the Supervisor. `log_only` and `costmap_hazard` classes bypass it entirely and act independently — no interruption to exploration for those.

---

## 5. Per-Class Behavior Policy (example `policy.yaml`)

```yaml
person:
  behavior: log_only

stairs:
  behavior: costmap_hazard
  params:
    radius: 0.5
    cost_value: 200

football:
  behavior: navigate_to
  params:
    approach_offset: 0.3      # meters; avoid driving into the object
    dwell_seconds: 5
    resume_after: true
```

Adding a new class or changing a class's behavior later = editing this file, not the Supervisor's code.

---

## 6. Why Map/Exploration State Is Safe

- SLAM keeps building the map regardless of Nav2 goal activity — pausing/canceling/redirecting goals never touches the map.
- `explore_lite` (m-explore-ros2) exposes an `explore/resume` topic (`std_msgs/Bool`): publish `False` to halt goal generation, `True` to resume — no node restart, no state loss.
- The Supervisor never restarts `explore_lite` or SLAM — it only toggles the resume flag and temporarily takes over the Nav2 action server for one-off goals.

---

## 7. Vision Model Details (recap)

| Item | Choice |
|---|---|
| Model | YOLO11n (nano) |
| Classes | person, sports_ball (football proxy, fine-tune later), stairs (custom), day-to-day objects (subset of COCO + custom) |
| Export path | PyTorch → ONNX → TensorRT engine |
| Precision | INT8 (with calibration set) for max throughput; FP16 fallback if accuracy drops too much |
| Target hardware | Jetson Orin NX (~100 TOPS) |
| Target FPS | ≥30 FPS (comfortably achievable with nano + TensorRT INT8 at 640×640 or lower) |
| Serving | DeepStream or jetson-inference pipeline, not a raw Python loop |

---

## 8. Build Order (suggested)

1. Vision node (detection only, publish `Detection2DArray`) — validate FPS and accuracy in isolation first.
2. 2D→3D lifting + TF transform — validate object map-poses look correct in RViz.
3. Semantic Memory node with dedup logic.
4. Policy router + `policy.yaml` loader.
5. Costmap hazard injection (stairs) — test independently, no Supervisor needed yet.
6. Mission Supervisor + `explore/resume` integration — test pause/resume in isolation before wiring in real detections.
7. Full integration test: run exploration, drop a football in the mapped area, confirm pause → navigate → dwell → resume cycle.

---

## 9. Open Items for Later (not needed now)
- Retry/backoff policy for failed navigate_to attempts.
- Confidence re-verification during DWELLING (e.g. re-detect before marking visited).
- Multiple simultaneous "go-to" detections — priority ordering.
- Battery-return or safety-interrupt behaviors competing with the same Supervisor.