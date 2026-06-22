# Video Script — Zero-Shot Pick-and-Place with Sawyer
## LLM-Driven Robotic Manipulation using Language Models as Trajectory Generators

**Total Duration:** ~7 minutes (≈1:24 per part)
**Reference Paper:** "Language Models as Zero-Shot Trajectory Generators" (Kwon et al., IEEE RA-L 2024) + "Code as Policies" (Liang et al.)

---

## Part 1 — JUN: Simulation Environment & Pipeline Overview (0:00 – 1:24)

**[DEMO: Show Gazebo window with Sawyer robot, table, and objects]**

**Narration:**
Hi, I'm Jun. Traditional robotic manipulation relies on rigid, hard-coded trajectories that are tedious to program and fail in dynamic environments. To solve this, our project implements a zero-shot trajectory generation system for the Sawyer robotic manipulator. The core idea comes from two key papers: "Language Models as Zero-Shot Trajectory Generators" and "Code as Policies". Instead of hard-coding robot behaviors, we use a large language model to generate motion code on the fly from natural language commands.

**[DEMO: Show terminal launching the simulation]**

Our system runs on ROS1 with the Sawyer robot simulated in Gazebo. The entry point is the launch file `sawyer_david.launch`, which orchestrates the full pipeline in five timed stages:

1. **Gazebo world launch** — loads the Sawyer URDF model with an electric gripper, starts the physics engine, and spawns the robot in a custom world with a table and 9 household objects: a block, mustard bottle, coke can, banana, strawberry, bowl, plant, sponge, and potatoes.
2. **Gazebo GUI** — opens the visual interface after 8 seconds so we can watch the simulation.
3. **Scene spawning** — after 18 seconds, `spawn_pick_place_scene.py` places all objects on the table at specific coordinates using Gazebo's spawn service. Each object has a known 3D model (SDF or URDF format).
4. **Safe pose** — after 35 seconds, `send_safe_pose.py` moves the arm to a retracted home position using direct joint commands.
5. **Scan pose** — after 70 seconds, `go_to_top_hand_camera_pos.py` moves the wrist camera to an overhead scanning position, ready for perception.

**[DEMO: Show each phase happening in Gazebo — objects appearing, arm moving]**

This timed sequence ensures everything initializes properly before the robot starts working.

---

## Part 2 — SERGIO: Computer Vision & Scene Perception (1:24 – 2:48)

**[DEMO: Show the wrist camera view — RGB feed with YOLO bounding boxes]**

**Narration:**
I'm Sergio, and I'll explain the perception pipeline. Once the robot's arm is in the scan position, the wrist camera — an RGB-D sensor mounted on the gripper — captures synchronized color and depth images.

**[DEMO: Show code snippet from `capture_frame.py` — the ApproximateTimeSynchronizer]**

The `capture_frame.py` script synchronizes the RGB and depth streams using ROS message filters. We subscribe to three topics:
- `/io/internal_camera/right_hand_camera/image_raw` — color image
- `/io/internal_camera/right_hand_camera/depth/image_raw` — depth map
- `/io/internal_camera/right_hand_camera/camera_info` — camera intrinsics matrix K

The camera pose in the world frame is obtained via TF transforms.

**[DEMO: Show YOLO detection overlay on the table objects]**

The core perception function is `make_scene()` in `llm_api.py`. It runs a fine-tuned **Ultralytics YOLO model** on the RGB frame to detect objects. Our custom dataset includes 9 classes matching the Gazebo models. The YOLO model outputs bounding boxes with class labels and confidence scores.

**[DEMO: Show 2D-to-3D projection diagram — pixel ray to world point]**

The key challenge is converting 2D pixel detections to 3D world coordinates. This is done in `frame2world.py` through a back-projection pipeline:

1. Sample depth from the center of each bounding box using the depth image.
2. Back-project the pixel (u, v, depth) into a 3D ray using the camera intrinsics matrix K.
3. Apply a calibration rotation `R_FIX` that corrects the Gazebo body frame to the ROS optical frame.
4. Rotate into the world frame using the camera's orientation quaternion from TF.
5. Add the camera position to get the final world coordinates.

**[DEMO: Show Gazebo with colored detection spheres — from `detection_markers.py`]**

Each object returns its center position, base position, half-extents, and orientation. The result is a structured scene dictionary that the LLM can understand: the end-effector pose, all detected objects with their 3D positions, and the table height. This is the robot's understanding of its environment.

---

## Part 3 — DAVID: LLM-Powered Code Generation (2:48 – 4:12)

**[DEMO: Show terminal — running `python3 run_task.py "pick up the block and put it in the bowl"`]**

**Narration:**
I'm David, and I'll cover the LLM integration. The heart of the system is `run_task.py`. This script is the bridge between natural language and robot motion.

**[DEMO: Show `system_prompt.md` being read]**

Here's the workflow:

1. **System prompt is loaded** from `system_prompt.md` — a detailed instruction file that teaches the LLM about our robot API, the world coordinate frame, grasping conventions, and the exact function signatures it must use.

2. **The user's task** is passed as a command-line argument — for example: "pick up the block and put it in the bowl".

3. **The prompt is sent to Ollama**, which runs **Qwen2.5-Coder 3B** locally on our machine. The LLM generates Python code that defines a single function called `execute_task()`.

**[DEMO: Show code generation in action — the `ollama_client.py` making the API call]**

The `OllamaChatClient` class sends a POST request to `http://localhost:11434/v1/chat/completions` with the system prompt and user input. It uses a temperature of 0.7 for creativity. The response is expected to be raw Python code.

4. **The generated code is parsed** — stripped of any markdown code fences — and written to `generated_task.py`.

5. **The script imports and calls** `generated_task.execute_task()`, which runs in the ROS context.

**[DEMO: Show the generated `generated_task.py` content being written]**

The system prompt enforces a strict structure: the LLM must call `make_scene()` to get object positions, compute pickup and placement coordinates using specific math rules, build a waypoint sequence, pass it through `adjust_grasp_waypoints()` for snapping, and finally `execute_waypoints()` for motion.

Key rules in the prompt include:
- Orientation: always roll=180°, pitch=0° (top-down grasp), yaw aligned to each object's orientation
- Hover clearance: 20 cm above the object's top surface for collision avoidance
- Spatial reasoning: prepositions like "left of", "right of", "in front of", "behind", "on top of" are translated to explicit coordinate offsets
- Multi-object sequencing: each object gets its own full pick-and-place 6-step cycle

---

## Part 4 — CADAVID: Motion Planning & Robot Control (4:12 – 5:36)

**[DEMO: Show RViz with Sawyer arm moving along a planned trajectory]**

**Narration:**
I'm Cadavid, and I'll explain how the generated code actually moves the robot. This is where `llm_api.py` does the heavy lifting.

**[DEMO: Show `execute_waypoints()` function]**

Once the LLM generates a list of waypoints, the `execute_waypoints()` function processes them sequentially. Each waypoint has:
- `pos` — [x, y, z] target position in world frame
- `ori` — [roll, pitch, yaw] orientation in degrees
- `gripper` — "open" or "close" command

**[DEMO: Show the `adjust_grasp_waypoints()` logic]**

Before execution, `adjust_grasp_waypoints()` refines the waypoints. For each "close" waypoint, it snaps the target to the nearest detected object's center and adjusts the Z height to the object's top surface. This compensates for any imprecision in the LLM's numerical output.

**[DEMO: Show TRAC-IK solving in action]**

For motion, we use **TRAC-IK** — an inverse kinematics solver that computes joint angles from Cartesian poses. The `_compute_ik()` function in `llm_api.py` takes the target x, y, z and r, p, y and finds a valid joint configuration for the 7-DOF Sawyer arm. It uses a Distance-based optimization with the joint angles as seeds.

If the Z height changes by more than 5mm, the system splits the movement:
1. First, move horizontally to the target XY at current Z (safe horizontal transit)
2. Then, move straight down (or up) along Z in 10 small steps using `move_straight_z()` — this ensures a vertical approach for grasping, avoiding collisions

**[DEMO: Show gripper closing on an object]**

The gripper is controlled via the Intera SDK. When closing, `wait_and_check_grasp()` monitors the gripper position over time:
- If the gripper stops mid-way (object grasped) — success
- If it closes completely (no object) — failure detection, abort and release

After all waypoints are executed, the arm returns to the safe pose automatically.

The velocity is scaled to 0.1 (10% of max) for cautious, repeatable motion, with an 8-second timeout per movement.

---

## Part 5 — JOSSUA: Integration & Live Demonstration (5:36 – 7:00)

**[DEMO: Full system live — terminal and Gazebo side by side]**

**Narration:**
I'm Jossua, and I'll show you how everything integrates in practice.

**[DEMO: Run `roslaunch python_scripts_v1 sawyer_david.launch`]**

First, we launch the simulation. The Gazebo world loads with Sawyer at the center, a cafe table in front, and 9 objects arranged in three rows:
- Row 1 (back): mustard, coke can, bowl
- Row 2 (middle): banana, strawberry, plant
- Row 3 (front): sponge, potatoes, block

The arm automatically moves to the scanning position.

**[DEMO: Run `python3 run_task.py "move the block to the left of the bowl"`]**

Now we give a command. The LLM processes this in seconds. Let's trace the flow:

1. `run_task.py` first calls `go_to_top_hand_camera_pos.py` to ensure the arm is in the scan pose.
2. The system prompt is loaded — 117 lines of detailed API specifications and rules.
3. The task is sent to Qwen2.5-Coder via Ollama.
4. The LLM generates `execute_task()` code that calls `make_scene()` → detects block and bowl → computes pickup hover at [0.55, -0.25, 1.01], grasp at [0.55, -0.25, 0.82] → computes placement "to the left of bowl" → dest = [0.55+0.08=0.63, 0.20+0.08=0.28, table_z + half_extents] → builds 6 waypoints.

**[DEMO: Show the generated code being written and executed]**

5. The code is saved to `generated_task.py` and executed.
6. `make_scene()` captures the current frame, runs YOLO, projects to 3D.
7. `adjust_grasp_waypoints()` snaps the grasp coordinate to the actual block position.
8. `execute_waypoints()` moves: hover → grasp block → hover → move to bowl → drop → retreat.

**[DEMO: Show the actual Gazebo animation of the arm completing the task]**

And there it is — the Sawyer arm picks up the block and moves it to the left of the bowl, entirely from a natural language command with no manual programming.

**[DEMO: Show second task — "place the banana in the bowl"]**

Let's try another: "put the banana in the bowl". Again, the LLM generates new code specific to this task, computing different grasp coordinates, different orientations, and a different placement.

**[DEMO: Show the banana being picked and placed in the bowl]**

The system handles orientation alignment, collision-free hover heights, and even detects grasp failures. This zero-shot approach — inspired by the "Language Models as Zero-Shot Trajectory Generators" and "Code as Policies" papers — demonstrates that modern LLMs possess enough understanding of physics and spatial reasoning to control a robot manipulator directly from natural language.

**[CLOSING: Show team names and references]**

Thank you from our team: **Jun, Sergio, David, Cadavid, and Jossua**.

---

## Appendix: Dependency Tree

```
run_task.py (entry point)
├── go_to_top_hand_camera_pos.py     # Move arm to scan pose
│   ├── intera_interface (ROS SDK)
│   └── rospy
├── ollama_client.py                  # LLM API client
│   └── requests (HTTP)
├── system_prompt.md                  # LLM instruction prompt
├── generated_task.py (auto-generated)
│   └── llm_api.py                    # Core API
│       ├── frame2world.py            # 2D→3D projection
│       ├── ultralytics YOLO          # Object detection
│       ├── intera_interface          # Robot control
│       ├── trac_ik_python            # IK solver
│       ├── tf.transformations        # Quaternion math
│       ├── cv_bridge / OpenCV        # Image processing
│       ├── message_filters           # ROS sync
│       └── sensor_msgs              # Camera topics
│
sawyer_david.launch (simulation pipeline)
├── sawyer_world.launch              # Gazebo + Sawyer
│   ├── sawyer.urdf.xacro            # Robot model
│   ├── sawyer.world                 # Physics world
│   ├── sawyer_sim_controllers.launch # Joint controllers
│   └── sawyer_sim_cameras.launch    # Camera plugins
├── spawn_pick_place_scene.py         # Table + 9 objects
│   └── gazebo_msgs (SpawnModel)
├── send_safe_pose.py                 # Home position
└── go_to_top_hand_camera_pos.py      # Scan position
```
