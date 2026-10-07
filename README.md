# e-Yantra Robotics Competition 2026-27 — StrataCobot (SC)

Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic

The task pages are the full instructions. This page is the short version.

## 1. Install and build

From this directory, with ROS 2 Jazzy already installed:

```bash
./requirements.sh
cd ..
colcon build
source install/setup.bash
```

## 2. Start a world

Each subtask has its own launch file. Leave it running in its own terminal.

```bash
ros2 launch eyantra_kepler_colony task2a.launch.py   # Task 2A
ros2 launch eyantra_kepler_colony task2b.launch.py   # Task 2B
```

Wait for `Task 2A world ready` or `Task 2B world ready` before going on.

## 3. Your scripts

Your code goes in the **`algorithms`** package, under these names:

| Subtask | Folder | Scripts |
|---|---|---|
| 2A | `algorithms/scripts/task2a/` | `task2A_perception.py`, `task2A_manipulation.py` |
| 2B | `algorithms/scripts/task2b/` | `task2B_planning.py`, `task2B_navigation.py` |

For each script: make it executable (`chmod +x`), add its path to `SCRIPTS` in
`algorithms/setup.py` (e.g. `'scripts/task2a/task2A_perception.py'`), then rebuild and run it:

```bash
colcon build --packages-select algorithms
source install/setup.bash
ros2 run algorithms task2A_perception.py
```

Rebuild and re-source after every edit: `ros2 run` starts the installed copy.

## 4. Evaluate and submit

1. Start the world and wait for its ready line.
2. In another terminal: `./eyrc-sc-evaluator --task 2A --team-id <YOUR_TEAM_ID>` (or `--task 2B`).
3. When the evaluator prints `Ready`, start your two scripts.
4. Press `q` in the evaluator when the run is done. It writes `result.zip`. Do not unpack or edit it.
5. Zip `result.zip` with your two scripts and upload it, named as the submission page says.

---

The UR7e description in `ur_description` is modified by e-Yantra from the upstream
Universal Robots ROS 2 description; see `ur_description/LICENSE`.
