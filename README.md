# Group 25 - TIAGo Exam Project

University of Bologna - Robotics Exam 2025

## What's in This Repo

This repo IS the complete `tiago_ws/src` folder. Everything is included:

```
tiago_exams/                      (this repo)
├── group25_tasks/                # OUR CODE — 3 task scripts + launch files
│   ├── scripts/
│   │   ├── task1_exploration.py       # Task 1: SLAM autonomous exploration
│   │   ├── task2_aruco_navigation.py  # Task 2: ArUco marker navigation
│   │   └── task3_pick_and_place.py    # Task 3: Pick and place pipeline
│   ├── launch/
│   │   ├── task1_mapping.launch.py
│   │   ├── task2_navigation.launch.py
│   │   └── task3_pick_place.launch.py
│   ├── config/waypoints.yaml
│   ├── CMakeLists.txt
│   └── package.xml
├── tiago_exam/                   # Exam launch files (from professor)
├── tiago_exam_worlds/            # World files + models (includes group25.world)
└── src/                          # All TIAGo robot packages
    ├── tiago_robot/
    ├── tiago_simulation/
    ├── tiago_moveit_config/
    ├── tiago_navigation/
    ├── pal_navigation_cfg_public/
    ├── pymoveit2/
    └── ...more packages
```

---

## Setup on Ubuntu 22.04 (Step by Step)

### Step 1: Install ROS2 Humble

If not already installed, follow:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

After install, add to your `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
```

### Step 2: Install required system packages

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-cv-bridge \
  ros-humble-moveit \
  ros-humble-moveit-ros-planning \
  ros-humble-moveit-ros-move-group \
  python3-opencv \
  python3-colcon-common-extensions \
  ros-humble-play-motion2 \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller
```

### Step 3: Clone this repo as your workspace

```bash
cd ~
git clone https://github.com/vintage254/tiago_exams.git tiago_ws/src
```

This creates `~/tiago_ws/src/` with everything inside.

### Step 4: Make all Python scripts executable

```bash
chmod +x ~/tiago_ws/src/group25_tasks/scripts/*.py
chmod +x ~/tiago_ws/src/tiago_exam/scripts/*.py
```

### Step 5: Build the workspace

```bash
cd ~/tiago_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

If the build succeeds, source it:
```bash
source ~/tiago_ws/install/setup.bash
```

Add this to `~/.bashrc` so you don't have to source every time:
```bash
echo "source ~/tiago_ws/install/setup.bash" >> ~/.bashrc
```

---

## Running the 3 Tasks

**IMPORTANT:** For every new terminal, always run:
```bash
source ~/tiago_ws/install/setup.bash
```

---

### Task 1: SLAM Map Generation

The robot explores the environment autonomously and builds a map.

**Terminal 1** — Start the simulation:
```bash
ros2 launch tiago_exam tiago_exam.launch.py world_name:=group25
```
Wait until Gazebo is fully loaded and the robot is visible.

**Terminal 2** — Start SLAM + autonomous exploration:
```bash
ros2 launch group25_tasks task1_mapping.launch.py
```
The robot will:
1. Do a 360-degree rotation to seed the SLAM map
2. Navigate through 17 waypoints covering the entire environment
3. Print progress to the terminal

**Terminal 3** — Save the map (after exploration completes):
```bash
mkdir -p ~/my_map
ros2 run nav2_map_server map_saver_cli -f ~/my_map/map
```

This creates `~/my_map/map.pgm` and `~/my_map/map.yaml`. You need these for Tasks 2 and 3.

**Kill everything** with Ctrl+C in all terminals before starting Task 2.

---

### Task 2: Navigation to Pick/Place Locations Using ArUco

The robot uses the saved map to navigate, detects ArUco markers on the pick and place surfaces, and drives to them.

**IMPORTANT — Change spawn position first:**
Edit `~/tiago_ws/src/tiago_exam/launch/tiago_spawn.launch.py`, line 33.
Change from:
```python
spawn_coordinates = [0.0, -1.3, 0.0]
```
To a different position, e.g.:
```python
spawn_coordinates = [2.0, -3.0, 0.0]
```

**Terminal 1** — Start the simulation:
```bash
ros2 launch tiago_exam tiago_exam.launch.py world_name:=group25
```

**Terminal 2** — Start navigation + ArUco detection:
```bash
ros2 launch group25_tasks task2_navigation.launch.py map_path:=$HOME/my_map/map.yaml
```

The robot will:
1. Move to search positions around the environment
2. Rotate and look for ArUco marker ID 26 (pick surface) and ID 238 (place surface)
3. Navigate to each detected surface

**Kill everything** with Ctrl+C before starting Task 3.

---

### Task 3: Pick and Place

The robot picks up two ArUco cubes from the pick surface and places them on the place surface.

**Note:** Change spawn position back or use any appropriate position.

**Terminal 1** — Start the simulation WITH MoveIt:
```bash
ros2 launch tiago_exam tiago_exam.launch.py world_name:=group25 moveit:=true
```

**Terminal 2** — Start navigation + pick-and-place:
```bash
ros2 launch group25_tasks task3_pick_place.launch.py map_path:=$HOME/my_map/map.yaml
```

The robot will:
1. Navigate to the pick surface (ArUco ID 26)
2. Detect cube with ArUco ID 63
3. Pick it up using the arm + gripper
4. Navigate to the place surface (ArUco ID 238)
5. Place the cube
6. Return for the second cube (ArUco ID 582)
7. Pick and place the second cube

---

## Key Parameters

| Item | Value |
|------|-------|
| Surface ArUco markers | ID 26 (pick table), ID 238 (place table), 25 cm |
| Cube ArUco markers | ID 63 (first), ID 582 (second), 7 cm |
| Cube dimensions | 7 x 7 x 7 cm, 0.05 kg |
| ArUco dictionary | DICT_ARUCO_ORIGINAL |
| Pick order | ID 63 first, then ID 582 |
| Robot default spawn | (0.0, -1.3, 0.0) |
| Pick surface world position | (0.95, -5.6, 0.15) |
| Place surface world position | (2.5, -9.0, 0.15) |

---

## Troubleshooting

**Gazebo won't close properly after Ctrl+C:**
```bash
killall -9 gzserver gzclient
```

**"Could not find controller" errors:**
Make sure MoveIt is enabled for Task 3:
```bash
ros2 launch tiago_exam tiago_exam.launch.py world_name:=group25 moveit:=true
```

**Robot doesn't move / NavigateToPose fails:**
- Check that Nav2 is running: `ros2 node list | grep nav`
- Check the map loaded: `ros2 topic echo /map --once`

**Arm joint positions need tuning:**
The predefined arm positions in `task3_pick_and_place.py` are approximate. If the robot misses the cubes, adjust `_home_position`, `_pre_grasp_position`, and the grasp sequence joint values in the script.

**Navigation parameters:**
If the robot has trouble navigating, edit:
```
~/tiago_ws/src/src/pal_navigation_cfg_public/pal_navigation_cfg_params/params/pmb2_nav2.yaml
```
