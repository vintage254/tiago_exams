# Group 25 - TIAGo Exam Project

University of Bologna - Robotics Exam 2025

## Repository Structure

```
tiago_ws/src/
├── tiago_exam/              # Provided: simulation launch files
├── tiago_exam_worlds/       # Provided: world files + models
│   └── worlds/
│       └── group25.world    # OUR group's world file
├── group25_tasks/           # OUR CODE: 3 task nodes + launch files
│   ├── scripts/
│   │   ├── task1_exploration.py
│   │   ├── task2_aruco_navigation.py
│   │   └── task3_pick_and_place.py
│   ├── launch/
│   │   ├── task1_mapping.launch.py
│   │   ├── task2_navigation.launch.py
│   │   └── task3_pick_place.launch.py
│   ├── config/
│   │   └── waypoints.yaml
│   ├── CMakeLists.txt
│   └── package.xml
└── (other TIAGo packages already in workspace)
```

## Setup on Ubuntu Machine

### 1. Prerequisites
- Ubuntu 22.04
- ROS2 Humble installed
- TIAGo workspace (`tiago_ws`) already set up with all TIAGo packages

### 2. Clone this repo into the workspace
```bash
cd ~/tiago_ws/src
git clone <REPO_URL>
```

### 3. Copy files into place
```bash
# Copy the world file
cp group25-exam/group25.world ~/tiago_ws/src/tiago_exam_worlds/worlds/group25.world

# Copy the exam packages (if not already there)
cp -r group25-exam/tiago_exam ~/tiago_ws/src/
cp -r group25-exam/tiago_exam_worlds ~/tiago_ws/src/

# Copy our custom task package
cp -r group25-exam/group25_tasks ~/tiago_ws/src/
```

### 4. Make scripts executable
```bash
chmod +x ~/tiago_ws/src/group25_tasks/scripts/*.py
```

### 5. Build
```bash
cd ~/tiago_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Tasks

### Task 1: SLAM Map Generation

**Terminal 1** - Launch simulation:
```bash
source ~/tiago_ws/install/setup.bash
ros2 launch tiago_exam tiago_exam.launch.py world_name:=group25
```

**Terminal 2** - Launch SLAM + exploration:
```bash
source ~/tiago_ws/install/setup.bash
ros2 launch group25_tasks task1_mapping.launch.py
```

**Terminal 3** - Save map (after exploration finishes):
```bash
mkdir -p ~/my_map
ros2 run nav2_map_server map_saver_cli -f ~/my_map/map
```

---

### Task 2: Navigation to Pick/Place Locations

**IMPORTANT:** For Task 2, the robot must start at a DIFFERENT position.
Edit `tiago_exam/launch/tiago_spawn.launch.py` line 33 to change:
```python
spawn_coordinates = [2.0, -3.0, 0.0]  # Different from Task 1
```

**Terminal 1** - Launch simulation:
```bash
source ~/tiago_ws/install/setup.bash
ros2 launch tiago_exam tiago_exam.launch.py world_name:=group25
```

**Terminal 2** - Launch navigation + ArUco search:
```bash
source ~/tiago_ws/install/setup.bash
ros2 launch group25_tasks task2_navigation.launch.py map_path:=$HOME/my_map
```

---

### Task 3: Pick and Place

**Terminal 1** - Launch simulation WITH MoveIt:
```bash
source ~/tiago_ws/install/setup.bash
ros2 launch tiago_exam tiago_exam.launch.py world_name:=group25 moveit:=true
```

**Terminal 2** - Launch navigation + pick-and-place:
```bash
source ~/tiago_ws/install/setup.bash
ros2 launch group25_tasks task3_pick_place.launch.py map_path:=$HOME/my_map
```

---

## Key Parameters

| Item | Value |
|------|-------|
| Surface ArUco markers | ID 26 (pick), ID 238 (place), 25 cm |
| Cube ArUco markers | ID 63, ID 582, 7 cm |
| Cube dimensions | 7 x 7 x 7 cm |
| ArUco dictionary | DICT_ARUCO_ORIGINAL |
| Pick order | ID 63 first, then ID 582 |
| Robot spawn (Task 1) | (0.0, -1.3, 0.0) |

## Troubleshooting

If Gazebo doesn't close properly after Ctrl+C:
```bash
ps -ef | grep ros
sudo kill -9 <PID>
```

If navigation parameters need tuning, edit:
```
tiago_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_params/params/pmb2_nav2.yaml
```
