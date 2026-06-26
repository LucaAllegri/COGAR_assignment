# UR5e + OnRobot 2FG7 Pick-and-Place Simulation <br> (Assignment Subgroup B1)

ROS 2 Humble simulation of a UR5e robotic arm equipped with an OnRobot 2FG7 parallel gripper. The project was developed for the **Cognitive Architectures for Robotics** assignment.

The system performs a collision-aware pick-and-place task in RViz using ROS 2 Control and MoveIt2. It includes cabinet and table collision objects, optional basket and obstacle configurations, object attachment during grasping, random placement-pose selection, joint-space interpolation, and Cartesian descent until contact with the support surface.<br>
(The project was validated in simulation only. The 2FG7 force-feedback capability is discussed through the official gripper documentation and grasp-stability literature; no physical force measurements are used in the simulation).

## Main Features

* UR5e and OnRobot 2FG7 integration in ROS 2 Humble
* MoveIt2 Planning Scene and collision-aware manipulation
* Easy and hard environments, with or without a placement basket
* Simulated contact-based gripper closure and object attachment
* Custom joint-space interpolation with collision validation
* Random valid placement-pose search
* Cartesian vertical descent until contact
* Quantitative output of planning time, execution time, and task success

## Requirements

* Ubuntu 22.04
* ROS 2 Humble
* MoveIt2
* ROS 2 Control
* RViz2
* UR5e + 2FG7 MoveIt configuration packages
* `moveit_planning` library

Build the workspace before running:

```bash
cd ~/b1a
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

Open **three terminals**. In each terminal, source the workspace:

```bash
cd ~/b1a
source install/setup.bash
```

### Terminal 1 — Launch UR5e, MoveIt2, and RViz

```bash
ros2 launch ur5e_2fg7_moveit_config demo.launch.py
```

### Terminal 2 — Launch the Environment

Choose one of the available collision scenes.

```bash
# Easy scene without basket
ros2 launch create_scene scene_easy_nobin.launch.py
```

```bash
# Easy scene with basket
ros2 launch create_scene scene_easy_bin.launch.py
```

```bash
# Hard scene without basket
ros2 launch create_scene scene_hard_nobin.launch.py
```

```bash
# Hard scene with basket
ros2 launch create_scene scene_hard_bin.launch.py
```

### Terminal 3 — Launch the Pick-and-Place Task

```bash
ros2 launch task_pick_and_place launch_task.launch.py
```

In the task_pick_and_place nods's lauch file```launch_task.launch.py```, you can see two flags: ```USE_BIN = False``` ```HARD_SCENE = True```. 
Make sure that both of these flags match the scene selected in Terminal 2.
