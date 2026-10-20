# gz_robot_ws

ROS 2 Jazzy workspace for Gazebo mobile-robot exercises.

## Package: `robot_lab`

| Node | Function |
|---|---|
| `drive_square` | Closed-loop square using odometry feedback. |
| `scan_inspector` | Read-only lidar diagnostics. |
| `go_to_goal` | Drives to a point with no obstacle sensing. |
| `avoid_to_goal` | Deives to a point with obstacle avoidance. |

`robot_lab/geometry.py` contains shared helper functions.

## Run

Gazebo diff-drive demo:

```bash
ros2 launch robot_lab drive_square.launch.py       
ros2 launch robot_lab drive_square.launch.py robot:=vehicle_green
```

TurtleBot3:

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py    # or empty_world
ros2 run robot_lab scan_inspector
ros2 run robot_lab avoid_to_goal --ros-args -p goal_x:=2.0 -p goal_y:=0.0
```
