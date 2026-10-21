# gz_robot_ws

ROS 2 Jazzy workspace for Gazebo mobile-robot exercises.

## Package: `robot_lab`

| Node | Function |
|---|---|
| `drive_square` | Closed-loop square using odometry feedback. |
| `scan_inspector` | Read-only lidar diagnostics. |
| `go_to_goal` | Drives to a point with no obstacle sensing. |
| `avoid_to_goal` | Drives to a point with obstacle avoidance. |
| `patrol` | Sends a waypoint list to Nav2 via the NavigateToPose action. |

`robot_lab/geometry.py` contains shared helper functions.
`config/` holds parameter files, `maps/` a SLAM-built map of `turtlebot3_world`.

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

Nav2, using the prebuilt map:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True \
  params_file:=$HOME/gz_robot_ws/src/robot_lab/config/nav2_burger.yaml
ros2 run robot_lab patrol --ros-args -p use_sim_time:=true \
  -p waypoints:="[1.5, 0.0, 1.5, 1.5, -1.0, 1.0]"
```

SLAM, to build a new map:

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:=$HOME/gz_robot_ws/src/robot_lab/config/slam_burger.yaml
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
ros2 run nav2_map_server map_saver_cli --ros-args -p use_sim_time:=true \
  -p save_map_timeout:=20.0 -f ~/gz_robot_ws/src/robot_lab/maps/my_map
```

Nav2 and SLAM each run alongside a TurtleBot3 world launched as above.

## Notes

- TurtleBot3's `/cmd_vel` is `geometry_msgs/TwistStamped`, not `Twist`; the
  diff-drive demo robot wants plain `Twist`. Nodes here take a `stamped`
  parameter.
- `/scan` is published best-effort, so subscribe with `qos_profile_sensor_data`.
