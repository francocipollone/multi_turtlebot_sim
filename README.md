# multi_turtlebot_sim

## Introduction

This package is meant to be an example on how robots needs to be configured in order to support simulating multi robots in the same environment.

## Platform
ROS 2 Version: Foxy

## Notes

### TF
 - One `/tf` topic is expected to be published for any number of robots
 - Each frame should have a prefix to know which robot they belong to

### ROS
 - Topics should be namespaced.
   - `<robot_prefix>/robot_description`
   - `<robot_prefix>/joint_states`
   - `<robot_prefix>/cmd_vel`
   - `<robot_prefix>/odom`
   - `<robot_prefix>/scan`
 - Nodes should be namespaced.
   - `robot_state_publisher`: Reads robot description and:
      - Publishes `<robot_prefix>/robot_description` topic.
      - Publishes transforms to `/tf`. Transforms should be prefixed for the robot.
      - Subscribes to joint_states. Topic should be prefixed for the robot.
   - Gazebo related nodes
      - `gazebo_ros_diff_drive` plugin: Should be namespaced.
      - `gazebo_ros_joint_state_publisher` plugin: Should be namespaced.


## Try it!

- Launch the standalone simulation, no robot is spawned
```
ros2 launch multi_turtlebot_sim standalone_world.launch.py
```

Note: By default the empty world is used, you can also use `world_name:=turtlebot3_world.world`.

- Spawn a robot using a `robot_prefix` in a particular `x_pose` and `y_pose`.
```
ros2 launch multi_turtlebot_sim spawn_turtlebot3.launch.py robot_prefix:=robot1 x_pose:=0.5 y_pose:=0.5
```
The `robot_prefix` will affect both the ros namespace and the tf prefix for that robot

- Spawn another robot in a different position

```
ros2 launch multi_turtlebot_sim spawn_turtlebot3.launch.py robot_prefix:=robot2 x_pose:=-0.5 y_pose:=-0.5
```

Check how the tf tree is conformed:
```
ros2 run tf2_tools view_frames.py
```

- The tf tree of each robot aren't connected therefore both robots in rviz can't be seen. As the simulation is starting the odom frame for each robot in the {0,0} world position we can add a static transform between both odometry frames to achieve the full tf_tree all connected.

```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot1/odom robot2/odom
```
