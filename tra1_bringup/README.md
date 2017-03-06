# tra1_bringup

## Quick start

### ROS controller

Start ROS controllers

```
$ roslaunch tra1_bringup tra1_bringup.launch
```

This will publish and subscribe following topics.
 
```
$ rostopic list
/diagnostics
/joint_states
/position_trajectory_controller/command
/position_trajectory_controller/follow_joint_trajectory/cancel
/position_trajectory_controller/follow_joint_trajectory/feedback
/position_trajectory_controller/follow_joint_trajectory/goal
/position_trajectory_controller/follow_joint_trajectory/result
/position_trajectory_controller/follow_joint_trajectory/status
/position_trajectory_controller/state
/rosout
/rosout_agg
/tf
/tf_static
```

To get current joint status, subscribe `/joint_states`, type of `sensor_msgs/JointState`.

To control the robot joint, publish `/position_trajectory_controller/follow_joint_trajectory/goal`, with `control_msgs/FollowJointTrajectoryActionGoal` message.

### rqt_joint_trajectory_controller

To control the robot using GUI, run

```
$ rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller 
```

### MoveIt

To control via MoveIt, run

```
$ tra1_bringup/launch/tra1_moveit.launch
```