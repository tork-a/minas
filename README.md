# minas package [![CircleCI](https://circleci.com/gh/tork-a/minas.svg?style=svg)](https://circleci.com/gh/tork-a/minas)

## Try it on MoveIt! with real robot 

On one terminal, start a minas_control as:

```
roslaunch tra1_bringup tra1_bringup.launch
```

If you have no real robot, specify the argument 'simulation' to use
loopback mode:

```
roslaunch tra1_bringup tra1_bringup.launch simulation:=true
```

On Another terminal, start a moveit script as:

```
roslaunch tra1_bringup tra1_moveit.launch
```

Now you can see the rviz screen of MoveIt! and interact the robot with
the GUI.

