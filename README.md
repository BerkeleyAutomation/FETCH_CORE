# FETCH_CORE

## Overview

This is our interface for the Fetch robot. Please import `fetch_core` in your
code if using the Fetch, and go from there.

Troubleshooting:

- Make sure you are on the AUTOLAB WiFi with `ROS_MASTER_URI` set up
  appropriately (e.g., in `~/.bashrc`).

- Don't set the Fetch to be as high as 0.4m. The max height we should use in
  practice is probably around 0.35. Make sure the robot is not turned off at the
  max height, otherwise it may be necessary to physically press the robot down.
  (We had to talk with Fetch support about this.)

- If you get some missing trajectory service (e.g., of the Head) then try
  turning off the robot and the breaker, and then turning on the breaker and the
  ON button. Actually that's a good strategy in case anything looks odd.

- If the Joystick (for teleoperation) won't connect to the Fetch, use the USB
  cable to connect it with the Fetch head.

## Tests

Look at `examples/`.

To test these, make sure the physical Fetch is on (or the Gazebo simulator is
running with some launch file) and that your machine is connected to the robot
by being on the AUTOLAB WiFi and with `ROS_MASTER_URI` set up appropriately.

Any time MoveIt is used (for planning, computing inverse kinematics, etc.), you
must have the following command:

```
roslaunch fetch_moveit_config move_group.launch
```

running in the background, for both the simulator and the real robot, before
calling these scripts. If running the simulator, then call something like this:

```
roslaunch fetch_gazebo simulation.launch
```

in a separate command line window.

Once the setup is ready, just call `python [script_name]`.

- `test_robot_interface.py` for basic tests of gripper, cameras, head movement,
  torso.

  **Status**: passing for both simulator and physical robot

- `test_base_and_position.py` for testing if we can move back to a specified
  starting location, also testing base movement more generally

  **Status**: passing for both simulator and physical robot, with some results
  recorded in `examples/README.md`.

- `test_move_to_pose.py` for moving the end-effectors to certain locations.

  **Status**: passing for the simulator, need to test more on the physical robot
  (but so far it's working, modulo some speed/safety issues)



## Other Reminders

When the robot is turned ON, the area where it is turned ON is considered the
`'odom'` link. So be careful if you set points with respect to that frame.

For the physical robot, for BOTH MoveIt! and rviz you need to export the correct
`ROS_MASTER_URI` variable.
