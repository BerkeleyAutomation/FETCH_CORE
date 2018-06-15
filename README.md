# FETCH_CORE

This is our interface for the Fetch. 

Partial list of requirements:

- Python 2.7
- Ubuntu 14.04
- ROS Indigo

Quick start:

- Install via `python setup.py develop`. (The `develop` argument meas you can
  change the code here and see the updates immediately reflected in the code
  rather than re-installing.)
- Use the skeleton code provided in `fetch_core/skeleton.py`. This provides
  minimum functionality to access all relevant parts of the code.
- To code, start off by using code similar to `examples/test_skeleton.py` which
  will give you an overview of common code usage. The `examples/` directory has
  other example code usage and screenshots of the Fetch in action.
- For the Siemens challenge ([repository here][1]), we used a different skeleton
  class, `fetch_core/robot_interface.py`, which has some hard-coded base and
  head tilting for the challenge. 

TODOs:

- Create a pose for the robot to go to based on camera image pixels (e.g., of a
  target object to grasp). We have draft code in `fetch_core/robot_interface.py`
  in tandem with `fetch_core/gripper.py`, but it is not working yet. The
  skeleton code defines poses with respect to the base link, which is easier to
  do but harder in practical situations.


## Troubleshooting

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

- When the robot is turned ON, the area where it is turned ON is considered the
  `'odom'` link. So be careful if you set points with respect to that frame.

- For the physical robot, for BOTH MoveIt! and rviz you need to export the
  correct `ROS_MASTER_URI` variable.

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
