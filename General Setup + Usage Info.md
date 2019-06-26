# Using the Fetch Robotics: Fetch and Freight robot

Updated: 06.24.19 by Jackson Chui

## Setting Up the FETCH with ROS and RVIZ

Heavily referenced [Fetch Docs: Getting Started](http://docs.fetchrobotics.com/in_the_box.html?highlight=indigo).

The Fetch is on `Ubuntu 14.04.6 LTS`. We are using an Apple Macbook Pro running `Ubuntu 14.04 LTS` (as of this writting Daniel's MacbookPro11,3). Our fetch has the robot name of `fetch59`.

### 1. **Connect the Fetch to a wifi network.** 
If your fetch is not connected to a wifi network, connect a monitor, mouse and keyboard and log into Ubuntu directly on the robot to get that  setup. The default username is _fetch_ and the default password is _robotics_. Once you have the wifi setup, when you are on the same network as the fetch, you can use `ssh fetch@fetch59` in order to ssh into the robot.
### 2. **Install ROS Indigo onto your computer.** 
We will be using ROS to communicate with the robot. Follow the instructions in the following [link](http://wiki.ros.org/indigo/Installation/Ubuntu).
### 3. **Take the keyboard for a spin.** 
Once you install ROS, we should be able to communicate with it. In the terminal on your computer, run in terminal the following lines. If the lines below are not sufficient enough, you might have to manually install the [teleop_twist_keyboard pkg](http://wiki.ros.org/teleop_twist_keyboard).
``` bash
>$ export ROS_MASTER_URI=http://fetch59:11311
>$ rosrun teleop_twist_keybaord teleop_twist_keyboard.py
```
### 4. **Get ROS Running** 
Run the following...
``` bash
>$ export ROS_MASTER_URI=http://fetch59:11311
>$ rosrun rviz rviz
```
Once you get an RViz window, you want to go to `Displays/Global Options/Fixed Frame` in the left sidebar, double click on `map` and select `base_link`. Next, go to `Add` at the bottom and add RobotModel. By now, you should see just the fetch in the frame. I recommend that you also add PointCloud2, TF, Grid, and LaserScan.

Play around with the settings. Test different configs by checking the box to the right to turn them on and off until you get something you like.
### 5. **Saving your ROS Config**

Create a config folder in your home folder (I call mine `rviz_configs`). From rviz window, go to the top menu bar (if you're using GNOME desktop) and `File/Save as...` and save it to the folder.

# **(CHECK to make sure it loads...might need to make this a module)**
### 6. **Loading Saved ROS Config**

To load a save config, you load rviz with the rviz file
Running rviz with config: 
``` bash
>$ rosrun rviz rviz -d rospack find [directory to .rviz config file]
```
### 7. (Optional) Runtime Monitoring
``` bash
>$ export ROS_MASTER_URI=http://fetch59:11311
>$ rosrun rqt_runtime_monitor rqt_runtime_monitor
```

Do you wish you could access the logs? Well ROS has you covered. "The runtime monitor will have one entry per motor controller board (MCB), as well as one entry per breaker. Each of these entries will be classified as either stale, an error, a warning, or OK. In the above image, the supply_breaker is disabled because the robot is not plugged in – this is only a warning, and not actually an issue. Common errors that can be detected are overly hot motors or breakers, breakers that have tripped. When the runstop on Fetch is pressed, a number of breakers become disabled and the motor controller boards are turned off, causing them to go stale." (_Official Fetch & Freight Repo_)

### **My Thoughts on ROS**

I am no ROS expert, but I was able to get this up and running in a day and play around with the Realsense camera on the Fetch in a ton of different configurations. It does all the RGBD stuff you expect and creates point clouds in 3D among other things. There is a Q&A section below and always the official docs linked at the top. Have fun!

## Questions and Answers

* How do I drive the Fetch? 
  * It's a holonomic drive train, so you strafe forward and backward and rotate around. [Here](http://docs.fetchrobotics.com/in_the_box.html?highlight=indigo#driving-fetch-and-freight-with-a-joystick) is the driving reference
* Is the Fetch fine on 14.04 LTS? Is there anything I need to install?
  * Yes it is fine on 14.04, but it is highly recommended you connect from a 14.04 laptop (yes that's old). Upgrading to 18.04 is supposedly possible (but probably would require a lot of debugging and frustration).
  * The Fetch should have everything you need. No need to really upgrade and install anything. Just install stuff on your ROS comptuer.
* ROS isn't working and I set the `ROS_MASTER_URI` and everything seems to be configured properly. What should I do?
  * Make sure both the Fetch and your computer are on the same (wifi) network. Make sure the Fetch is on and charged. Make sure that the red spinny thing on the side of the robot isn't locked in (push and twist right if it is and it should pop out if it was locked).
* I ran `rosrun rviz rviz`, but nothing shows?
  * Under `Displays/Global Options/Fixed Frame` in the left side menu, click on `map` and change it to base link. Then, go to the `Add` button at the bottom and select `RobotModel` and press okay. The Fetch should now show.
* What is `fetch59`?
  * It's our fetch, but usually they are not all named that. Your fetch should have a number on the side (ours says 0059), Replace `fetch59` with your own `<robot_name_or_ip>`.
* There is no need to do anything special for the robot model (your `ROS_MASTER_URI` tells rviz that you are connecting to a Fetch robot). Yes, 14.04 is a pain in the a** (I'll let you know if you can upgrade it and how to make that process go smoothly ^__^ ).