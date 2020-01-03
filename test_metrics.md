# Test Metrics (01.02.2020)

## Goal

To test reported latency issues on the FETCH. I am running everything locally on the FETCH (ssh'ed from another computer). I report the time (eg. `time python [script.py]`). The time includes the initialization, any path planning it has to do, and the actual execution on the physical robot. By not adding `rospy.spin()` at the end of main, ROS stops listening for any future commands from a given script. 

1. `reset_to_base_position.py`: 8.110s (init), 0.011413 (planning for 3 start, 2 goal states) 31.731s (total)
2. `test_base_and_position_ri.py`: 25.404s (total)
3. `test_basics.py`: 31.302s (total)
4. `test_camera_only.py`: 20.559s (total)
5. `test_movement_heuristics.py`: 27.717s (total)
6. `test_picking.py`: 37.597s (total)
7. `test_ri.py`: n/a, 0.007642s (planning path to tuck arm from picking)
8. `test_wrist_roll_ri.py`: throws errors (see screenshots)


## Errors
### 1. Gripper Handler Issue?
```bash
Unhandled exception in thread started by <bound method Gripper._loop_fake of <fetch_core.gripper.Gripper object at 0x7f5b1be77d90>>
Traceback (most recent call last):
  File "/home/jacksonchui/FETCH_CORE/fetch_core/gripper.py", line 210, in _loop_fake
    'fake_head1')
  File "/opt/ros/melodic/lib/python2.7/dist-packages/tf/broadcaster.py", line 68, in sendTransform
    self.sendTransformMessage(t)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/tf/broadcaster.py", line 75, in sendTransformMessage
    self.tf2_broadcaster.sendTransform([transform])
  File "/opt/ros/melodic/lib/python2.7/dist-packages/tf2_ros/transform_broadcaster.py", line 54, in sendTransform
    self.pub_tf.publish(TFMessage(transform))
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/topics.py", line 882, in publish
    self.impl.publish(data)
  File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/topics.py", line 1041, in publish
    raise ROSException("publish() to a closed topic")
rospy.exceptions.ROSException: publish() to a closed topic
```

### 2. Error thrown when robot not in expected position? (see screenshots)
* fix by setting to correct initial position (I've had runs where it doesn't appear when initialized to start_pose with `robot.body_start_pose()`)