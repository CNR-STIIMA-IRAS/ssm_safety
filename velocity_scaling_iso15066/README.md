# velocity_scaling_iso15066

The module provides an optimized speed and separation monitoring package to slow down the robot in case of human-robot proximity.
The module needs the information on the target trajectory, and the actual robot position. Then, the position is projected on the trajectory, and using the information on the human position with respect to the robot, the scaling override is computed.

The module subscribes:  

- /unscaled_joint_target: a message of the type sensor_msgs::JointState with the unscaled joint target. 

- /poses: a message of the type geometry_msgs::PoseArray containing the poses of human body parts (i.e. a human) in the scene reference frame. 

The module publishes: 

- safe_ovr_1: a message of the type std_msgs::Int64 containing the safety override 0-100 (the topics are two, with the same content, to have a redundant communication channel) 

```yaml
base_frame: "world"   # root of the chain
tool_frame: "open_tip"   # end of the chain
test_links:  # links fo the robot used for evaluating the distance 
- tool0
- forearm_link
- open_tip

minimum_distance: 0.3  # optional (default: 0.3). Minimal distance (meter), if slower the robot will stop
maximum_cartesian_acceleration: 0.1  # optional (default: 0.1). Maximum cartesian acceleration (m/s^2)
```