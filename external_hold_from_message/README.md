## Safety monitored protective stop from external trigger

The package contains a simple ROS node that allows for soft-stopping the robot when a trigger signal is received.
I.e., when it receives a "pause" trigger, it drives the robot speed to zero; when it receives the "play" trigger, it resumes the motion previously in execution.

The node receives a ```std_msgs::String``` msg on topic ```hold_topic```.
If the message contains the string ```pause```, the node publishes a ```std_msgs::Int64``` msg smoothly decreasing to zero on topic ```safe_ovr_2```.
If the message contains the string ```play```, the node publishes a ```std_msgs::Int64``` msg smoothly increasing to 1 on topic ```safe_ovr_2```.

