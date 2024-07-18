#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <geometry_msgs/PoseArray.h>
#include <fixed_distance_ssm/fixed_distance_ssm.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fixed_distance_ssm");
  ros::NodeHandle nh;

  ros::Rate lp(30);
  std_msgs::Int64 msg;
  std_msgs::Float32 msg_float;

  ros::Publisher ovr_pub=nh.advertise<std_msgs::Int64>("safe_ovr_1",1);
  ros::Publisher ovr_float_pb=nh.advertise<std_msgs::Float32>("/safe_ovr_1_float",1);

  safety::FixedDistanceSSM ssm(nh);
  if (!ssm.loadAreas())
  {
    msg.data=0;
    msg_float.data=0.0;
    while (ros::ok())
    {
      ROS_ERROR_THROTTLE(10,"fixed areas speed and separation monitoring (SSM) is not well configured ");
      ovr_pub.publish(msg);
      ovr_float_pb.publish(msg_float);
      lp.sleep();
    }
  }

  ros_helper::SubscriptionNotifier<geometry_msgs::PoseArray> poses_sub(nh,"poses",1);
  auto cb=boost::bind(&safety::FixedDistanceSSM::callback,&ssm,_1);
  poses_sub.setAdvancedCallback(cb);

  while (ros::ok())
  {
    ros::spinOnce();
    msg.data=ssm.getOverride();
    msg_float.data = (float) msg.data;

    ovr_pub.publish(msg);
    ovr_float_pb.publish(msg_float);

    lp.sleep();

  }
  return 0;
}

