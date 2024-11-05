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
  std_msgs::Int64 ovr_msg;
  std_msgs::Float32 ovr_msg_float;
  std_msgs::Float32 min_dist_msg;

  ros::Publisher ovr_pub=nh.advertise<std_msgs::Int64>("safe_ovr_1",1);
  ros::Publisher ovr_float_pb=nh.advertise<std_msgs::Float32>("/safe_ovr_1_float",1);
  ros::Publisher min_dist_pb=nh.advertise<std_msgs::Float32>("/min_distance_from_poses",1);

  safety::FixedDistanceSSM ssm(nh);
  if (!ssm.loadAreas())
  {
    ovr_msg.data=0;
    ovr_msg_float.data=0.0;
    while (ros::ok())
    {
      ROS_ERROR_THROTTLE(10,"fixed areas speed and separation monitoring (SSM) is not well configured ");
      ovr_pub.publish(ovr_msg);
      ovr_float_pb.publish(ovr_msg_float);
      lp.sleep();
    }
  }

  ros_helper::SubscriptionNotifier<geometry_msgs::PoseArray> poses_sub(nh,"poses",1);
  auto cb=boost::bind(&safety::FixedDistanceSSM::callback,&ssm,_1);
  poses_sub.setAdvancedCallback(cb);

  while (ros::ok())
  {
    ros::spinOnce();
    if (ssm.hasNewPoses())
    {
      ovr_msg.data=ssm.getOverride();
      ovr_msg_float.data = (float) ovr_msg.data;
      min_dist_msg.data = (float) ssm.getMinDistanceFromPoses();

      ovr_pub.publish(ovr_msg);
      ovr_float_pb.publish(ovr_msg_float);
      min_dist_pb.publish(min_dist_msg);
    }

    lp.sleep();

  }
  return 0;
}

