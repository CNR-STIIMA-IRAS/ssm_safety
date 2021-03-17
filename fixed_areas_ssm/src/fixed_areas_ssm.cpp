#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <geometry_msgs/PoseArray.h>
#include <fixed_areas_ssm/fixed_areas_ssm.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fixed_area_ssm");
  ros::NodeHandle nh;

  ros::Rate lp(30);
  std_msgs::Int64 msg;
  ros::Publisher ovr_pub=nh.advertise<std_msgs::Int64>("safe_ovr_1",1);

  safety::FixedAreasSSM ssm(nh);
  if (!ssm.loadAreas())
  {
    msg.data=0;
    ovr_pub.publish(msg);
    while (ros::ok())
    {
      ROS_ERROR_THROTTLE(10,"fixed areas speed and separation monitoring (SSM) is not well configured ");
      ovr_pub.publish(msg);
      lp.sleep();
    }
  }

  ros_helper::SubscriptionNotifier<geometry_msgs::PoseArray> poses_sub(nh,"poses",1);
  auto cb=boost::bind(&safety::FixedAreasSSM::callback,&ssm,_1);
  poses_sub.setAdvancedCallback(cb);

  while (ros::ok())
  {
    ros::spinOnce();
    msg.data=ssm.getOverride();

    ovr_pub.publish(msg);
    lp.sleep();

  }
  return 0;
}

