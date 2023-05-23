#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

#include <fixed_areas_ssm/fixed_areas_ssm.h>
#include <rosdyn_core/primitives.h>
#include <name_sorting/name_sorting.h>
#include <sensor_msgs/JointState.h>
#include <subscription_notifier/subscription_notifier.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fixed_area_ssm");

  ros::NodeHandle nh;

  ros::Rate lp(30);
  std_msgs::Int64 msg;
  std_msgs::Float32 msg_float;

  ros::Publisher ovr_pub=nh.advertise<std_msgs::Int64>("safe_ovr_1",1);
  ros::Publisher ovr_pub_float=nh.advertise<std_msgs::Float32>("safe_ovr_1_float",1);

  bool both_agent_checking = true;
  if (!nh.getParam("both_agent_checking",both_agent_checking))
  {
    ROS_INFO_STREAM("Both_agent_checking not set, set to default: false");
  }


  urdf::Model model;
  model.initParam("robot_description");

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  std::string base_frame;
  if (!nh.getParam("base_frame",base_frame))
  {
    ROS_ERROR("%s/base_link not defined",nh.getNamespace().c_str());
    return 0;
  }

  std::string tool_frame;
  if (!nh.getParam("tool_frame",tool_frame))
  {
    ROS_ERROR("tool_link not defined");
    return 0;
  }

  rosdyn::ChainPtr chain = rosdyn::createChain(model,base_frame,tool_frame,grav);
  if (!chain)
  {
    ROS_ERROR("Unable to create a chain between %s and %s",base_frame.c_str(),tool_frame.c_str());
    return 0;
  }

  std::vector<std::string> links_to_test = chain->getLinksName();

  if (!nh.getParam("links_test", links_to_test))
  {
    ROS_INFO_STREAM("Unable to load links_test. It is set to all chain links.");
  }
  for(std::string link_name: links_to_test)
  {
      ROS_INFO_STREAM("Link name to test: "<< link_name);
  }


  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_notif(nh,"/unscaled_joint_target",1);
  std::vector<std::string> joint_names=chain->getMoveableJointNames();
  size_t nAx=joint_names.size();
  ROS_INFO_STREAM("N moveable joint: " << nAx);


  /*
  std::vector<std::string> links_names = chain_->getLinksName();
  size_t nAx=joint_names.size();
  */

  safety::FixedAreasSSM ssm(nh);
  if (!ssm.loadAreas())
  {
    msg.data=0;
    msg_float.data=0.0;
    ovr_pub.publish(msg);
    ovr_pub_float.publish(msg_float);
    while (ros::ok())
    {
      ROS_ERROR_THROTTLE(10,"fixed areas speed and separation monitoring (SSM) is not well configured ");
      ovr_pub.publish(msg);
      ovr_pub_float.publish(msg_float);

      lp.sleep();
    }
  }

  ros_helper::SubscriptionNotifier<geometry_msgs::PoseArray> poses_sub(nh,"poses",1);
  auto cb=boost::bind(&safety::FixedAreasSSM::callback,&ssm,_1);
  poses_sub.setAdvancedCallback(cb);


  Eigen::VectorXd q(nAx);
  std::vector<geometry_msgs::Point> agent_points;

  while (ros::ok())
  {
    ros::spinOnce();
//    ROS_INFO_STREAM("Target ovr: "<< ssm.getTargetOvr());
    if(both_agent_checking)
    {
      if (js_notif.isANewDataAvailable())
      {

        std::vector<double> pos = js_notif.getData().position;

        std::vector<std::string> tmp_names = js_notif.getData().name;


        name_sorting::permutationName(joint_names,tmp_names,pos);

        for (unsigned int iax=0;iax<nAx;iax++)
        {
          q(iax)=pos.at(iax);
        }

        Eigen::Affine3d T_poi_base_link;
        agent_points.clear();
        for(std::string link_name: links_to_test)
        {
          try
          {
            T_poi_base_link = chain->getTransformationLink(q, link_name);
            double x = T_poi_base_link.translation()(0);
            double y = T_poi_base_link.translation()(1);
            double z = T_poi_base_link.translation()(2);
            geometry_msgs::Point point;
            point.x = T_poi_base_link.translation()(0);
            point.y = T_poi_base_link.translation()(1);
            agent_points.push_back(point);

//            std::cout << "[x,y,z] = " << "[" << x << ", " << y << ", " << z << "]" << std::endl;

          }
          catch(std::invalid_argument)
          {
            ROS_INFO_STREAM("Link: " << link_name << " not in chain.");
          }
        }
//        ROS_INFO_STREAM("Check agent inside area result: "<< ssm.secondAgentInArea(agent_points));


      }
      msg.data=ssm.getOverride(agent_points);

    }
    else
    {
      msg.data=ssm.getOverride();
    }

    msg_float.data=msg.data;



    ovr_pub.publish(msg);
    ovr_pub_float.publish(msg_float);
    lp.sleep();

  }
  return 0;
}



