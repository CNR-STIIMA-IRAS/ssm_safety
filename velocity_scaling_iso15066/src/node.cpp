/*
Copyright (c) 2020, Manuel Beschi 
UNIBS-DIMI manuel.beschi@unibs.it
CNR-STIIMA manuel.beschi@stiima.cnr.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros/ros.h>
#include <rosdyn_core/primitives.h>
#include <velocity_scaling_iso15066/ssm15066.h>
#include <random>
#include <subscription_notifier/subscription_notifier.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ssm_iso15066");
  ros::NodeHandle nh("~");

  double st=0.02;
  double pos_ovr_change=0.25*st;
  double neg_ovr_change=2.0*st;

  ros::WallRate lp(1.0/st);
  urdf::Model model;
  model.initParam("robot_description");

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  // Loading chain
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

  std::vector<std::string> joint_names=chain->getMoveableJointNames();
  size_t nAx=joint_names.size();

  Eigen::VectorXd velocity_limits(nAx);
  Eigen::VectorXd inv_velocity_limits(nAx);
  Eigen::VectorXd upper_limits(nAx);
  Eigen::VectorXd lower_limits(nAx);
  for (size_t iAx=0; iAx<nAx; iAx++)
  {
    upper_limits(iAx)    = model.getJoint(joint_names.at(iAx))->limits->upper;
    lower_limits(iAx)    = model.getJoint(joint_names.at(iAx))->limits->lower;
    velocity_limits(iAx) = model.getJoint(joint_names.at(iAx))->limits->velocity;
    inv_velocity_limits(iAx) = 1./model.getJoint(joint_names.at(iAx))->limits->velocity;
  }


  ros::Publisher ovr_pb=nh.advertise<std_msgs::Int64>("safe_ovr_1",1);
  ros::Publisher ovr_float_pb=nh.advertise<std_msgs::Float32>("safe_ovr_1_float",1);
  ros_helper::SubscriptionNotifier<geometry_msgs::PoseArray> obstacle_notif(nh,"/poses",1);


  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_notif(nh,"/unscaled_joint_target",1);
  bool unscaled_joint_target_received=false;
  bool poses_received=false;

  Eigen::Matrix<double,3,Eigen::Dynamic> pc_in_b;

  ssm15066::DeterministicSSM ssm(chain);
  ssm.setPointCloud(pc_in_b);

  Eigen::VectorXd q(nAx);
  Eigen::VectorXd dq(nAx);

  std_msgs::Int64 ovr_msg;
  ovr_msg.data=0;
  double last_ovr=0;
  while (ros::ok())
  {
    ros::spinOnce();
    if (js_notif.isANewDataAvailable())
    {
      for (unsigned int iax=0;iax<nAx;iax++)
      {
        q(iax)=js_notif.getData().position.at(iax);
        dq(iax)=js_notif.getData().velocity.at(iax);
      }
      unscaled_joint_target_received=true;
    }
    if (obstacle_notif.isANewDataAvailable())
    {
      geometry_msgs::PoseArray poses=obstacle_notif.getData();
      if (poses.header.frame_id.compare(base_frame))
      {
        ROS_ERROR_THROTTLE(1,"Poses topic has wrong frame, %s instead of %s",poses.header.frame_id.c_str(),base_frame.c_str());
        continue;
      }
      pc_in_b.resize(3,poses.poses.size());
      for (size_t ip=0;ip<poses.poses.size();ip++)
      {
        pc_in_b(0,ip)=poses.poses.at(ip).position.x;
        pc_in_b(1,ip)=poses.poses.at(ip).position.y;
        pc_in_b(2,ip)=poses.poses.at(ip).position.z;
      }
      ssm.setPointCloud(pc_in_b);
      poses_received=true;
    }

    if (not unscaled_joint_target_received)
      ROS_INFO_THROTTLE(2,"unscaled joint target topic has been received yet");
    if (not poses_received)
      ROS_INFO_THROTTLE(2,"poses topic has been received yet");


    double ovr=0;
    if (unscaled_joint_target_received)
    {
      ovr=ssm.computeScaling(q,dq);
    }

    if (ovr>(last_ovr+pos_ovr_change))
      ovr=last_ovr+pos_ovr_change;
    else if (ovr<(last_ovr-neg_ovr_change))
      ovr=last_ovr-neg_ovr_change;
    last_ovr=ovr;
    ovr_msg.data=100*ovr;
    ovr_pb.publish(ovr_msg);

    std_msgs::Float32 msg_float;
    msg_float.data=ovr_msg.data;
    ovr_float_pb.publish(msg_float);
    ROS_DEBUG_STREAM_THROTTLE(1,"ovr = " << ovr_msg.data);
    lp.sleep();
  }


  return 0;


}
