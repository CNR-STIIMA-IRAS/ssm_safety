/*
Copyright (c) 2020, Marco Faroni
UNIBS-DIMI manuel.beschi@unibs.it
CNR-STIIMA marco.faroni@stiima.cnr.it
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
#include <ros/console.h>
#include <rosdyn_core/primitives.h>
#include <velocity_scaling_multi_robot/ssm_multirobot.h>
#include <random>
#include <subscription_notifier/subscription_notifier.h>
#include <robot_state_controller_msgs/PoseTwistArray.h>
#include <robot_state_controller_msgs/PoseTwist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <name_sorting/name_sorting.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ssm_multirobot");
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
  // Loading publishing obstacles information
  bool publish_obstacles;
  if (!nh.getParam("publish_obstacles",publish_obstacles))
  {
    ROS_ERROR("%s/publish_obstacles not defined, set false",nh.getNamespace().c_str());
    publish_obstacles = false;
  }

  double sphere_radius;
  if(publish_obstacles)
  {
    if (!nh.getParam("sphere_radius",sphere_radius))
    {
      ROS_ERROR("%s/sphere_radius not defined, set 0.3",nh.getNamespace().c_str());
      sphere_radius = 0.3;
    }
  }

  // Loading expected robots
  std::vector<std::string> topics_pose_twist;
  if (!nh.getParam("topics_pose_twist",topics_pose_twist))
  {
    ROS_ERROR("%s/topics_pose_twist not defined.");
  }


  tf::TransformListener listener;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

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


  ros::Publisher ovr_pb=nh.advertise<std_msgs::Int64>("/safe_ovr_1",1);
  ros::Publisher ovr_float_pb=nh.advertise<std_msgs::Float32>("/safe_ovr_1_float",1);
  ros::Publisher dist_pb=nh.advertise<std_msgs::Float32>("/min_distance_from_poses",1);
  std::vector<std::shared_ptr<ros_helper::SubscriptionNotifier<robot_state_controller_msgs::PoseTwistArray>>> obstacle_notif_array;

  for (const std::string& topic: topics_pose_twist)
    {
      std::shared_ptr<ros_helper::SubscriptionNotifier<robot_state_controller_msgs::PoseTwistArray>> sub_ptr(new ros_helper::SubscriptionNotifier<robot_state_controller_msgs::PoseTwistArray>(nh,topic,1));
      obstacle_notif_array.push_back(sub_ptr);
  }

  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_notif(nh,"/unscaled_joint_target",1);
  bool unscaled_joint_target_received=false;
  bool poses_received=false;

  Eigen::Matrix<double,3,Eigen::Dynamic> pc_in_b;
  Eigen::Matrix<double,3,Eigen::Dynamic> vc_in_b;

  ssm15066::MultiRobotSSM ssm(chain);

  ssm.setPointCloud(pc_in_b);
  ssm.setPointCloudTwist(vc_in_b);

  Eigen::VectorXd q(nAx);
  Eigen::VectorXd dq(nAx);

  std_msgs::Int64 ovr_msg;
  ovr_msg.data=0;
  double last_ovr=0;

  std::string pose_frame_id;

  int iter=0;
  ros::Time last_pose_topic=ros::Time(0);
  while (ros::ok())
  {
    ros::spinOnce();
    double ovr=0;
    double min_ovr=1;
    bool error=false;
    if (js_notif.isANewDataAvailable())
    {
      std::vector<double> pos = js_notif.getData().position;
      std::vector<double> vel = js_notif.getData().velocity;

      std::vector<std::string> tmp_names = js_notif.getData().name;
      name_sorting::permutationName(joint_names,tmp_names,pos,vel);

      for (unsigned int iax=0;iax<nAx;iax++)
      {
        q(iax)=pos.at(iax);
        dq(iax)=vel.at(iax);
      }
      unscaled_joint_target_received=true;
    }

    /* Print links and poses for debug */
    if (iter==500 || iter==0)
    {
      auto links = chain->getLinks();
      std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> Tbl = chain->getTransformations(q);

      ROS_INFO("Links used for safety check: %d",links.size());
      for (unsigned int idx=0;idx<links.size();idx++)
      {
        double x = Tbl.at(idx).translation()(0);
        double y = Tbl.at(idx).translation()(1);
        double z = Tbl.at(idx).translation()(2);
        std::cout << "#" << idx << " : " << links.at(idx)->getName() << "\t";
        std::cout << "[x,y,z] = " << "[" << x << ", " << y << ", " << z << "]" << std::endl;
      }

      ROS_INFO("Joints used for safety check: %d",joint_names.size());
      for (unsigned int idx=0;idx<joint_names.size();idx++)
      {
        std::cout << idx << ": " << joint_names.at(idx) << " : " << q(idx) << std::endl;
      }

      iter=1;
    }
    iter++;

    for (auto& obstacle_notif: obstacle_notif_array)
    {
      double ovr_i=0.0;

      if (obstacle_notif->isANewDataAvailable())
      {
        robot_state_controller_msgs::PoseTwistArray poses=obstacle_notif->getData();
        Eigen::Affine3d T_base_camera;
        T_base_camera.setIdentity();
        tf::StampedTransform tf_base_camera;

        if (poses.header.frame_id.compare(base_frame))
        {

          if (not listener.waitForTransform(base_frame.c_str(),poses.header.frame_id,poses.header.stamp,ros::Duration(0.01)))
          {
            ROS_ERROR_THROTTLE(1,"Poses topic has wrong frame, %s instead of %s. No TF available",poses.header.frame_id.c_str(),base_frame.c_str());
            error=true;
          }
          else
          {
            listener.lookupTransform(base_frame,poses.header.frame_id,poses.header.stamp,tf_base_camera);
            tf::poseTFToEigen(tf_base_camera,T_base_camera);
          }
        }
        else
        {
          tf::poseEigenToTF(T_base_camera,tf_base_camera);
        }

        pc_in_b.resize(3,poses.pose_twist_array.size());
        vc_in_b.resize(3,poses.pose_twist_array.size());

        for (size_t ip=0;ip<poses.pose_twist_array.size();ip++)
        {
          Eigen::Vector3d point_in_c;
          point_in_c(0)=poses.pose_twist_array.at(ip).pose.position.x;
          point_in_c(1)=poses.pose_twist_array.at(ip).pose.position.y;
          point_in_c(2)=poses.pose_twist_array.at(ip).pose.position.z;
          pc_in_b.col(ip)=T_base_camera*point_in_c;

          point_in_c(0)=poses.pose_twist_array.at(ip).twist.linear.x;
          point_in_c(1)=poses.pose_twist_array.at(ip).twist.linear.y;
          point_in_c(2)=poses.pose_twist_array.at(ip).twist.linear.z;
          vc_in_b.col(ip)=T_base_camera.linear()*point_in_c;
        }

        ssm.setPointCloud(pc_in_b);
        ssm.setPointCloudTwist(vc_in_b);

        poses_received=true;
        last_pose_topic=ros::Time::now();

        //#if ROS_VERSION_MINIMUM(1, 15, 1)
        if(publish_obstacles)
        {
          shape_msgs::SolidPrimitive primitive;
          primitive.type = primitive.SPHERE;
          primitive.dimensions.resize(1);
          primitive.dimensions[0] = sphere_radius;

          moveit_msgs::CollisionObject collision_object;

          collision_object.header.frame_id=base_frame;
          collision_object.header.stamp=ros::Time::now();
  //        collision_object.pose.orientation.w=1;
  //        collision_object.id="skeleton_obs";
          if (poses.pose_twist_array.size()>0)
          {
            pose_frame_id = "skeleton_obj_";
            pose_frame_id.append(poses.header.frame_id);

            collision_object.id = pose_frame_id;
            collision_object.operation = collision_object.ADD;

            for (size_t ip=0;ip<poses.pose_twist_array.size();ip++)
            {
              geometry_msgs::Pose p;
              p.position.x=pc_in_b.col(ip)(0);
              p.position.y=pc_in_b.col(ip)(1);
              p.position.z=pc_in_b.col(ip)(2);
              p.orientation.w=1.0;
              collision_object.primitive_poses.push_back(p);
              collision_object.primitives.push_back(primitive);
            }
          }
          else
          {
            collision_object.operation = collision_object.REMOVE;
          }

          std::vector<moveit_msgs::CollisionObject> collision_objects; //addCollisionObect requires a vector
          collision_objects.push_back(collision_object);
          planning_scene_interface.addCollisionObjects(collision_objects);
        }
        //#endif
      }

      // poses is old
      if ((ros::Time::now()-last_pose_topic).toSec()>0.5)
      {
        pc_in_b.resize(3,0);
        vc_in_b.resize(3,0);
        ssm.setPointCloud(pc_in_b);
        ssm.setPointCloudTwist(vc_in_b);
        error=true;
      }

      if (not unscaled_joint_target_received)
        ROS_INFO_THROTTLE(2,"unscaled joint target topic has been received yet");
      if (not poses_received)
        ROS_INFO_THROTTLE(2,"poses topic has been received yet");

      if (unscaled_joint_target_received)
      {
        ovr_i=ssm.computeScaling(q,dq);
      }
      if (error)
        ovr_i=0.0;

      if (ovr_i<min_ovr)
        min_ovr=ovr_i;

      ovr=min_ovr;

    } // endfor obstacle notifiers

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

    msg_float.data=ssm.getDistanceFromClosestPoint();
    dist_pb.publish(msg_float);

    lp.sleep();
  }


  return 0;


}
