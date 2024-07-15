#pragma
/*
Copyright (c) 2024, Marco Faroni
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
#include <geometry_msgs/PoseArray.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <std_msgs/Int64.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "fixed_areas_ssm/fixed_areas_ssm.h"

namespace safety
{

  class FixedDistanceSSM : public FixedAreasSSM
  {
  protected:
    std::string target_frame_;


  public:
    FixedDistanceSSM(const ros::NodeHandle& nh): FixedAreasSSM(nh)
    {
      if (!nh_.getParam("ssm/target_frame",target_frame_))
      {
        ROS_ERROR("Parameter ssm/base_frame does not exist");
      }
    }

    void callback(const geometry_msgs::PoseArrayConstPtr& msg)
    {
      Eigen::Affine3d T_base_camera;
      T_base_camera.setIdentity();
      tf::StampedTransform tf_base_camera;
      tf::StampedTransform tf_base_target;


      if (target_frame_.compare(base_frame_))
      {
        if (! listener_.waitForTransform(base_frame_.c_str(),target_frame_.c_str(),msg->header.stamp,ros::Duration(0.01)))
        {
          ROS_ERROR_THROTTLE(1,"Could not find a tf from %s to %s. No TF available",base_frame_.c_str(),target_frame_.c_str());
        }
        else
        {
          listener_.lookupTransform(base_frame_,target_frame_.c_str(),msg->header.stamp,tf_base_target);
        }
      }
      else
      {
        tf_base_target.setIdentity();
      }

      if (msg->header.frame_id.compare(base_frame_))
      {

        if (! listener_.waitForTransform(base_frame_.c_str(),msg->header.frame_id,msg->header.stamp,ros::Duration(0.01)))
        {
          ROS_ERROR_THROTTLE(1,"Poses topic has wrong frame, %s instead of %s. No TF available",msg->header.frame_id.c_str(),base_frame_.c_str());
        }
        else
        {
          listener_.lookupTransform(base_frame_,msg->header.frame_id,msg->header.stamp,tf_base_camera);
          tf::poseTFToEigen(tf_base_camera,T_base_camera);
        }
      }
      else
      {
        tf::poseEigenToTF(T_base_camera,tf_base_camera);
      }

      pc_in_b.resize(3,msg->poses.size());
      for (size_t ip=0;ip<msg->poses.size();ip++)
      {
        Eigen::Vector3d point_in_c;
        point_in_c(0)=msg->poses.at(ip).position.x;
        point_in_c(1)=msg->poses.at(ip).position.y;
        point_in_c(2)=msg->poses.at(ip).position.z;
        pc_in_b.col(ip)=T_base_camera*point_in_c;
      }

      double override=100.0;
      for (size_t idx=0; idx<pc_in_b.cols();idx++)
      {
        std::vector<double> p(2);
        p.at(0)=pc_in_b(0,idx) - tf_base_target.getOrigin()[0];
        p.at(1)=pc_in_b(1,idx) - tf_base_target.getOrigin()[1];
        checkArea(p,override);
      }
      target_override_=override;

    }


  };
}
