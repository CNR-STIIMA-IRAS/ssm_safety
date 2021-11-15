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
#include <velocity_scaling_iso15066/microinterpolator.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <name_sorting/name_sorting.h>
#include <sensor_msgs/JointState.h>

cnr::control::Microinterpolator interpolator;
bool received_trj=false;
ros::Duration scaled_time;
sensor_msgs::JointStatePtr js;
std::vector<std::string> ordered_names;
std::vector<std::string> fjt_names;


void goalCb(const control_msgs::FollowJointTrajectoryActionGoalConstPtr& fjt_goal)
{
  trajectory_msgs::JointTrajectoryPtr trj;
  trj=boost::make_shared<trajectory_msgs::JointTrajectory>(fjt_goal->goal.trajectory);
  ROS_DEBUG_STREAM("receive trjactory\n"<<*trj);
  interpolator.setTrajectory(trj);
  scaled_time=ros::Duration(0.0);
  fjt_names=trj->joint_names;
  received_trj=true;
}

void jointStateCb(const sensor_msgs::JointStateConstPtr& msg)
{
  js=boost::make_shared<sensor_msgs::JointState>(*msg);
  ordered_names=js->name; // desired order
}

double distance(const std::vector<double>& conf1,
                const std::vector<double>& conf2)
{
  assert(conf1.size()==conf2.size());
  double squared_dist=0.0;
  for (size_t idx=0;idx<conf1.size();idx++)
  {
    squared_dist+=std::pow(conf1.at(idx)-conf2.at(idx),2.0);
  }
  return std::sqrt(squared_dist);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unscaled_joint_target_pub");
  ros::NodeHandle nh;

  ros::Subscriber js_sub=nh.subscribe("/joint_states",1,jointStateCb);
  ros::Subscriber goal_sub=nh.subscribe("/follow_joint_trajectory/goal",1,goalCb);
  ros::Publisher unscaled_joint_target_pub=nh.advertise<sensor_msgs::JointState>("/unscaled_joint_target",1);
  sensor_msgs::JointState unscaled_joint_target;

  double dt=0.01;
  ros::Rate lp(1.0/dt);
  while(ros::ok())
  {
    lp.sleep();
    ros::spinOnce();
    if ( not js  )
    {
      continue;
    }
    else if (not received_trj)
    {
      unscaled_joint_target=*js;
      unscaled_joint_target.header.stamp=ros::Time::now();
      unscaled_joint_target_pub.publish(unscaled_joint_target);
      continue;
    }

    unscaled_joint_target=*js;

    std::vector<double> position=js->position;

    ros::Duration t=scaled_time;
//    ros::Duration t=ros::Duration(std::max(0.0,scaled_time.toSec()-dt));
    trajectory_msgs::JointTrajectoryPoint pnt;
    interpolator.interpolate(t,pnt);

    std::vector<std::string> tmp_trj_names=fjt_names;
    name_sorting::permutationName(ordered_names,
                                  tmp_trj_names,
                                  pnt.positions,
                                  pnt.velocities,
                                  pnt.effort);

    double dist=distance(position,pnt.positions);
    while (t<interpolator.trjTime())
    {
      t+=ros::Duration(0.1*dt);

      tmp_trj_names=fjt_names;
      interpolator.interpolate(t,pnt);
      name_sorting::permutationName(ordered_names,
                                    tmp_trj_names,
                                    pnt.positions,
                                    pnt.velocities,
                                    pnt.effort);
      double new_dist=distance(position,pnt.positions);
      if (new_dist<dist)
      {
        dist=new_dist;
        scaled_time=t;
      }
      else
      {
        break;
      }
    }
    unscaled_joint_target.position=pnt.positions;
    unscaled_joint_target.velocity=pnt.velocities;
    unscaled_joint_target.effort=pnt.effort;

    unscaled_joint_target.name=ordered_names;
    unscaled_joint_target.header.stamp=ros::Time::now();
    unscaled_joint_target_pub.publish(unscaled_joint_target);
  }

  return 0;


}
