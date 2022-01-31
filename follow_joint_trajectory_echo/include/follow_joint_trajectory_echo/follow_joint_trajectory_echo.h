/*
Copyright (c) 2020, Marco Faroni
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
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <subscription_notifier/subscription_notifier.h>

class EchoFollowJointTrajectoryAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
  std::string action_name_;
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryFeedback feedback;

public:

  EchoFollowJointTrajectoryAction(std::string name) :
    as_(nh_, name, boost::bind(&EchoFollowJointTrajectoryAction::executeCB, this, _1), false),
    ac_("/manipulator/follow_joint_trajectory", true),
    action_name_(name)
  {
    as_.start();
    ac_.waitForServer();
  }

  ~EchoFollowJointTrajectoryAction()
  {
  }

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
//    ros::Duration(2.0).sleep();
    control_msgs::FollowJointTrajectoryGoal goal_echo = *goal;
    ac_.sendGoal(goal_echo);

    bool scaling_timer_active = false;
    int sc_threshold = 10;

    int scaling = 100;

    float timeout = 100.0;
    float timeout_scaling = 5.0;



    ros::Time t0 = ros::Time::now();
    ros::Time t0_scaling = ros::Time::now();

    while (!(ac_.getState().isDone()))
    {
      if ((ros::Time::now() - t0).toSec() > timeout)
      {
        ROS_INFO("Action did not finish before the time out.");
        ac_.cancelAllGoals();
        break;
      }
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        ac_.cancelAllGoals();
        break;
      }
      if (scaling < sc_threshold)
      {
        if (scaling_timer_active==false)
          t0_scaling = ros::Time::now();
        if ((ros::Time::now()-t0_scaling).toSec() > timeout_scaling)
        {
          ac_.cancelAllGoals();
          ac_.sendGoal(goal_echo);
        }
      }
      else
      {
        scaling_timer_active = false;
      }
    }

    if (ac_.getState().isDone())
    {
      actionlib::SimpleClientGoalState state = ac_.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
      result_ = *ac_.getResult();
      as_.setSucceeded(result_,"result successfully echoed");
    }
  }

};


