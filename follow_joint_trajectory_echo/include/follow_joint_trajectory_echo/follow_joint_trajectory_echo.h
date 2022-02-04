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
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <name_sorting/name_sorting.h>
#include <std_msgs/Int64.h>



class EchoFollowJointTrajectoryAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
  std::string action_name_;
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryFeedback feedback;

  enum State { EXIT, SEND_GOAL, CHECK_CONDITIONS, CANCEL_GOALS, SEND_FBK, REPLAN };
  State state_=EXIT;

  ros::Time t0_;
  ros::Time t0_scaling_;

  boost::shared_ptr<ros_helper::SubscriptionNotifier<std_msgs::Int64>> scaling_notif_;
  int override_threshold_;
  double override_timeout_;
  double allowed_execution_duration_scaling_;
  double allowed_goal_duration_margin_;

  std::string group_name_;
  moveit::planning_interface::MoveGroupInterfacePtr group_;
  robot_model::RobotModelPtr kinematic_model_;
  moveit::core::JointModelGroup* jmg_;
  planning_scene::PlanningScenePtr planning_scene_;


public:

  EchoFollowJointTrajectoryAction(const ros::NodeHandle& nh, const std::string& group_name) :
    as_(nh_, "/"+group_name+"/follow_joint_trajectory_echo", boost::bind(&EchoFollowJointTrajectoryAction::executeCB, this, _1), false),
    ac_("/"+group_name+"/follow_joint_trajectory", true),
    action_name_("/"+group_name+"/follow_joint_trajectory_echo")
  {
    as_.start();
    ac_.waitForServer();
    group_name_ = group_name;
  }

  ~EchoFollowJointTrajectoryAction()
  {
  }

  int init()
  {

    /* Get parameters */
    ros::NodeHandle nh;
    double tmp1;
    if (!nh.getParam("/move_group/trajectory_execution/allowed_execution_duration_scaling",tmp1))
    {
      ROS_FATAL("/move_group/trajectory_execution/allowed_execution_duration_scaling is not defined. Exit.");
      return 0;
    }
    double tmp2;
    if (!nh.getParam("/move_group/trajectory_execution/allowed_goal_duration_margin",tmp2))
    {
      ROS_FATAL("/move_group/trajectory_execution/allowed_goal_duration_margin is not defined. Exit.");
      return 0;
    }
    allowed_execution_duration_scaling_=tmp1;
    allowed_goal_duration_margin_=tmp2;

    ros::NodeHandle pnh("~");

    override_threshold_=5;
    if (!pnh.getParam("override_threshold",override_threshold_))
      ROS_WARN("%s/override_threshold not defined. Deafult %d", nh_.getNamespace().c_str(),override_threshold_);
    override_timeout_=5.0;
    if (!pnh.getParam("override_timeout",override_timeout_))
      ROS_WARN("%s/override_timeout not defined. Deafult %f", nh_.getNamespace().c_str(),override_timeout_);

    /* Create Moveit inteface */
    group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name_);
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    kinematic_model_ = robot_model_loader.getModel();
    jmg_=kinematic_model_->getJointModelGroup(group_name_);

    /* Create and update planning scene */
    ros::ServiceClient ps_client=nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    moveit_msgs::GetPlanningScene srv;
    ps_client.call(srv);
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(kinematic_model_);
    planning_scene_->setPlanningSceneMsg(srv.response.scene);
    moveit_msgs::PlanningScene scene_msg;
    planning_scene_->getPlanningSceneMsg(scene_msg);

    /* Crate subscriber to scaling */
    scaling_notif_.reset(new ros_helper::SubscriptionNotifier<std_msgs::Int64>(nh,"/scaling",1));
    scaling_notif_->waitForANewData(ros::Duration(1.0));

  }

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {

    ros::Rate rate(10);

    int ovr=100;

    double expected_duration = goal->trajectory.points.back().time_from_start.toSec();
    double fjt_timeout=expected_duration*allowed_execution_duration_scaling_+allowed_goal_duration_margin_;

    state_=SEND_GOAL;

    while(state_!=EXIT)
    {
      ros::spinOnce();

      if (scaling_notif_->isANewDataAvailable())
      {
        ovr=scaling_notif_->getData().data;
        ROS_INFO_THROTTLE(0.5,"scaling=%d",ovr);
      }

      switch(state_)
      {
        case EXIT:
          ROS_DEBUG("EXIT");
          break;
        case SEND_GOAL:
          ROS_DEBUG("SEND_GOAL");
          t0_=ros::Time::now();
          t0_scaling_ = t0_;
          ros::Duration(0.2).sleep();
          ac_.sendGoal(*goal);
          state_=CHECK_CONDITIONS;
          break;
        case CHECK_CONDITIONS:
          ROS_DEBUG_THROTTLE(1,"CHECK_CONDITIONS");
          if (ac_.getState().isDone())
          {
            state_=SEND_FBK;
          }
          else if ((ros::Time::now() - t0_).toSec() > fjt_timeout)
          {
            ROS_INFO("Action did not finish before the time out.");
            state_=REPLAN;
          }
          else if (as_.isPreemptRequested() || !ros::ok())
          {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            state_=CANCEL_GOALS;
          }
          else if (ovr<override_threshold_)
          {
            if ((ros::Time::now()-t0_scaling_).toSec()>override_timeout_)
            {
              state_=REPLAN;
            }
          }
          else
          {
            t0_scaling_=ros::Time::now();
          }
          break;
        case CANCEL_GOALS:
          ROS_DEBUG("CANCEL_GOALS");
          as_.setPreempted();
          ac_.cancelAllGoals();
          state_=EXIT;
          break;
        case SEND_FBK:
          ROS_DEBUG("SEND_FBK");
          ROS_INFO("Action finished: %s",ac_.getState().toString().c_str());
          result_ = *ac_.getResult();
          as_.setSucceeded(result_,"result successfully echoed");
          state_=EXIT;
          break;
        case REPLAN:
          ROS_WARN("REPLAN");
          ac_.cancelAllGoals();
          ros::Duration(1.0).sleep();
          std::vector<double> goal_jconfig = goal->trajectory.points.back().positions;
          std::vector<std::string> tmp_names =group_->getActiveJoints();
          name_sorting::permutationName(goal->trajectory.joint_names,tmp_names,goal_jconfig);
          moveit::planning_interface::MoveGroupInterface::Plan plan = planTo(goal_jconfig);
          execute(plan);
          state_=CHECK_CONDITIONS;
          break;

      }
      rate.sleep();
    }
  }

  moveit::planning_interface::MoveGroupInterface::Plan planTo(  const std::vector<double>& goal_jconf )
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    try
    {
      if (!group_->startStateMonitor(3))
      {
        ROS_ERROR("unable to get robot state for group %s",group_name_.c_str());
        return plan;
      }

      robot_state::RobotState state = *group_->getCurrentState();
      std::vector<double> actual_jconf;
      if (jmg_)
        state.copyJointGroupPositions(jmg_, actual_jconf);
      moveit::core::robotStateToRobotStateMsg(state,plan.start_state_);
      group_->setJointValueTarget(goal_jconf);

      if(group_->plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        ROS_WARN("Could not compute plan successfully.");

      return plan;
    }
    catch(const std::exception& ex)
    {
      ROS_ERROR("FJT Echoer::planTo Exception: %s",ex.what());
      return plan;
    }

  }


  bool execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan)
  {
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = plan.trajectory_.joint_trajectory;

    double fjt_result = std::nan("1");

    ac_.sendGoal(goal);
    ROS_INFO("Execution started!");

    ac_.waitForResult(ros::Duration());

    ROS_INFO("Execution finished!");
    return true;
  }

//  bool execute(const std::string& group_name,
//                          const moveit::planning_interface::MoveGroupInterface::Plan& plan)
//  {
//    control_msgs::FollowJointTrajectoryGoal goal;
//    goal.trajectory = plan.trajectory_.joint_trajectory;

//    if (m_fjt_result.find(group_name) == m_fjt_result.end())
//    {
//      ROS_ERROR("Can't find FollowJointTrajectory result for group %s", group_name.c_str());
//      return false;
//    }

//    m_fjt_result.at(group_name) = std::nan("1");

//    auto cb = boost::bind(&manipulation::SkillBase::doneCb,this,_1,_2,group_name);

//    if (m_fjt_clients.find(group_name) == m_fjt_clients.end())
//    {
//      ROS_ERROR("Can't find FollowJointTrajectory client for group %s", group_name.c_str());
//      return false;
//    }

//    if (!m_fjt_clients.at(group_name)->waitForServer(ros::Duration(10)))
//    {
//      ROS_ERROR("Timeout FollowJointTrajectory client for group %s", group_name.c_str());
//      return false;
//    }
//    m_fjt_clients.at(group_name)->sendGoal(goal, cb);

//    ROS_INFO("Execution started!");
//    return true;
//  }

};


