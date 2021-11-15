#pragma once // workaround qtcreator clang-tidy
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/console.h>

namespace cnr
{

namespace control
{


class Microinterpolator
{
protected:
  trajectory_msgs::JointTrajectoryPtr m_trj;
  unsigned int m_order;
public:
  Microinterpolator();
  bool setTrajectory(const trajectory_msgs::JointTrajectoryPtr& trj);
  trajectory_msgs::JointTrajectoryPtr getTrajectory(){return m_trj;}
  /* setSplineOrder:
   * order = 0 positions are continuous
   * order = 1 velocities are continuous
   * order = 2 accelerations are continuous
   * order = 3 jerks are continuous, supposed zero at the waypoints
   * order = 4 snaps are continuous, supposed zero at the waypoints
   */
  void setSplineOrder(const unsigned int& order);
  bool interpolate(const ros::Duration& time, trajectory_msgs::JointTrajectoryPoint& pnt, const double& scaling=1.0);
  ros::Duration trjTime();
};
  
}  // namespace control
}  // namespace cnr
