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
#include <follow_joint_trajectory_echo/follow_joint_trajectory_echo.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_joint_trajectory_echo");
  ros::NodeHandle pnh("~");

  /*
   * Add getParam
   * Get maximum execution time and set timeout smaller than that
   * Create launcher for multi-robot
   * Handle re-planning in class
   */

  std::string group_name;
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_FATAL("%s/group_name is not defined. Exit.", pnh.getNamespace().c_str());
    return 0;
  }

  EchoFollowJointTrajectoryAction echoer(pnh, group_name);

  echoer.init();

  ROS_INFO("EchoFollowJointTrajectoryAction created");

  ros::spin();

  return 0;

}
