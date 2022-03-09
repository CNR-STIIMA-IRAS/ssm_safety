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
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE PLAYODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <subscription_notifier/subscription_notifier.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_hold");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Publisher ovr_pub=nh.advertise<std_msgs::Int64>("safe_ovr_2",1);
  ros_helper::SubscriptionNotifier<std_msgs::String> hold_sub(nh,"hold_topic",1);


  ros::Rate lp(50);
  std_msgs::Int64 msg;
  int scaling = 100;

  enum RobotState{PLAY, PAUSE};
  RobotState state = PLAY;

  while(ros::ok())
  {
    ros::spinOnce();
    if (hold_sub.isANewDataAvailable())
    {
      std::string new_cmd = hold_sub.getData().data;
      if (new_cmd.compare("play")==0)
      {
        ROS_WARN("Command 'play' issued");
        state = PLAY;
      }
      else if (new_cmd.compare("pause")==0)
      {
        ROS_WARN("Command 'pause' issued");
        state = PAUSE;
      }
      else
      {
        ROS_WARN_STREAM(pnh.getNamespace() << "Undefined command " << hold_sub.getData().data);
      }
    }

    if (state == PLAY)
    {
      scaling+=1;
      if (scaling>=100)
        scaling=100;
    }
    else if (state = PAUSE)
    {
      scaling-=5;
      if (scaling<=0)
        scaling=0;
    }

    msg.data = scaling;
    ovr_pub.publish(msg);

    lp.sleep();

  }

  return 0;


}
