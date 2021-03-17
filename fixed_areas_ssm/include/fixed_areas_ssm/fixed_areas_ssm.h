#pragma
/*
Copyright (c) 2019, Manuel Beschi CNR-STIIMA manuel.beschi@stiima.cnr.it
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
namespace safety
{

  class ConvexPolygon
  {
  protected:
    std::vector<std::vector<double>> corners_;
    std::vector<std::vector<double>> normals_;

    double override_;

  public:
    ConvexPolygon(const std::vector<std::vector<double>>& corners,
                  const double& override):
      corners_(corners),
      override_(override)
    {
      ROS_DEBUG("loading area with maximum override=%f, with the following corners\n", override_);
      normals_.resize(corners_.size());
      for (const std::vector<double>& c: corners_)
      {
        if (c.size()!=2)
        {
          ROS_FATAL("corners dimension should be 2");
          assert(c.size()==2);
        }
        ROS_DEBUG_STREAM(c.at(0) << ", " << c.at(1));
      }

      for (unsigned int idx=0;idx<corners_.size();idx++)
      {
        unsigned inext=idx+1;
        if (inext==corners_.size())
          inext=0;

        std::vector<double>& corner1=corners_.at(idx);
        std::vector<double>& corner2=corners_.at(inext);
        std::vector<double> tangent;
        std::vector<double>& normal=normals_.at(idx);

        double length=std::sqrt(std::pow(corner2.at(0)-corner1.at(0),2)+std::pow(corner2.at(1)-corner1.at(1),2));
        tangent.resize(2);
        normal.resize(2);
        tangent.at(0)=(corner2.at(0)-corner1.at(0))/length;
        tangent.at(1)=(corner2.at(1)-corner1.at(1))/length;
        normal.at(0)=-tangent.at(1);
        normal.at(1)=tangent.at(0);

        bool correct_direction=false; // correct_direction is true if there is at least one corner where dot(c,n)>0.
        for (unsigned int ic=0;ic<corners_.size();ic++)
        {
          std::vector<double>& p=corners_.at(ic);
          double dist=dot(p,corner1,normal);
          if (dist<0)
          {
            if (correct_direction)
            {
              ROS_FATAL("polygon is not convex, corners should be order in clockwise or anticlockwise way");
              throw("polygon is not convex");
            }
            else
            {
              correct_direction=true;
              normal.at(0)*=-1;
              normal.at(1)*=-1;
            }
          }
          else if (dist>0)
          {
            correct_direction=true;
          }
        }
      }
    }

    double getOverride(){return override_;}
    double dot(const std::vector<double>& p, const std::vector<double>& corner, const std::vector<double>& normal)
    {
      return (p.at(0)-corner.at(0))*normal.at(0)+(p.at(1)-corner.at(1))*normal.at(1);
    }

    bool inPolygon(const std::vector<double>& p)
    {
      for (unsigned int idx=0;idx<normals_.size();idx++)
      {
        const std::vector<double> n= normals_.at(idx);
        const std::vector<double> c= corners_.at(idx);

        if (dot(p,c,n)<0)
          return false;
      }
      return true;
    }

  };

  typedef std::shared_ptr<ConvexPolygon> ConvexPolygonPtr;

  class FixedAreasSSM
  {
  protected:
    ros::NodeHandle nh_;
    std::map<std::string,ConvexPolygonPtr> areas_;
    double override_;
    double target_override_;
    ros::Time last_time_;
    double override_increase_speed_=10;
    double override_decrease_speed_=10;
  public:
    FixedAreasSSM(const ros::NodeHandle& nh):
      nh_(nh)
    {
      override_=0;
      last_time_=ros::Time(0);
    }

    bool loadAreas()
    {

      if (!nh_.getParam("ssm/override_increase_speed",override_increase_speed_))
      {
        ROS_ERROR("Parameter ssm/override_increase_speed does not exist");
        return false;
      }
      if (!nh_.getParam("ssm/override_decrease_speed",override_decrease_speed_))
      {
        ROS_ERROR("Parameter ssm/override_decrease_speed does not exist");
        return false;
      }
      assert(override_increase_speed_>0);
      assert(override_decrease_speed_>0);
      XmlRpc::XmlRpcValue config;
      if (!nh_.getParam("ssm/areas",config))
      {
        ROS_ERROR("Parameter ssm/areas does not exist");
        return false;
      }

      if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
      {
        ROS_ERROR("The param is not a list of areas" );
        return false;
      }

      ROS_DEBUG_STREAM("Scan of feasible areas list :" << config.size()  );
      for(size_t i=0; i < config.size(); i++)
      {
        XmlRpc::XmlRpcValue area = config[i];
        if( area.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
          ROS_WARN("The element #%zu is not a struct", i);
          continue;
        }
        if( !area.hasMember("name") )
        {
          ROS_WARN("The element #%zu has not the field 'name'", i);
          continue;
        }
        if( !area.hasMember("override") )
        {
          ROS_WARN("The element #%zu has not the field 'override'", i);
          continue;
        }
        if( !area.hasMember("corners") )
        {
          ROS_WARN("The element #%zu has not the field 'corners'", i);
          continue;
        }

        if (area["corners"].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR("The param is not a list of 2Dcorners" );
          return false;
        }

        std::vector<std::vector<double>> corners;
        for(size_t i=0; i < area["corners"].size(); i++)
        {
          XmlRpc::XmlRpcValue corner = area["corners"][i];
          if (corner.getType() != XmlRpc::XmlRpcValue::TypeArray)
          {
            ROS_ERROR("The corner is not a vector" );
            return false;
          }

          if (corner.size()!=2)
          {
            ROS_ERROR("The corner is not a 2d vector" );
            return false;
          }

          std::vector<double> c(2);
          c.at(0)=rosparam_utilities::fromXmlRpcValue<double>(corner[0]);
          c.at(1)=rosparam_utilities::fromXmlRpcValue<double>(corner[1]);
          corners.push_back(c);
        }
        std::string name=area["name"];
        double ovr=rosparam_utilities::fromXmlRpcValue<double>(area["override"]);
        if (ovr<0 || ovr>100)
        {
          ROS_ERROR("override cannot be <0 or >100");
          return false;
        }
        ROS_DEBUG("creating a polygon named %s",name.c_str());
        ConvexPolygonPtr poly=std::make_shared<ConvexPolygon>(corners,ovr);

        areas_.insert(std::pair<std::string,ConvexPolygonPtr>(name,poly));

      }
      return true;
    }

    double checkArea(const std::vector<double>& p, double& override)
    {
      for (const std::pair<std::string,ConvexPolygonPtr>& area: areas_)
      {
        if (area.second->inPolygon(p))
        {
          override=std::min(override,area.second->getOverride());
        }
      }
    }

    void callback(const geometry_msgs::PoseArrayConstPtr& msg)
    {

      double override=100;
      for (const geometry_msgs::Pose& pose: msg->poses)
      {
        std::vector<double> p(2);
        p.at(0)=pose.position.x;
        p.at(1)=pose.position.y;
        checkArea(p,override);
      }
      target_override_=override;

    }

    double getOverride()
    {
      if (last_time_==ros::Time(0))
        last_time_=ros::Time::now();
      ros::Time t=ros::Time::now();
      double dt=(t-last_time_).toSec();
      last_time_=t;
      double diff_ovr=target_override_-override_;
      if (diff_ovr>dt*override_increase_speed_)
        diff_ovr=dt*override_increase_speed_;
      else if (diff_ovr<-dt*override_decrease_speed_)
        diff_ovr=-dt*override_decrease_speed_;

      override_+=diff_ovr;
      return override_;
    }

  };
}
