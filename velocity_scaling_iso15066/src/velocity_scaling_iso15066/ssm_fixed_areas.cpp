/*
Copyright (c) 2024, Marco Faroni, Manuel Beschi
CARI Joint Research Lab
UNIBS-DIMI manuel.beschi@unibs.it
CNR-STIIMA manuel.beschi@stiima.cnr.it
Politecnico di Milano marco.faroni@polimi.it
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


#include <velocity_scaling_iso15066/ssm_fixed_areas.h>

namespace ssm15066 {

Shape::Shape(const double& override):
  override_(override){}

double Shape::getOverride(){return override_;}

Circle::Circle(const double& radius, const double& override):
  Shape(override), radius_(radius)
{
  //std::cout << "loading circular area with maximum override= << " override_ << ", with radius " << radius_ << std::endl;
  if (radius_<=0)
  {
    //std::cout << "radius must be positive" << std::endl;
    assert(radius_>0);
  }
}


bool Circle::checkArea(const std::vector<double>& p)
{
  return (p[0]*p[0] + p[1]*p[1] <= radius_*radius_);
}

ConvexPolygon::ConvexPolygon(const std::vector<std::vector<double>>& corners,
                             const double& override):
  Shape(override), corners_(corners)
{
  // ROS_DEBUG("loading area with maximum override=%f, with the following corners\n", override_);
  normals_.resize(corners_.size());
  for (const std::vector<double>& c: corners_)
  {
    if (c.size()!=2)
    {
      //ROS_FATAL("corners dimension should be 2");
      assert(c.size()==2);
    }
    //ROS_DEBUG_STREAM(c.at(0) << ", " << c.at(1));
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
          //ROS_FATAL("polygon is not convex, corners should be order in clockwise or anticlockwise way");
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

double ConvexPolygon::dot(const std::vector<double>& p, const std::vector<double>& corner, const std::vector<double>& normal)
{
  return (p.at(0)-corner.at(0))*normal.at(0)+(p.at(1)-corner.at(1))*normal.at(1);
}

bool ConvexPolygon::inPolygon(const std::vector<double>& p)
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

bool ConvexPolygon::checkArea(const std::vector<double>& p)
{
  return inPolygon(p);
}


FixedAreasSSM::FixedAreasSSM(){}

FixedAreasSSM::FixedAreasSSM(const rdyn::ChainPtr& chain):
  BaseSSM(chain){}

void FixedAreasSSM::addArea(const std::string& name, const std::vector<std::vector<double>>& corners, const double& speed_ovr)
{
  ShapePtr area=std::make_shared<ConvexPolygon>(corners,speed_ovr);
  areas_.insert(std::pair<std::string,ShapePtr>(name,area));
}

void FixedAreasSSM::addArea(const std::string& name, const double& radius, const double& speed_ovr)
{
  ShapePtr area=std::make_shared<Circle>(radius,speed_ovr);
  areas_.insert(std::pair<std::string,ShapePtr>(name,area));
}

void FixedAreasSSM::checkArea(const std::vector<double>& p, std::string& occupied_area)
{
  double min_ovr = 1.0;
  for (const std::pair<std::string,ShapePtr>& area: areas_)
  {
    if (area.second->checkArea(p))
    {
      if (area.second->getOverride() < min_ovr)
      {
        min_ovr = area.second->getOverride();
        occupied_area = area.first;
      }
    }
  }
}

void FixedAreasSSM::checkAreaFromPointCloud(std::string& occupied_area)
{
  for (size_t idx=0; idx<human_points_in_b_.cols();idx++)
  {
    std::vector<double> p(2);
    p.at(0)=human_points_in_b_(0,idx);
    p.at(1)=human_points_in_b_(1,idx);
    checkArea(p,occupied_area);
  }
}


/* DEPRECATED */
void FixedAreasSSM::checkArea(const std::vector<double>& p, double& speed_ovr)
{
  for (const std::pair<std::string,ShapePtr>& area: areas_)
  {
    if (area.second->checkArea(p))
    {
      speed_ovr=std::min(speed_ovr,area.second->getOverride());
    }
  }
}

/* DEPRECATED */
void FixedAreasSSM::checkAreaFromPointCloud(double& speed_ovr)
{
  speed_ovr = 1.0;
  for (size_t idx=0; idx<human_points_in_b_.cols();idx++)
  {
    std::vector<double> p(2);
    p.at(0)=human_points_in_b_(0,idx);
    p.at(1)=human_points_in_b_(1,idx);
    checkArea(p,speed_ovr);
  }
}

void FixedAreasSSM::init()
{
  is_configured_=true;
}

void FixedAreasSSM::init(bool activate_on_h, bool activate_on_r, bool activate_on_s)
{
  activate_on_human_ = activate_on_h;
  activate_on_robot_ = activate_on_r;
  activate_on_signal_ = activate_on_s;

  if ( (activate_on_human_ || activate_on_robot_ || activate_on_signal_) == false)
  {
    std::cerr << "At least one between activate_on_human, activate_on_robot, and activate_on_signal should be set to true for the fixed areas to work."
              << std::endl;
  }
  this->init();
}

void FixedAreasSSM::printAreas()
{
  for (const std::pair<std::string,ShapePtr>& area: areas_)
  {
    std::cout << "area name: " << area.first << ". area ovr: " << area.second->getOverride() << std::endl;
  }
}



double FixedAreasSSM::computeScaling(const Eigen::VectorXd& q,
                                     const Eigen::VectorXd& dq)
{
  if (!this->isConfigured())
  {
   std::cout << "[ssm15066] [WARNING] trying to compute scaling before using init()." << std::endl;
  }


  std::string area_h;
  std::string area_r;
  std::vector<std::string> areas_s;

  if (activate_on_human_)
  {
    if (human_points_in_b_.cols()==0)
    {
      dist_from_closest_=std::numeric_limits<double>::infinity();
      return 1.0;
    }
    checkAreaFromPointCloud(area_h);

    if (area_h.empty())
    {
      return 1.0;
    }
  }
  if (activate_on_robot_)
  {
    //checkAreaFromRobot(area_r);
    if (area_r.empty())
    {
      return 1.0;
    }
  }
  if (activate_on_signal_)
  {
    //checkAreaFromSignal(areas_s);
    if (areas_s.size()==0)
    {
      return 1.0;
    }
  }

  if (activate_on_human_ && activate_on_robot_ && activate_on_signal_)
  {
    for (const auto& area_s: areas_s)
    {
      if (area_s.compare(area_r)==0 && area_s.compare(area_h)==0)
      {
        return areas_.find(area_s)->second->getOverride();
      }
    }
    return 1.0;
  }
  if (activate_on_human_ && activate_on_robot_)
  {
    if (area_r.compare(area_h)==0)
    {
      return areas_.find(area_r)->second->getOverride();
    }
    return 1.0;
  }
  if (activate_on_human_ && activate_on_signal_)
  {
    for (const auto& area_s: areas_s)
    {
      if (area_s.compare(area_h)==0)
      {
        return areas_.find(area_s)->second->getOverride();
      }
    }
    return 1.0;
  }
  if (activate_on_robot_ && activate_on_signal_)
  {
    for (const auto& area_s: areas_s)
    {
      if (area_s.compare(area_r)==0)
      {
        return areas_.find(area_s)->second->getOverride();
      }
    }
    return 1.0;
  }
  if (activate_on_robot_)
  {
    return areas_.find(area_r)->second->getOverride();
  }
  if (activate_on_human_)
  {
    return areas_.find(area_h)->second->getOverride();
  }
  if (activate_on_signal_)
  {
    double min_ovr=1.0;
    for (const auto& area_s: areas_s)
    {
      double ovr = areas_.find(area_s)->second->getOverride();
      if (ovr < min_ovr)
      {
        min_ovr = ovr;
      }
    }
    return min_ovr;
  }

  std::cerr << "you should not be here!!!" << std::endl;
  return 1.0;
}

}  // end ssm15066
