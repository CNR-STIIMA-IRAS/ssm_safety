/*
Copyright (c) 2024, Manuel Beschi, Marco Faroni
CARI Joint Research Lab
Politecnico di Milano
UNIBS-DIMI manuel.beschi@unibs.it
CNR-STIIMA manuel.beschi@stiima.cnr.it
POLIMI marco.faroni@polimi.it
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

#pragma once

#include "velocity_scaling_iso15066/ssm_base.h"
#include <map>

namespace ssm15066 {

class Shape
{
protected:
  double override_;

public:
  Shape(const double& override);

  double getOverride();

  virtual bool checkArea(const std::vector<double>& p)=0;

};
using ShapePtr = std::shared_ptr<Shape>;

class Circle : public Shape
{
protected:
  double radius_;

public:
  Circle(const double& radius, const double& override);

  bool checkArea(const std::vector<double>& p);

};

using CirclePtr = std::shared_ptr<Circle>;

class ConvexPolygon : public Shape
{
protected:
  std::vector<std::vector<double>> corners_;
  std::vector<std::vector<double>> normals_;

  double dot(const std::vector<double>& p, const std::vector<double>& corner, const std::vector<double>& normal);

  bool inPolygon(const std::vector<double>& p);

public:
  ConvexPolygon(const std::vector<std::vector<double>>& corners,
                const double& override);

  bool checkArea(const std::vector<double>& p);

};

using ConvexPolygonPtr = std::shared_ptr<ConvexPolygon>;


class FixedAreasSSM : public BaseSSM
{
protected:
  std::map<std::string,ShapePtr> areas_;
  // ros::Time last_time_;

  void checkArea(const std::vector<double>& p, double& speed_ovr); // DEPRECATED

  void checkAreaFromPointCloud(double& speed_ovr); // DEPRECATED

  void checkArea(const std::vector<double>& p, std::string& occupied_area);

  void checkAreaFromPointCloud(std::string& occupied_area);


  bool activate_on_human_{false};
  bool activate_on_robot_{false};
  bool activate_on_signal_{false};

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FixedAreasSSM();

  FixedAreasSSM(const rdyn::ChainPtr& chain);

  void init() override;

  void init(bool activate_on_h, bool activate_on_r, bool activate_on_s);

  double computeScaling(const Eigen::VectorXd& q,
                        const Eigen::VectorXd& dq) override;

  void addArea(const std::string& name, const std::vector<std::vector<double>>& corners, const double& speed_ovr);

  void addArea(const std::string& name, const double& radius, const double& speed_ovr);

  void printAreas();


};

using FixedAreasSSMPtr = std::shared_ptr< FixedAreasSSM >;




}  // end ssm15066
