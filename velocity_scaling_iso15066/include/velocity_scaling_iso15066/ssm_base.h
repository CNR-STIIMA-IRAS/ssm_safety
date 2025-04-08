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

#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <cstdint>
#include <assert.h>
#include <memory>
#include <iostream>

# include <Eigen/Geometry>
# include <Eigen/StdVector>

namespace ssm15066 {


class BaseSSM
{
protected:

  bool is_configured_=false;
  bool measured_velocities_=false;
  //double s_ref_lc_;
  //double s_ref_;
  double dist_from_closest_=-1.0;
  
  Eigen::Matrix<double,3,Eigen::Dynamic> human_points_in_b_;
  Eigen::Matrix<double,3,Eigen::Dynamic> human_velocities_in_b_;
  Eigen::Vector2d robot_position_in_b_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BaseSSM();

  virtual void init();

  bool isConfigured();

  void setPointCloud(const Eigen::Matrix<double, 3, Eigen::Dynamic>& human_points_in_b,
                     const Eigen::Matrix<double, 3, Eigen::Dynamic>& human_velocities_in_b);

  virtual double computeScaling(const Eigen::VectorXd& q,
                                const Eigen::VectorXd& dq) = 0;

  virtual double getDistanceFromClosestPoint();

};

using BaseSSMPtr = std::shared_ptr< BaseSSM >;

}  // end ssm15066
