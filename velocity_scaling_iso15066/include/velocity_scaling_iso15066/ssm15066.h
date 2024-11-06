/*
Copyright (c) 2020, Manuel Beschi 
CARI Joint Research Lab
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

#pragma once


#include <rdyn_core/primitives.h>

namespace ssm15066 {


bool ssm_safe_velocity_limits(const double& vr,
                      const double& vh,
                      const double& a,
                      const double& Tr,
                      const double &D,
                      const double& C,
                      double& solution1,
                      double& solution2);


class DeterministicSSM
{
protected:
  rdyn::ChainPtr chain_;

  Eigen::VectorXd inv_velocity_limits_;

  std::vector<std::string> links_names_;
  std::vector<std::string> poi_names_;  // list of point of interests to consider along the robot structure

  bool is_configured_=false;
  bool measured_velocities_=false;
  double self_distance_=0.15; // filter out points too close to the robot (likely false positive)
  double min_distance_=0.3  ; // min distance
  double max_cart_acc_=0.1;  // m/s^2
  double t_r_=0.15;  // reaction time;
  double dist_dec_;
  double term1_;
  double term2_;
  double distance_;
  double s_ref_lc_;
  double s_ref_;
  double robot_tangential_speed_;
  double human_tangential_speed_;
  double vmax_;
  double default_human_velocity_{0.0};
  double dist_from_closest_;

  Eigen::Vector3d d_lc_in_b_;

  Eigen::Matrix<double,3,Eigen::Dynamic> human_points_in_b_;
  Eigen::Matrix<double,3,Eigen::Dynamic> human_velocities_in_b_;

  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> Tbl_;
  std::vector< Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d> > vl_in_b_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DeterministicSSM(const rdyn::ChainPtr& chain);

  void init();

  void setMaxCartesianAcceleration(const double& acc);

  void setReactionTime(const double& t_r);

  void setDefaultHumanSpeed(const double& vel);

  void setMinProtectiveDistance(const double& dist);

  void setFilteringSelfDistance(const double& dist);

  void useMeasuredHumanVelocity(const bool& flag);

  void setCheckedRobotLinks(const std::vector<std::string>& links);

  bool isConfigured();

  void setPointCloud(const Eigen::Matrix<double, 3, Eigen::Dynamic>& human_points_in_b,
                     const Eigen::Matrix<double, 3, Eigen::Dynamic>& human_velocities_in_b);
  double computeScaling(const Eigen::VectorXd& q,
                        const Eigen::VectorXd& dq);
  double getDistanceFromClosestPoint();

  std::vector<std::string> getPoiNames()
  {
    return poi_names_;
  }
};

class ProbabilisticSSM: public DeterministicSSM
{
  Eigen::VectorXd occupancy_;
  std::map<double,double> scaling_;
  double occupancy_min_=0.0;
public:
  ProbabilisticSSM(const rdyn::ChainPtr& chain): DeterministicSSM(chain){}
  void setPointCloud(const Eigen::Matrix<double, 3, Eigen::Dynamic>& human_points_in_b,
                     const Eigen::Matrix<double, 3, Eigen::Dynamic>& human_velocities_in_b,
                     const Eigen::VectorXd& occupancy);
  double computeScaling(const Eigen::VectorXd& q,
                        const Eigen::VectorXd& dq);

};


typedef std::shared_ptr< DeterministicSSM   > DeterministicSSMPtr;
typedef std::shared_ptr< ProbabilisticSSM   > ProbabilisticSSMPtr;




}  // end ssm15066
