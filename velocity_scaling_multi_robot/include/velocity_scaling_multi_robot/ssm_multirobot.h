/*
Copyright (c) 2020, Marco Faroni
CARI Joint Research Lab
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

#pragma once

#include <velocity_scaling_iso15066/ssm15066.h>
#include <robot_state_controller_msgs/PoseTwistArray.h>
#include <robot_state_controller_msgs/PoseTwist.h>

//TODO: modify computeScaling to consider also twist information
// poi creare nodo di scaling da lanciare

namespace ssm15066 {

class MultiRobotSSM : public DeterministicSSM
{
protected:
  Eigen::Matrix<double,3,Eigen::Dynamic> vc_in_b_;
  double vc_on_lc_=0;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MultiRobotSSM(const rosdyn::ChainPtr& chain, const ros::NodeHandle nh = ros::NodeHandle("~"));
  void setPointCloudTwist(const Eigen::Matrix<double, 3, Eigen::Dynamic>& pc_in_b){vc_in_b_=pc_in_b;}
  double computeScaling(const Eigen::VectorXd& q,
                        const Eigen::VectorXd& dq);
};

typedef shared_ptr_namespace::shared_ptr< MultiRobotSSM > MultiRobotSSMPtr;

inline MultiRobotSSM::MultiRobotSSM(const rosdyn::ChainPtr& chain, const ros::NodeHandle nh):
  DeterministicSSM::DeterministicSSM(chain, nh)
{
}

inline double MultiRobotSSM::computeScaling(const Eigen::VectorXd& q,
                                            const Eigen::VectorXd& dq)
{
  assert(pc_in_b_.cols()==vc_in_b_.cols());

  if (pc_in_b_.cols()==0)
    return 1.0;

  Tbl_=chain_->getTransformations(q);

  vl_in_b_=chain_->getTwist(q,dq);

  s_ref_=1.0;
  dist_from_closest_=std::numeric_limits<double>::infinity();
  for (Eigen::Index ic=0;ic<pc_in_b_.cols();ic++)
  {
    for (size_t il=0;il<Tbl_.size();il++)
    {
      d_lc_in_b_=pc_in_b_.col(ic)-Tbl_.at(il).translation();
      distance_=d_lc_in_b_.norm();
      if (distance_<self_distance_)
        continue;
      vc_on_lc_ = vc_in_b_.col(ic).dot(d_lc_in_b_)/distance_;
      tangential_speed_=vl_in_b_.at(il).block(0,0,3,1).dot(d_lc_in_b_)/distance_ - vc_on_lc_;
      if (tangential_speed_<=0)  // robot is going away
      {
        s_ref_lc_=1.0;
      }
      else if (distance_>min_distance_)
      {
        vmax_=std::sqrt(term1_+2.0*max_cart_acc_*distance_ + vc_on_lc_*vc_on_lc_) -dist_dec_ - vc_on_lc_;
        s_ref_lc_=vmax_/tangential_speed_;  // no division by 0
      }
      else  //distance<=min_distance
      {
        return 0.0;
      }
      if (distance_<dist_from_closest_)
        dist_from_closest_=distance_;
      if (s_ref_lc_<s_ref_)
        s_ref_=s_ref_lc_;
    }
  }
  return s_ref_;
}


}  // end ssm15066
