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

#include <rosdyn_core/primitives.h>

namespace ssm15066 {

class DeterministicSSM
{
protected:
  rosdyn::ChainPtr chain_;

  Eigen::VectorXd inv_velocity_limits_;


  double self_distance_=0.2;
  double min_distance_=0.3  ; // min distance
  double max_cart_acc_=0.1;  // m/s^2
  double t_r_=0.15;  // reaction time;
  double dist_dec_;
  double term1_;
  double distance_;
  double s_ref_lc_;
  double s_ref_;
  double tangential_speed_;
  double vmax_;
  double dist_from_closest_;

  Eigen::Vector3d d_lc_in_b_;

  Eigen::Matrix<double,3,Eigen::Dynamic> pc_in_b_;

  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> Tbl_;
  std::vector< Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d> > vl_in_b_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DeterministicSSM(const rosdyn::ChainPtr& chain, const ros::NodeHandle nh = ros::NodeHandle("~"));
  void setPointCloud(const Eigen::Matrix<double, 3, Eigen::Dynamic>& pc_in_b){pc_in_b_=pc_in_b;}
  double computeScaling(const Eigen::VectorXd& q,
                        const Eigen::VectorXd& dq);
  double getDistanceFromClosestPoint();
};

class ProbabilisticSSM: public DeterministicSSM
{
  Eigen::VectorXd occupancy_;
  std::map<double,double> scaling_;
  double occupancy_min_=0.0;
public:
  ProbabilisticSSM(const rosdyn::ChainPtr& chain, const ros::NodeHandle nh=ros::NodeHandle("~")): DeterministicSSM(chain,nh){}
  void setPointCloud(const Eigen::Matrix<double, 3, Eigen::Dynamic>& pc_in_b1,
                     const Eigen::VectorXd& occupancy);
  double computeScaling(const Eigen::VectorXd& q,
                        const Eigen::VectorXd& dq);
  double getDistanceFromClosestPoint();

};


typedef shared_ptr_namespace::shared_ptr< DeterministicSSM   > DeterministicSSMPtr;
typedef shared_ptr_namespace::shared_ptr< ProbabilisticSSM   > ProbabilisticSSMPtr;

inline DeterministicSSM::DeterministicSSM(const rosdyn::ChainPtr& chain, const ros::NodeHandle nh)
{
  chain_=chain;
  Eigen::VectorXd velocity_limits=chain_->getDQMax();
  inv_velocity_limits_=velocity_limits.cwiseInverse();

  min_distance_=nh.param("minimum_distance",0.3);
  self_distance_=nh.param("self_distance",0.0);
  max_cart_acc_=nh.param("maximum_cartesian_acceleration",0.1);
  t_r_=nh.param("reaction_time",0.15);

  ROS_INFO("[%s]: Minimum distance               =  %f",nh.getNamespace().c_str(),min_distance_);
  ROS_INFO("[%s]: Maximum Cartesian acceleration =  %f",nh.getNamespace().c_str(),max_cart_acc_);
  ROS_INFO("[%s]: reaction time                  =  %f",nh.getNamespace().c_str(),t_r_);

  dist_dec_=max_cart_acc_*t_r_;
  term1_=std::pow(dist_dec_,2)-2*max_cart_acc_*min_distance_;
}

inline double DeterministicSSM::computeScaling(const Eigen::VectorXd& q,
                                        const Eigen::VectorXd& dq)
{
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
      tangential_speed_=((vl_in_b_.at(il).block(0,0,3,1)).dot(d_lc_in_b_))/distance_;
      if (tangential_speed_<=0)  // robot is going away
      {
        s_ref_lc_=1.0;
      }
      else if (distance_>min_distance_)
      {
        vmax_=std::sqrt(term1_+2.0*max_cart_acc_*distance_)-dist_dec_;
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

inline double DeterministicSSM::getDistanceFromClosestPoint()
{
  return dist_from_closest_;
}

inline void ProbabilisticSSM::setPointCloud(const Eigen::Matrix<double, 3, Eigen::Dynamic> &pc_in_b1, const Eigen::VectorXd &occupancy)
{
  assert(pc_in_b1.cols()==occupancy.rows());
  pc_in_b_=pc_in_b1;
  occupancy_=occupancy;
}

inline double ProbabilisticSSM::computeScaling(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
{
  if (pc_in_b_.cols()==0)
    return 1.0;
  scaling_.clear();
  Tbl_=chain_->getTransformations(q);
  vl_in_b_=chain_->getTwist(q,dq);

  for (size_t ic=0;ic<pc_in_b_.cols();ic++)
  {
    if (occupancy_(ic)<=occupancy_min_)
      continue;
    double s_ref_c=1;
    dist_from_closest_=std::numeric_limits<double>::infinity();
    for (size_t il=0;il<Tbl_.size();il++)
    {
      d_lc_in_b_=pc_in_b_.col(ic)-Tbl_.at(il).translation();
      distance_=d_lc_in_b_.norm();
      if (distance_<dist_from_closest_)
      	dist_from_closest_=distance_;
      tangential_speed_=((vl_in_b_.at(il).block(0,0,3,1)).dot(d_lc_in_b_))/distance_;
      if (tangential_speed_<=0)  // robot is going away
      {
        s_ref_lc_=1.0;
      }
      else if (distance_>min_distance_)
      {
          vmax_=std::sqrt(term1_+2.0*max_cart_acc_*distance_)-dist_dec_;
          s_ref_lc_=vmax_/tangential_speed_;  // no division by 0
      }
      else  //distance<=min_distance
      {
        s_ref_c=0;
        break;
      }
      if (s_ref_lc_<s_ref_c)
        s_ref_c=s_ref_lc_;
    }
    scaling_.insert(std::pair<double,double>(s_ref_c,occupancy_(ic)));
  }

  s_ref_=0.0;
  double previous_probability=1;
  for (const std::pair<double,double>& p: scaling_)
  {
    // p.first  = scaling
    // p.second = occupancy probability
    s_ref_+=p.first*p.second*previous_probability;
    previous_probability*=(1-p.second);
  }
  s_ref_+=previous_probability;
  return s_ref_;
}

inline double ProbabilisticSSM::getDistanceFromClosestPoint()
{
  return dist_from_closest_;
}


}  // end ssm15066
