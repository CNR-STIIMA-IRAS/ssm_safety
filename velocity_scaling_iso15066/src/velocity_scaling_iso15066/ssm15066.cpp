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


#include <velocity_scaling_iso15066/ssm15066.h>

namespace ssm15066 {

//DeterministicSSM::DeterministicSSM(const rosdyn::ChainPtr& chain1)
//{
//  chain_=chain1;
//  Eigen::VectorXd velocity_limits=chain_->getDQMax();
//  inv_velocity_limits_=velocity_limits.cwiseInverse();

//  dist_dec_=max_cart_acc_*t_r_;
//  term1_=std::pow(dist_dec_,2)-2*max_cart_acc_*min_distance_;
//}

//double DeterministicSSM::computeScaling(const Eigen::VectorXd& q,
//                                        const Eigen::VectorXd& dq)
//{

//  Tbl_=chain_->getTransformations(q);
//  vl_in_b_=chain_->getTwist(q,dq);

//  s_ref_=1.0;
//  for (size_t ic=0;ic<pc_in_b_.cols();ic++)
//  {
//    for (size_t il=0;il<Tbl_.size();il++)
//    {
//      d_lc_in_b_=Tbl_.at(il).translation()-pc_in_b_.col(ic);
//      distance_=d_lc_in_b_.norm();

//      if (distance_>min_distance_)
//      {
//        tangential_speed_=((vl_in_b_.at(il).block(0,0,3,1)).dot(d_lc_in_b_))/distance_;
//        if (tangential_speed_<=0)  // robot is going away
//        {
//          s_ref_lc_=1.0;
//        }
//        else
//        {
//          //vmax=std::sqrt(std::pow(max_cart_acc*t_r,2)+2*max_cart_acc*(distance-C))-max_cart_acc*t_r;
//          vmax_=std::sqrt(term1_+2.0*max_cart_acc_*distance_)-dist_dec_;
//          s_ref_lc_=vmax_/tangential_speed_;  // no division by 0
//        }
//      }
//      else  //distance<=min_distance
//      {
//        //s_ref_lc=0;
//        return 0.0;
//      }
//      if (s_ref_lc_<s_ref_)
//        s_ref_=s_ref_lc_;
//    }
//  }
//  return s_ref_;
//}


//void ProbabilisticSSM::setPointCloud(const Eigen::Matrix<double, 3, Eigen::Dynamic> &pc_in_b1, const Eigen::VectorXd &occupancy)
//{
//  assert(pc_in_b1.cols()==occupancy.rows());
//  pc_in_b_=pc_in_b1;
//  occupancy_=occupancy;
//}

//double ProbabilisticSSM::computeScaling(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
//{

//  scaling_.clear();
//  Tbl_=chain_->getTransformations(q);
//  vl_in_b_=chain_->getTwist(q,dq);

//  for (size_t ic=0;ic<pc_in_b_.cols();ic++)
//  {
//    if (occupancy_(ic)<=occupancy_min_)
//      continue;
//    double s_ref_c=1;
//    for (size_t il=0;il<Tbl_.size();il++)
//    {
//      d_lc_in_b_=Tbl_.at(il).translation()-pc_in_b_.col(ic);
//      distance_=d_lc_in_b_.norm();

//      if (distance_>min_distance_)
//      {
//        tangential_speed_=((vl_in_b_.at(il).block(0,0,3,1)).dot(d_lc_in_b_))/distance_;
//        if (tangential_speed_<=0)  // robot is going away
//        {
//          s_ref_lc_=1.0;
//        }
//        else
//        {
//          //vmax=std::sqrt(std::pow(max_cart_acc*t_r,2)+2*max_cart_acc*(distance-C))-max_cart_acc*t_r;
//          vmax_=std::sqrt(term1_+2.0*max_cart_acc_*distance_)-dist_dec_;
//          s_ref_lc_=vmax_/tangential_speed_;  // no division by 0
//        }
//      }
//      else  //distance<=min_distance
//      {
//        s_ref_c=0;
//        break;
//      }
//      if (s_ref_lc_<s_ref_c)
//        s_ref_c=s_ref_lc_;
//    }
//    scaling_.insert(std::pair<double,double>(s_ref_c,occupancy_(ic)));
//  }

//  s_ref_=0.0;
//  double previous_probability=1;
//  for (const std::pair<double,double>& p: scaling_)
//  {
//    // p.first  = scaling
//    // p.second = occupancy probability
//    s_ref_+=p.first*p.second*previous_probability;
//    previous_probability*=(1-p.second);
//  }
//  s_ref_+=previous_probability;
//  return s_ref_;
//}

}  // end ssm15066
