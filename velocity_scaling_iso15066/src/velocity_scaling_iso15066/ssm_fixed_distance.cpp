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


#include <velocity_scaling_iso15066/ssm_fixed_distance.h>

namespace ssm15066 {


void FixedDistanceSSM::checkDistanceFromPointCloud(double& speed_ovr,
                                                   const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>& Tbl)
{
  speed_ovr = 1.0;
  dist_from_closest_=std::numeric_limits<double>::infinity();

  std::vector<double> p(2);
  for (Eigen::Index idx=0; idx<human_points_in_b_.cols();idx++)
  {
    for (size_t il=0;il<Tbl_.size();il++)
    {
      //consider only links inside the poi_names_ list
      if(std::find(poi_names_.begin(),poi_names_.end(),links_names_[il])>=poi_names_.end())
        continue;

      p.at(0)=human_points_in_b_(0,idx) - Tbl_.at(il).translation()(0);
      p.at(1)=human_points_in_b_(1,idx) - Tbl_.at(il).translation()(1);
      double distance_human_to_link_squared = std::pow(p.at(0),2.0) + std::pow(p.at(1),2.0);

      if (distance_human_to_link_squared < dist_from_closest_)
      {
        dist_from_closest_=distance_human_to_link_squared;
      }
      checkArea(p,speed_ovr);
    }
  }
  dist_from_closest_=std::sqrt(dist_from_closest_);
}

FixedDistanceSSM::FixedDistanceSSM(){}

FixedDistanceSSM::FixedDistanceSSM(const rdyn::ChainPtr& chain):
  FixedAreasSSM(chain){}

void FixedDistanceSSM::init()
{
  is_configured_=true;
}

double FixedDistanceSSM::computeScaling(const Eigen::VectorXd& q,
                                        const Eigen::VectorXd& dq)
{
  if (!this->isConfigured())
  {
   std::cout << "[ssm15066] [WARNING] trying to compute scaling before using init()." << std::endl;
  }

  if (human_points_in_b_.cols()==0)
  {
    dist_from_closest_=std::numeric_limits<double>::infinity();
    return 1.0;
  }

  Tbl_=chain_->getTransformations(q);

  double ovr=1.0;
  checkDistanceFromPointCloud(ovr, Tbl_);
  return ovr;
}

}  // end ssm15066
