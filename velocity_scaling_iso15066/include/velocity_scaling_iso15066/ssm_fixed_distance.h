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

#include "velocity_scaling_iso15066/ssm_fixed_areas.h"
#include <map>

namespace ssm15066 {

class FixedDistanceSSM : public FixedAreasSSM
{
protected:

  void checkDistanceFromPointCloud(double& speed_ovr, const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > &Tbl);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FixedDistanceSSM();

  FixedDistanceSSM(const rdyn::ChainPtr& chain);

  void init() override;

  double computeScaling(const Eigen::VectorXd& q,
                        const Eigen::VectorXd& dq) override;

};

using FixedDistanceSSMPtr = std::shared_ptr< FixedDistanceSSM >;




}  // end ssm15066
