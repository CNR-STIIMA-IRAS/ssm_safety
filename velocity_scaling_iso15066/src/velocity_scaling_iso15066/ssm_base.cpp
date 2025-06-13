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


#include <velocity_scaling_iso15066/ssm_base.h>

namespace ssm15066 {

BaseSSM::BaseSSM(){}

BaseSSM::BaseSSM(
     const std::shared_ptr<pinocchio::Model> model,
     std::shared_ptr<pinocchio::Data> data)
   : model_ (model), data_ (data)
{
  links_names_.clear ();

  for (const auto &frame : model_->frames)
  {
    if (frame.type == pinocchio::BODY)
    {
      links_names_.push_back (frame.name);
      links_idx_.push_back (model_->getFrameId (frame.name));
    }
  }
  Tbl_.resize (links_names_.size ());
  vl_in_b_.resize (links_names_.size ());
}

void BaseSSM::init(){}

bool BaseSSM::isConfigured()
{
  return is_configured_;
}


void BaseSSM::setLinkId()
{
  links_idx_.clear();
  for (const auto &name: links_names_)
  {
    links_idx_.push_back (model_->getFrameId (name));
  }
}

void BaseSSM::useMeasuredHumanVelocity (const bool &flag)
{
  measured_velocities_ = flag;
  is_configured_ = false;
}


void BaseSSM::computeKinematics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
{


  Eigen::VectorXd a = Eigen::VectorXd::Zero (model_->nv);

  // Computes the kinematics derivatives for all the joints of the robot
  pinocchio::forwardKinematics (*model_, *data_, q, dq, a);
  pinocchio::computeForwardKinematicsDerivatives (*model_, *data_, q, dq, a);
  pinocchio::updateFramePlacements (*model_, *data_);

  pinocchio::Motion twist;
  Eigen::Vector6d v6;

  for (size_t i = 0; i < links_idx_.size (); i++)
  {
    const pinocchio::FrameIndex &idx = links_idx_[i];
    const auto &oMf = data_->oMf[idx]; // Transformation: world → frame

    Eigen::Affine3d T (oMf.toHomogeneousMatrix ()); // also validlinks_idx_
    Tbl_.at(i)=T;
    twist = pinocchio::getFrameVelocity (*model_, *data_, idx,
                                         pinocchio::LOCAL_WORLD_ALIGNED);
    v6 << twist.linear (), twist.angular (); // linear first, then angular
    vl_in_b_.at(i)=v6;
  }

}



void BaseSSM::setPointCloud(const Eigen::Matrix<double, 3, Eigen::Dynamic>& human_points_in_b,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& human_velocities_in_b)
{
  human_points_in_b_=human_points_in_b;
  if (measured_velocities_)
  {
    if (human_velocities_in_b.cols()!=human_points_in_b.cols())
    {
      throw std::invalid_argument("human points and velocities do not match");
    }
    else
    {
      human_velocities_in_b_=human_velocities_in_b;
    }
  }
}

double BaseSSM::getDistanceFromClosestPoint()
{
  return dist_from_closest_;
}

std::vector<std::string> BaseSSM::getPoiNames()
{
  return links_names_;
}

void
BaseSSM::setCheckedRobotLinks (const std::vector<std::string> &links)
{
  links_names_ = links;
  setLinkId();
}

}

// end ssm15066
