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

namespace ssm15066
{

  bool
  ssm_safe_velocity_limits (const double &vr, const double &vh, const double &a,
                            const double &Tr, const double &D, const double &C,
                            double &solution1, double &solution2)
  {
    // D+Sh-Sr>C
    // D distance
    // Sr robot travel space during slow down (positive in the direction
    // robot->human) Sh human travel space during slow down (positive in the
    // direction robot->human) slow-down time = vr/a, for vr>=0  <---- Sr =
    // (vr*Tr)+Vr^2/a/2 Sh = vh*(Tr+Vr/a)  if human acceleration is zero
    // slow-down time = -Vr/a, for vr<0  <----
    // Sr = (vr*Tr)-Vr^2/a/2
    // Sh = vh*(Tr-Vr/a)  if human acceleration is zero
    // vrlimit =
    // Positive velocity
    // equation: D - Tr*vr + vh*(Tr + vr/a) - vr**2/(2*a)-C>0
    // solution1: -Tr*a + vh - sqrt(2.0*(D-C)*a + Tr**2*a**2 + vh**2)
    // solution2: -Tr*a + vh + sqrt(2.0*(D-C)*a + Tr**2*a**2 + vh**2)
    // solution1 < vr < solution2
    // Negative velocity
    // equation: D - Tr*vr + vh*(Tr - vr/a) + 0.5*vr**2/a-C>0
    // solution1: Tr*a + vh - sqrt(-2.0*(D-C)*a + Tr**2*a**2 + vh**2)
    // solution2: Tr*a + vh + sqrt(-2.0*(D-C)*a + Tr**2*a**2 + vh**2)
    // vr<solution1  OR  vr>solution2

    if (D < C)
    {
      solution1 = 0.0;
      solution2 = 0.0;
      return false;
    }

    if (vr >= 0)
    {
      double sqrt_discriminant = std::sqrt (
                                   2.0 * (D - C) * a + std::pow (Tr * a, 2.0) + std::pow (vh, 2.0));
      solution1 = -Tr * a + vh + sqrt_discriminant;
      solution2 = -Tr * a + vh - sqrt_discriminant;
      // solution2 < vr < solution1 => always choose solution1
      return true;
    }
    else // robot velocity is negative
    {
      double discriminant
          = -2.0 * (D - C) * a + std::pow (Tr * a, 2.0) + std::pow (vh, 2.0);
      if (discriminant > 0.0)
      {
        double sqrt_discriminant = std::sqrt (discriminant);
        solution1 = Tr * a + vh + sqrt_discriminant;
        solution2 = Tr * a + vh - sqrt_discriminant;
        // vr<solution2 OR vr>solution1 with  solution2 < solution1
        // if solution1<vr => any velocity > solution1 is good => choose vr
        // elif solution1>vr AND solution2<vr => any velocity up to solution2
        // is good => choose solution2 elif solution1>vr AND solution2>vr =>
        // any velocity up to solution2 is good => choose solution2
        return true;
      }
      else // the distance is high, all the negative robot velocities are good
        // => choose current vr
      {
        solution1 = vr;
        solution2 = vr;
        return true;
      }
    }
  }

  void DeterministicSSM::computeKinematics(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
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

  void DeterministicSSM::setLinkId()
  {
    links_idx_.clear();
    for (const auto &name: links_names_)
    {
      links_idx_.push_back (model_->getFrameId (name));
    }
  }

  DeterministicSSM::DeterministicSSM (
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

  void
  DeterministicSSM::init ()
  {

    dist_dec_ = max_cart_acc_ * t_r_;
    term2_ = dist_dec_;
    term1_ = std::pow (dist_dec_, 2) - 2 * max_cart_acc_ * min_distance_;

    if (measured_velocities_)
    {
      term1_ += std::pow (default_human_velocity_, 2);
      term2_ += default_human_velocity_;
    }
    else
    {
      // CHECK ??
      human_velocities_in_b_.setConstant (default_human_velocity_);
    }

    is_configured_ = true;
  }

  void
  DeterministicSSM::setMaxCartesianAcceleration (const double &acc)
  {
    max_cart_acc_ = acc;
    is_configured_ = false;
  }

  void
  DeterministicSSM::setReactionTime (const double &t_r)
  {
    t_r_ = t_r;
    is_configured_ = false;
  }

  void
  DeterministicSSM::setDefaultHumanSpeed (const double &vel)
  {
    default_human_velocity_ = vel;
    is_configured_ = false;
  }

  void
  DeterministicSSM::setMinProtectiveDistance (const double &dist)
  {
    min_distance_ = dist;
    is_configured_ = false;
  }

  void
  DeterministicSSM::setFilteringSelfDistance (const double &dist)
  {
    self_distance_ = dist;
    is_configured_ = false;
  }

  void
  DeterministicSSM::useMeasuredHumanVelocity (const bool &flag)
  {
    measured_velocities_ = flag;
    is_configured_ = false;
  }

  void
  DeterministicSSM::setCheckedRobotLinks (const std::vector<std::string> &links)
  {
    links_names_ = links;
    setLinkId();
  }

  double
  DeterministicSSM::computeScaling (const Eigen::VectorXd &q,
                                    const Eigen::VectorXd &dq)
  {
    if (!this->isConfigured ())
    {
      std::cout << "[ssm15066] [WARNING] trying to compute scaling before "
                   "using init()."
                << std::endl;
    }

    if (human_points_in_b_.cols () == 0)
    {
      dist_from_closest_ = std::numeric_limits<double>::infinity ();
      return 1.0;
    }

    computeKinematics(q,dq);
    s_ref_ = 1.0;
    dist_from_closest_ = std::numeric_limits<double>::infinity ();
    for (Eigen::Index ic = 0; ic < human_points_in_b_.cols (); ic++)
    {
      for (size_t il = 0; il < Tbl_.size (); il++)
      {

        d_lc_in_b_
            = human_points_in_b_.col (ic) - Tbl_.at (il).translation ();
        distance_ = d_lc_in_b_.norm ();
        if (distance_ < self_distance_)
          continue;
        robot_tangential_speed_
            = ((vl_in_b_.at (il).head (3)).dot (d_lc_in_b_)) / distance_;
        if (measured_velocities_)
          human_tangential_speed_
              = ((human_velocities_in_b_.col (ic)).dot (d_lc_in_b_))
                / distance_;
        else
          human_tangential_speed_ = -default_human_velocity_;
        if (distance_ > min_distance_)
        {
          double solution1, solution2;
          if (ssm_safe_velocity_limits (
                robot_tangential_speed_, human_tangential_speed_,
                max_cart_acc_, t_r_, distance_, min_distance_, solution1,
                solution2))
          {
            if (robot_tangential_speed_ >= 0.0)
            {
              //  solution2 < vr < solution1
              vmax_ = std::max (0.0, solution1);
            }
            else //  couldn't we just set vmax_=robot_tangential_speed_
              //  because robot is moving away?
            {
              // vr<solution1 OR vr>solution2
              if (solution1 <= robot_tangential_speed_)
              {
                vmax_ = robot_tangential_speed_;
              }
              else
              {
                vmax_ = std::max (0.0, solution2);
              }
            }
          }
          else
          {
            vmax_ = 0.0;
          }
        }

        if (robot_tangential_speed_ == 0.0)
          s_ref_lc_ = 1.0;
        else
          s_ref_lc_ = vmax_ / robot_tangential_speed_; // no division by 0

        if (distance_ < dist_from_closest_)
          dist_from_closest_ = distance_;
        if (s_ref_lc_ < s_ref_) // saturate to 1
          s_ref_ = s_ref_lc_;
      }
    }
    return s_ref_;
  }


  void
  ProbabilisticSSM::setPointCloud (
      const Eigen::Matrix<double, 3, Eigen::Dynamic> &human_points_in_b,
      const Eigen::Matrix<double, 3, Eigen::Dynamic> &human_velocities_in_b,
      const Eigen::VectorXd &occupancy)
  {
    assert (human_points_in_b.cols () == occupancy.rows ());
    DeterministicSSM::setPointCloud (human_points_in_b, human_velocities_in_b);
    occupancy_ = occupancy;
  }

  double
  ProbabilisticSSM::computeScaling (const Eigen::VectorXd &q,
                                    const Eigen::VectorXd &dq)
  {
    if (!this->isConfigured ())
    {
      std::cout << "[ssm15066] [WARNING] trying to compute scaling before "
                   "using init()."
                << std::endl;
    }

    if (human_points_in_b_.cols () == 0)
    {
      dist_from_closest_ = std::numeric_limits<double>::infinity ();
      return 1.0;
    }

    scaling_.clear ();
    computeKinematics(q,dq);

    for (Eigen::Index ic = 0; ic < human_points_in_b_.cols (); ic++)
    {
      if (occupancy_ (ic) <= occupancy_min_)
        continue;
      double s_ref_c = 1;
      dist_from_closest_ = std::numeric_limits<double>::infinity ();
      for (size_t il = 0; il < Tbl_.size (); il++)
      {

        d_lc_in_b_
            = human_points_in_b_.col (ic) - Tbl_.at (il).translation ();
        distance_ = d_lc_in_b_.norm ();
        if (distance_ < dist_from_closest_)
          dist_from_closest_ = distance_;
        robot_tangential_speed_
            = ((vl_in_b_.at (il).head (3) - human_velocities_in_b_.col (ic))
               .dot (d_lc_in_b_))
              / distance_;
        human_tangential_speed_
            = ((human_velocities_in_b_.col (ic)).dot (d_lc_in_b_))
              / distance_;
        if (distance_ > min_distance_)
        {
          double solution1, solution2;
          if (ssm_safe_velocity_limits (
                robot_tangential_speed_, human_tangential_speed_,
                max_cart_acc_, t_r_, distance_, min_distance_, solution1,
                solution2))
          {
            if (robot_tangential_speed_ >= 0.0)
            {
              //  solution2 < vr < solution1
              vmax_ = std::max (0.0, solution1);
            }
            else
            {
              // vr<solution1 OR vr>solution2
              if (solution1 <= robot_tangential_speed_)
              {
                vmax_ = robot_tangential_speed_;
              }
              else
              {
                vmax_ = std::max (0.0, solution2);
              }
            }
          }
          else
          {
            vmax_ = 0.0;
          }
        }
        else
        {
          s_ref_lc_ = 0.0;
        }

        if (robot_tangential_speed_ == 0.0)
          s_ref_lc_ = 1.0;
        else
          s_ref_lc_ = vmax_ / robot_tangential_speed_; // no division by 0

        if (distance_ < dist_from_closest_)
          dist_from_closest_ = distance_;
        if (s_ref_lc_ < s_ref_c)
          s_ref_c = s_ref_lc_;
      }
      scaling_.insert (std::pair<double, double> (s_ref_c, occupancy_ (ic)));
    }

    s_ref_ = 0.0;
    double previous_probability = 1;
    for (const std::pair<double, double> &p : scaling_)
    {
      // p.first  = scaling
      // p.second = occupancy probability
      s_ref_ += p.first * p.second * previous_probability;
      previous_probability *= (1 - p.second);
    }
    s_ref_ += previous_probability;
    return s_ref_;
  }

} // end ssm15066
