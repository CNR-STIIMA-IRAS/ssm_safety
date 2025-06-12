/*
Copyright (c) 2020, Manuel Beschi 
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
#include "pinocchio/parsers/urdf.hpp"
#include <random>


int main(int argc, char **argv)
{
  std::string urdf_filename = std::string(TEST_DIR) + "/ur10.urdf";
  std::cout << "file_path = " << urdf_filename << std::endl;

  std::shared_ptr<pinocchio::Model> model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdf_filename,*model);
  std::cout << "model name: " << model->name << std::endl;


  // Create data required by the algorithms
  std::shared_ptr<pinocchio::Data> data = std::make_shared<pinocchio::Data>(*model);

  ssm15066::DeterministicSSM ssm(model,data);

  double C=0.01; // min distance
  double max_cart_acc=1;  // m/s^2
  double t_r=0.1;  // reaction time;


  ssm.setDefaultHumanSpeed(1.6);
  ssm.setReactionTime(0.1);
  ssm.setMaxCartesianAcceleration(5.0);
  ssm.setMinProtectiveDistance(0.1);
  ssm.setFilteringSelfDistance(0.05);
  ssm.useMeasuredHumanVelocity(false);
  //ssm.setCheckedRobotLinks(links);
  ssm.init();


  int np=10; //
  double ub=2;
  double lb=0.5;

  Eigen::Matrix<double,3,Eigen::Dynamic> pc_in_b;
  pc_in_b.resize(3,std::pow(np,3));

  Eigen::Matrix<double,3,Eigen::Dynamic> human_velocities_in_b;
  human_velocities_in_b.resize(3,std::pow(np,3));
  human_velocities_in_b.setZero();


  double x,y,z;
  x=y=z=lb;
  double step=(ub-lb)/(np-1);
  int idx=0;
  for (int ix=0;ix<np;ix++)
  {
    for (int iy=0;iy<np;iy++)
    {
      for (int iz=0;iz<np;iz++)
      {
        pc_in_b(0,idx)=lb+step*ix;
        pc_in_b(1,idx)=lb+step*iy;
        pc_in_b(2,idx)=lb+step*iz;
        idx++;
      }
    }
  }

  Eigen::VectorXd occupancy;
  occupancy.resize(std::pow(np,3));
  std::mt19937 gen;
  std::uniform_real_distribution<double> dist(0.0,1.0);
  for (size_t ic=0;ic<occupancy.size();ic++)
    occupancy(ic)=dist(gen);

  occupancy.setConstant(0.5);

  ssm.setPointCloud(pc_in_b,human_velocities_in_b);

  Eigen::VectorXd q(model->nq);
  Eigen::VectorXd dq(model->nq);

  for (size_t idx=0;idx<10;idx++)
  {
    q.setRandom();
    dq.setRandom();
    double scaling;
    double distance;

    scaling = ssm.computeScaling(q,dq);
    distance = ssm.getDistanceFromClosestPoint();
    std::cout << "scaling = " << scaling << " distance  = " << distance << std::endl;

    dq*=10.0;
    scaling = ssm.computeScaling(q,dq);
    distance = ssm.getDistanceFromClosestPoint();
    std::cout << "scaling = " << scaling << " distance  = " << distance << std::endl;

    dq*=10.0;
    scaling = ssm.computeScaling(q,dq);
    distance = ssm.getDistanceFromClosestPoint();
    std::cout << "scaling = " << scaling << " distance  = " << distance << std::endl;
  }
  // double dist_dec=max_cart_acc*t_r;
  // double term1=std::pow(dist_dec,2)-2*max_cart_acc*C;

  // unsigned int ntrials=100000;

  // ros::WallTime t0,t1;
  // double t_obj=0;
  //  double k_limit;
  // for (unsigned int it=0;it<ntrials;it++)
  // {
  //   q.setRandom();
  //   u.setRandom();
  //   u.normalize();

  //   t0=ros::WallTime::now();
  //   k_limit=1./(inv_velocity_limits.cwiseProduct(u.cwiseAbs()).minCoeff());  // no division by 0
  //   u*=k_limit;
  //   ssm.computeScaling(q,u);
  //   t1=ros::WallTime::now();
  //   t_obj+=(t1-t0).toSec()*1e3;
  // }

  // double t_main=0;
  // for (unsigned int it=0;it<ntrials;it++)
  // {
  //   q.setRandom();
  //   u.setRandom();
  //   u.normalize();

  //   t0=ros::WallTime::now();
  //   k_limit=1./(inv_velocity_limits.cwiseProduct(u.cwiseAbs()).minCoeff());  // no division by 0
  //   u*=k_limit;

  //   std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> Tbl=chain->getTransformations(q);
  //   std::vector< Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d> > vl_in_b=chain->getTwist(q,u);

  //   double s_ref=1;
  //   for (size_t ic=0;ic<pc_in_b.cols();ic++)
  //   {
  //     for (size_t il=0;il<Tbl.size();il++)
  //     {
  //       if(std::find(poi_names.begin(),poi_names.end(),link_names[il])>=poi_names.end())
  //         continue;

  //       Eigen::Vector3d d_lc_in_b=Tbl.at(il).translation()-pc_in_b.col(ic);
  //       double distance=d_lc_in_b.norm();
  //       double s_ref_lc;
  //       if (distance>C)
  //       {
  //         double tangential_speed=((vl_in_b.at(il).block(0,0,3,1)).dot(d_lc_in_b))/distance;
  //         if (tangential_speed<=0)  // robot is going away
  //         {
  //           s_ref_lc=1;
  //         }
  //         else
  //         {
  //           //vmax=std::sqrt(std::pow(max_cart_acc*t_r,2)+2*max_cart_acc*(distance-C))-max_cart_acc*t_r;
  //           double vmax=std::sqrt(term1+2*max_cart_acc*distance)-dist_dec;
  //           s_ref_lc=vmax/tangential_speed;  // no division by 0
  //         }
  //       }
  //       else  //distance<=min_distance
  //       {
  //         s_ref_lc=0;
  //       }
  //       if (s_ref_lc<s_ref)
  //         s_ref=s_ref_lc;
  //     }
  //   }
  //   t1=ros::WallTime::now();
  //   t_main+=(t1-t0).toSec()*1e3;
  // }


  // ssm15066::ProbabilisticSSM pssm(chain);
  // pssm.setPointCloud(pc_in_b,occupancy);

  // double t_prob=0;
  // for (unsigned int it=0;it<ntrials;it++)
  // {
  //   q.setRandom();
  //   u.setRandom();
  //   u.normalize();

  //   ros::WallTime t0=ros::WallTime::now();
  //   double k_limit=1./(inv_velocity_limits.cwiseProduct(u.cwiseAbs()).minCoeff());  // no division by 0
  //   u*=k_limit;
  //   double s_ref2=pssm.computeScaling(q,u);
  //   ros::WallTime t1=ros::WallTime::now();
  //   t_prob+=(t1-t0).toSec()*1e3;
  //   double s_ref=ssm.computeScaling(q,u);
  //   ROS_INFO_THROTTLE(0.3,"s det = %f, prob = %f",s_ref,s_ref2);
  // }


  // ROS_INFO("iteration number=%u, compute in main = %f [ms], compute in obj = %f [ms], probabilist = %f [ms]",ntrials,t_main/ntrials,t_obj/ntrials,t_prob/ntrials);
  return 0;


}
