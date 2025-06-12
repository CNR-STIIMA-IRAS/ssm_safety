#include <gtest/gtest.h>
#include <velocity_scaling_iso15066/ssm15066.h>
#include "pinocchio/parsers/urdf.hpp"
#include <random>
#include <chrono>

TEST(SSM15066Test, ComputeScalingExecutionTime)
{
  std::string urdf_filename = std::string(TEST_DIR) + "/ur10.urdf";
  std::shared_ptr<pinocchio::Model> model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdf_filename, *model);
  std::shared_ptr<pinocchio::Data> data = std::make_shared<pinocchio::Data>(*model);

  ssm15066::DeterministicSSM ssm(model, data);

  ssm.setDefaultHumanSpeed(1.6);
  ssm.setReactionTime(0.1);
  ssm.setMaxCartesianAcceleration(5.0);
  ssm.setMinProtectiveDistance(0.1);
  ssm.setFilteringSelfDistance(0.05);
  ssm.useMeasuredHumanVelocity(false);
  ssm.init();

  int np = 10;
  double ub = 2, lb = 0.5;
  double step = (ub - lb) / (np - 1);
  int num_points = static_cast<int>(std::pow(np, 3));
  Eigen::Matrix<double, 3, Eigen::Dynamic> pc_in_b(3, num_points);
  Eigen::Matrix<double, 3, Eigen::Dynamic> human_velocities_in_b(3, num_points);
  Eigen::VectorXd occupancy(num_points);


  human_velocities_in_b.setZero();

  int idx = 0;
  for (int ix = 0; ix < np; ix++) {
    for (int iy = 0; iy < np; iy++) {
      for (int iz = 0; iz < np; iz++) {
        pc_in_b(0, idx) = lb + step * ix;
        pc_in_b(1, idx) = lb + step * iy;
        pc_in_b(2, idx) = lb + step * iz;
        ++idx;
      }
    }
  }

  occupancy.setConstant(0.5);
  ssm.setPointCloud(pc_in_b, human_velocities_in_b);

  Eigen::VectorXd q(model->nq), dq(model->nq);
  q.setRandom();
  dq.setRandom();

  double scaling;
  double speed_up = 10.0; // test with a reasonable speed factor

  auto start_time = std::chrono::high_resolution_clock::now();
  scaling = ssm.computeScaling(q, speed_up * dq);
  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end_time - start_time;

  std::cout << "Scaling factor: " << scaling << std::endl;
  std::cout << "Execution time: " << elapsed.count() << " ms" << std::endl;

  EXPECT_LT(elapsed.count(), 10.0) << "Scaling computation took too long!";
}
