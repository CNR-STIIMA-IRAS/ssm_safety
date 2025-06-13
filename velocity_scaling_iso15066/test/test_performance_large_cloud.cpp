#include <gtest/gtest.h>
#include <velocity_scaling_iso15066/ssm15066.h>
#include <pinocchio/parsers/urdf.hpp>
#include <chrono>

template <typename T>
void benchmarkLargeCloud(T& ssm, const pinocchio::Model& model)
{
    int N = 100000;
    Eigen::Matrix<double, 3, Eigen::Dynamic> pc(3, N), vel(3, N);
    pc.setRandom(); vel.setZero();
    ssm.setPointCloud(pc, vel);

    Eigen::VectorXd q(model.nq), dq(model.nq);
    q.setRandom(); dq.setRandom();

    auto start = std::chrono::high_resolution_clock::now();
    double scaling = ssm.computeScaling(q, dq);
    auto end = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << "Execution time: " << ms << " ms" << std::endl;
    EXPECT_LT(ms, 50.0);
}

TEST(SSM15066Test, LargePointCloudPerformanceDeterministic)
{
    std::string urdf_filename = std::string(TEST_DIR) + "/ur10.urdf";
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_filename, *model);
    auto data = std::make_shared<pinocchio::Data>(*model);
    ssm15066::DeterministicSSM ssm(model, data);
    ssm.init();

    benchmarkLargeCloud(ssm, *model);
}
