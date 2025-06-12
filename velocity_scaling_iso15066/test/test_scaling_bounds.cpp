#include <gtest/gtest.h>
#include <velocity_scaling_iso15066/ssm15066.h>
#include <pinocchio/parsers/urdf.hpp>

void test_scaling_bounds(ssm15066::BaseSSM& ssm, const pinocchio::Model& model)
{
    Eigen::Matrix<double, 3, Eigen::Dynamic> pc(3, 1), vel(3, 1);
    pc.setZero(); vel.setZero();
    ssm.setPointCloud(pc, vel);

    Eigen::VectorXd q(model.nq), dq(model.nq);
    q.setRandom(); dq.setRandom();

    double scaling = ssm.computeScaling(q, dq);
    EXPECT_GE(scaling, 0.0);
    EXPECT_LE(scaling, 1.0);
}

TEST(SSM15066Test, ScalingInBoundsDeterministic)
{
    std::string urdf_filename = std::string(TEST_DIR) + "/ur10.urdf";
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_filename, *model);
    auto data = std::make_shared<pinocchio::Data>(*model);
    ssm15066::DeterministicSSM ssm(model, data);
    ssm.init();

    test_scaling_bounds(ssm, *model);
}
