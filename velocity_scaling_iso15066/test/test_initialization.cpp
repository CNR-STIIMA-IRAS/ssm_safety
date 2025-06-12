#include <gtest/gtest.h>
#include <velocity_scaling_iso15066/ssm15066.h>
#include <pinocchio/parsers/urdf.hpp>

TEST(SSM15066Test, InitializationDeterministic)
{
    std::string urdf_filename = std::string(TEST_DIR) + "/ur10.urdf";
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_filename, *model);
    auto data = std::make_shared<pinocchio::Data>(*model);

    ssm15066::DeterministicSSM ssm(model, data);
    EXPECT_NO_THROW(ssm.init());
}

TEST(SSM15066Test, InitializationProbabilistic)
{
    std::string urdf_filename = std::string(TEST_DIR) + "/ur10.urdf";
    auto model = std::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_filename, *model);
    auto data = std::make_shared<pinocchio::Data>(*model);

    ssm15066::ProbabilisticSSM ssm(model, data);
    EXPECT_NO_THROW(ssm.init());
}
