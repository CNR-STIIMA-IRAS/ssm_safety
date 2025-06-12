#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <memory>
#include "velocity_scaling_iso15066/ssm_fixed_areas.h"

using namespace ssm15066;

class FixedAreasSSMTest : public ::testing::Test {
protected:
    FixedAreasSSM ssm;

    void SetUp() override {
        ssm.init();
    }

    void setHumanPoints3D(const std::vector<Eigen::Vector3d>& points) {
        Eigen::Matrix<double, 3, Eigen::Dynamic> cloud(3, points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            cloud.col(i) = points[i];
        }

        Eigen::Matrix<double, 3, Eigen::Dynamic> dummy_velocities = cloud;
        ssm.setPointCloud(cloud, dummy_velocities);  // uses BaseSSM::setPointCloud
    }
};

TEST_F(FixedAreasSSMTest, NoPointsMeansFullSpeed) {
    setHumanPoints3D({});
    EXPECT_DOUBLE_EQ(ssm.computeScaling(Eigen::VectorXd(), Eigen::VectorXd()), 1.0);
}

TEST_F(FixedAreasSSMTest, PointOutsideAnyArea) {
    ssm.addArea("circle1", 1.0, 0.3);  // radius 1.0
    setHumanPoints3D({Eigen::Vector3d(5.0, 5.0, 0.0)});
    EXPECT_DOUBLE_EQ(ssm.computeScaling(Eigen::VectorXd(), Eigen::VectorXd()), 1.0);
}

TEST_F(FixedAreasSSMTest, PointInsideCircleArea) {
    ssm.addArea("circle1", 2.0, 0.6);
    setHumanPoints3D({Eigen::Vector3d(1.0, 1.0, 0.0)});
    EXPECT_DOUBLE_EQ(ssm.computeScaling(Eigen::VectorXd(), Eigen::VectorXd()), 0.6);
}

TEST_F(FixedAreasSSMTest, PointInsidePolygonArea) {
    std::vector<std::vector<double>> square = {
        {0.0, 0.0}, {0.0, 2.0}, {2.0, 2.0}, {2.0, 0.0}
    };
    ssm.addArea("poly1", square, 0.4);
    setHumanPoints3D({Eigen::Vector3d(1.0, 1.0, 0.0)});
    EXPECT_DOUBLE_EQ(ssm.computeScaling(Eigen::VectorXd(), Eigen::VectorXd()), 0.4);
}

TEST_F(FixedAreasSSMTest, PointInMultipleAreasChoosesLowest) {
    ssm.addArea("circle", 3.0, 0.8);
    std::vector<std::vector<double>> square = {
        {0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}
    };
    ssm.addArea("polygon", square, 0.3);
    setHumanPoints3D({Eigen::Vector3d(0.5, 0.5, 0.0)});
    EXPECT_DOUBLE_EQ(ssm.computeScaling(Eigen::VectorXd(), Eigen::VectorXd()), 0.3);
}
