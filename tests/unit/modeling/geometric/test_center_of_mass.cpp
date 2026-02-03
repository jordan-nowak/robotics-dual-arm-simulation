#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "modeling/geometric/center_of_mass.hpp"
#include "core/robot_config.hpp"

using namespace robot;
using namespace robot::modeling::geometric;

/**
 * @brief Test fixture for CenterOfMass
 */
class CenterOfMassTest : public ::testing::Test {
protected:
    std::unique_ptr<robot::core::RobotConfig> configtest;
    std::unique_ptr<std::vector<double>> q;

    void SetUp() override {
        /*
            Robot with:
            - two links
            - mass: 2kg and 1kg
            - CoM in local link frames: [1,0,0], [0,1,0]
        */

        configtest = std::make_unique<robot::core::RobotConfig>(
            "test_robot",
            std::vector<robot::core::JointConfig>{
                { .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}, 
                { .name = "Joint_1", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}
            },
            std::vector<robot::core::LinkConfig>{
                { .name = "Link_0", .a = 0, .alpha = 0, .d = 0, .theta = 0, .mass = 2.0, .com = Eigen::Vector3d(1.0, 0.0, 0.0), .ID_link = 0, .ID_parent = -1, .motor = true}, 
                { .name = "Link_1", .a = 0, .alpha = 0, .d = 0, .theta = 0, .mass = 1.0, .com = Eigen::Vector3d(0.0, 1.0, 0.0), .ID_link = 1, .ID_parent = 0, .motor = true}
            }
        );

        q = std::make_unique<std::vector<double>>(std::vector<double>{0.0f, 0.0f});
    }
};

TEST_F(CenterOfMassTest, TotalMass)
{
    CenterOfMass com_global(*configtest);

    EXPECT_DOUBLE_EQ(com_global.getTotalMass(), 3.0);
}

TEST_F(CenterOfMassTest, GlobalCenterOfMass)
{
    CenterOfMass com_global(*configtest);

    Eigen::Vector3d com = com_global.getGlobalCoM(*q);

    /*
        CoM total =
        ( mass_1 *   CoM_1 + mass_2 *   CoM_2 ) / Mass_total
        (      2 * [1,0,0] +      1 * [0,1,0] ) / 3
        = [2/3, 1/3, 0]
    */

    Eigen::Vector3d expected(2.0 / 3.0, 1.0 / 3.0, 0.0);

    EXPECT_TRUE(com.isApprox(expected, 1e-6));
}
