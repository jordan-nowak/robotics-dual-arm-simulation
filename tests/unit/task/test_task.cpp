#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "task/task.hpp"
#include "core/robot.hpp"

namespace robot::task::test {

// ============================================================================
// 0. TEST FIXTURE
// ============================================================================

/// @brief Mock implementation of Task for testing the base class behavior
class MockTask : public Task {
public:
    MockTask(const std::string& _name, int _priority, const Eigen::VectorXd& _error)
        : Task(_name, _priority), m_error(_error) {}

    Eigen::VectorXd computeError(const core::Robot& _robot) const override {
        return m_error;
    }

    Eigen::MatrixXd getJacobian(const core::Robot& _robot) const override {
        return Eigen::MatrixXd::Identity(m_error.size(), m_error.size());
    }

    int getDimension() const override {
        return static_cast<int>(m_error.size());
    }

    void setError(const Eigen::VectorXd& _error) { m_error = _error; }

private:
    Eigen::VectorXd m_error;
};

/// @brief Base class for shared setup
class TaskTestBase {
    protected:
        std::unique_ptr<robot::core::Robot> m_robot;

        void SetUpConfig() {
            robot::core::RobotConfig config = {
                "test_robot",
                std::vector<robot::core::JointConfig>{
                    { .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}
                },
                std::vector<robot::core::LinkConfig>{
                    { .name = "Link_0", .a = 0, .alpha = 0, .d = 0, .theta = 0, .mass = 1.0, .com = Eigen::Vector3d(0, 0, 0), .ID_link = 0, .ID_parent = -1, .motor = true}
                }
            };
            
            m_robot = std::make_unique<robot::core::Robot>(config);
        }
};

class TaskTest : public TaskTestBase, 
                 public ::testing::Test {
    protected:
        void SetUp() override {
            SetUpConfig();
        }
};

// ============================================================================
// 1. CONSTRUCTOR & INITIALIZATION
// ============================================================================

/// @brief Construct and verify that base class stores parameters correctly
TEST_F(TaskTest, Constructor_InitializesCorrectly) {
    MockTask task("TestTask", 5, Eigen::VectorXd::Zero(3));
    
    EXPECT_EQ(task.getName(), "TestTask");
    EXPECT_EQ(task.getPriority(), 5);
}

// ============================================================================
// 2. NORMAL CASES OF USE
// ============================================================================

/// @brief Verify that the the class set correctly the new name
TEST_F(TaskTest, SetName_UpdatesNameValue) {
    MockTask task("Initial", 0, Eigen::VectorXd::Zero(3));
    
    task.setName("Updated");
    EXPECT_EQ(task.getName(), "Updated");
}

/// @brief Verify that the the class set correctly the new priority
TEST_F(TaskTest, SetPriority_UpdatesPriorityValue) {
    MockTask task("Test", 1, Eigen::VectorXd::Zero(3));
    
    task.setPriority(10);
    EXPECT_EQ(task.getPriority(), 10);
}

/// @brief Verify polymorphism works: the base class calls derived method
TEST_F(TaskTest, IsAchieved_CallsComputeError) {
    Eigen::VectorXd error(3);
    error << 0.3, 0.4, 0.0;  // norm = 0.5
    
    MockTask task("Test", 0, error);
    
    EXPECT_FALSE(task.isAchieved(*m_robot, 0.4));  // 0.5 > 0.4
    EXPECT_TRUE(task.isAchieved(*m_robot, 0.6));   // 0.5 < 0.6
}

// ============================================================================
// 3. SPECIAL CASES
// ============================================================================

/// @brief Verify that the norm is strictly unequal to the accuracy.
TEST_F(TaskTest, IsAchieved_StrictInequality) {
    double precision = 0.5;
    Eigen::VectorXd error(1);
    error << precision;  // norm exactly equals precision
    
    MockTask task("Test", 0, error);
    
    EXPECT_FALSE(task.isAchieved(*m_robot, precision));
}

// ============================================================
// 4. PARAMETERIZED TESTS
// ============================================================

struct DimensionTestCase {
    int dimension;
    double component_value;  // Value for each component
    double precision_pass;   // Should achieve
    double precision_fail;   // Should NOT achieve
    std::string description;
};

class TaskDimensionTest : public TaskTestBase,
                          public ::testing::TestWithParam<DimensionTestCase> {
    protected:
        void SetUp() override {
            SetUpConfig();
        }
};

/// @brief Verify that isAchieved() works for any error dimension
TEST_P(TaskDimensionTest, IsAchieved_VariousDimensions) {
    auto testCase = GetParam();
    
    // Create error vector with specified dimension and values
    Eigen::VectorXd error = Eigen::VectorXd::Constant(
        testCase.dimension, 
        testCase.component_value
    );
    double actual_norm = error.norm();
    
    MockTask task("Test", 0, error);
    
    EXPECT_TRUE(task.isAchieved(*m_robot, testCase.precision_pass))
        << "Failed for " << testCase.description
        << " (norm = " << actual_norm << ")";
    
    EXPECT_FALSE(task.isAchieved(*m_robot, testCase.precision_fail))
        << "Failed for " << testCase.description
        << " (norm = " << actual_norm << ")";
}

INSTANTIATE_TEST_SUITE_P(
    AllDimensions,
    TaskDimensionTest,
    ::testing::Values(
        DimensionTestCase{1, 0.01, 0.02, 0.005, "Scalar (1D)"},
        DimensionTestCase{3, 0.3, 0.6, 0.4, "Position (3D)"},
        DimensionTestCase{6, 0.1, 0.3, 0.2, "Position and orientation (6D)"},
        DimensionTestCase{12, 0.05, 0.2, 0.1, "High-dimensional (12D)"}
    )
);

} // namespace robot::task::test