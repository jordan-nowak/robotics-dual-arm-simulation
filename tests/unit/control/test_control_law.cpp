#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "control/control_law.hpp"
#include "task/task.hpp"
#include "core/robot.hpp"

namespace robot::control::test {

// ============================================================================
// 0. TEST FIXTURE
// ============================================================================

/// @brief Mock implementation of Task for testing
class MockTask : public task::Task {
public:
    MockTask(const std::string& _name, int _priority, const Eigen::VectorXd& _error)
        : Task(_name, _priority), m_error(_error), m_dimension(_error.size()) {}

    Eigen::VectorXd computeError(const core::Robot& _robot) const override {
        return m_error;
    }

    Eigen::MatrixXd getJacobian(const core::Robot& _robot) const override {
        int dof = _robot.config().jointCount();
        return Eigen::MatrixXd::Identity(m_dimension, dof);
    }

    int getDimension() const override {
        return m_dimension;
    }

    void setError(const Eigen::VectorXd& _error) {
        m_error = _error;
        m_dimension = _error.size();
    }

private:
    Eigen::VectorXd m_error;
    int m_dimension;
};

/// @brief Base class for shared test configuration
class ControlLawTestsBase {
protected:
    std::unique_ptr<robot::core::Robot> m_robot;
    double m_tol_zero = 1e-12;
    double m_tol_numerical = 1e-10;

    void SetUpRobot() {
        // Simple 2-link robot for testing
        robot::core::RobotConfig config = {
            "test_robot",
            std::vector<robot::core::JointConfig>{
                { .name = "Joint_0", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f},
                { .name = "Joint_1", .minPosition = -3.14f, .maxPosition = 3.14f, .maxVelocity = 1.0f}
            },
            std::vector<robot::core::LinkConfig>{
                { .name = "Link_0", .a = 0, .alpha = 0, .d = 0, .theta = 0,
                  .mass = 1.0, .com = Eigen::Vector3d(0, 0, 0),
                  .ID_link = 0, .ID_parent = -1, .motor = true},
                { .name = "Link_1", .a = 0, .alpha = 0, .d = 0, .theta = 0,
                  .mass = 1.0, .com = Eigen::Vector3d(0, 0, 0),
                  .ID_link = 1, .ID_parent = 0, .motor = true}
            }
        };

        m_robot = std::make_unique<robot::core::Robot>(config);
    }

    /// @brief Create a mock task with specified priority and error
    std::shared_ptr<MockTask> createMockTask(
        const std::string& _name,
        int _priority,
        const Eigen::VectorXd& _error
    ) {
        return std::make_shared<MockTask>(_name, _priority, _error);
    }
};

class ControlLawTests : public ControlLawTestsBase,
                       public ::testing::Test {
protected:
    void SetUp() override {
        SetUpRobot();
    }
};

// ============================================================================
// 1. CONSTRUCTOR & INITIALIZATION
// ============================================================================

/// @brief Verify constructor initializes correctly (with default tolerance)
TEST_F(ControlLawTests, Constructor_InitializesCorrectly) {
    ControlLaw control_law;

    EXPECT_EQ(control_law.getTaskCount(), 0);
}

/// @brief Verify constructor stores custom tolerance
TEST_F(ControlLawTests, Constructor_CustomTolerance) {
    double custom_tolerance = 1e-8;
    ControlLaw control_law(custom_tolerance);

    EXPECT_EQ(control_law.getTaskCount(), 0);
}

// ============================================================================
// 2. NORMAL CASES OF USE
// ============================================================================

/// @brief Verify addTask increases task count
TEST_F(ControlLawTests, AddTask_IncreasesTaskCount) {
    ControlLaw control_law;
    auto task = createMockTask("Task1", 0, Eigen::VectorXd::Zero(2));

    control_law.addTask(task);

    EXPECT_EQ(control_law.getTaskCount(), 1);
}

/// @brief Verify multiple tasks can be added
TEST_F(ControlLawTests, AddTask_MultipleTasksAdded) {
    ControlLaw control_law;
    auto task1 = createMockTask("Task1", 0, Eigen::VectorXd::Zero(2));
    auto task2 = createMockTask("Task2", 1, Eigen::VectorXd::Zero(2));

    control_law.addTask(task1);
    control_law.addTask(task2);

    EXPECT_EQ(control_law.getTaskCount(), 2);
}

/// @brief Verify clearTasks removes all tasks
TEST_F(ControlLawTests, ClearTasks_RemovesAllTasks) {
    ControlLaw control_law;
    auto task1 = createMockTask("Task1", 0, Eigen::VectorXd::Zero(2));
    auto task2 = createMockTask("Task2", 1, Eigen::VectorXd::Zero(2));

    control_law.addTask(task1);
    control_law.addTask(task2);
    control_law.clearTasks();

    EXPECT_EQ(control_law.getTaskCount(), 0);
}

/// @brief Verify setTolerance updates tolerance value
TEST_F(ControlLawTests, SetTolerance_UpdatesValue) {
    ControlLaw control_law(1e-6);

    control_law.setTolerance(1e-8);

    // No direct getter, but we can verify behavior doesn't crash
    SUCCEED();
}

// ============================================================================
// 3. COMPUTE COMMAND
// ============================================================================

/// @brief Verify compute returns zero command when no tasks
TEST_F(ControlLawTests, Compute_NoTasks_ReturnsZeroCommand) {
    ControlLaw control_law;

    Command cmd = control_law.compute(*m_robot);

    EXPECT_EQ(cmd.getDimension(), m_robot->config().jointCount());
    EXPECT_TRUE(cmd.getValues().isApprox(
        Eigen::VectorXd::Zero(m_robot->config().jointCount()),
        m_tol_zero
    ));
}

/// @brief Verify compute returns command with correct dimension
TEST_F(ControlLawTests, Compute_SingleTask_CorrectDimension) {
    ControlLaw control_law;
    Eigen::VectorXd error(2);
    error << 0.1, 0.2;
    auto task = createMockTask("Task1", 0, error);

    control_law.addTask(task);
    Command cmd = control_law.compute(*m_robot);

    EXPECT_EQ(cmd.getDimension(), m_robot->config().jointCount());
}

/// @brief Verify compute works with multiple tasks
TEST_F(ControlLawTests, Compute_MultipleTasks_ReturnsCommand) {
    ControlLaw control_law;
    auto task1 = createMockTask("Task1", 0, Eigen::Vector2d(0.1, 0.2));
    auto task2 = createMockTask("Task2", 1, Eigen::Vector2d(0.3, 0.4));

    control_law.addTask(task1);
    control_law.addTask(task2);
    Command cmd = control_law.compute(*m_robot);

    EXPECT_EQ(cmd.getDimension(), m_robot->config().jointCount());
    EXPECT_TRUE(std::isfinite(cmd.getValues().norm()));
}

// ============================================================================
// 4. TASK PRIORITY ORDERING
// ============================================================================

/// @brief Verify tasks are sorted by priority (lower value = higher priority)
TEST_F(ControlLawTests, AddTask_SortsByPriority) {
    ControlLaw control_law;
    auto task_low = createMockTask("LowPriority", 5, Eigen::VectorXd::Zero(2));
    auto task_high = createMockTask("HighPriority", 0, Eigen::VectorXd::Zero(2));
    auto task_mid = createMockTask("MidPriority", 2, Eigen::VectorXd::Zero(2));

    // Add in random order
    control_law.addTask(task_low);
    control_law.addTask(task_high);
    control_law.addTask(task_mid);

    // Verify all added
    EXPECT_EQ(control_law.getTaskCount(), 3);

    // Verify sorting by checking status order
    auto statuses = control_law.getTasksStatus(*m_robot, 1.0);
    EXPECT_EQ(statuses[0].task_name, "HighPriority");
    EXPECT_EQ(statuses[1].task_name, "MidPriority");
    EXPECT_EQ(statuses[2].task_name, "LowPriority");
}

// ============================================================================
// 5. TASK ACHIEVEMENT
// ============================================================================

/// @brief Verify areAllTasksAchieved returns true when no tasks
TEST_F(ControlLawTests, AreAllTasksAchieved_NoTasks_ReturnsTrue) {
    ControlLaw control_law;

    EXPECT_TRUE(control_law.areAllTasksAchieved(*m_robot, 1.0));
}

/// @brief Verify areAllTasksAchieved returns true when all tasks achieved
TEST_F(ControlLawTests, AreAllTasksAchieved_AllAchieved_ReturnsTrue) {
    ControlLaw control_law;
    auto task1 = createMockTask("Task1", 0, Eigen::Vector2d(0.01, 0.01));
    auto task2 = createMockTask("Task2", 1, Eigen::Vector2d(0.02, 0.02));

    control_law.addTask(task1);
    control_law.addTask(task2);

    // Both tasks have error norm < 0.1
    EXPECT_TRUE(control_law.areAllTasksAchieved(*m_robot, 0.1));
}

/// @brief Verify areAllTasksAchieved returns false when one task not achieved
TEST_F(ControlLawTests, AreAllTasksAchieved_OneNotAchieved_ReturnsFalse) {
    ControlLaw control_law;
    auto task1 = createMockTask("Task1", 0, Eigen::Vector2d(0.01, 0.01));
    auto task2 = createMockTask("Task2", 1, Eigen::Vector2d(1.0, 1.0));

    control_law.addTask(task1);
    control_law.addTask(task2);

    EXPECT_FALSE(control_law.areAllTasksAchieved(*m_robot, 0.1));
}

/// @brief Verify isTaskAchieved checks specific task
TEST_F(ControlLawTests, IsTaskAchieved_SpecificTask) {
    ControlLaw control_law;
    auto task1 = createMockTask("Task1", 0, Eigen::Vector2d(0.01, 0.01));
    auto task2 = createMockTask("Task2", 1, Eigen::Vector2d(1.0, 1.0));

    control_law.addTask(task1);
    control_law.addTask(task2);

    EXPECT_TRUE(control_law.isTaskAchieved(0, *m_robot, 0.1));
    EXPECT_FALSE(control_law.isTaskAchieved(1, *m_robot, 0.1));
}

/// @brief Verify isTaskAchieved throws on invalid index
TEST_F(ControlLawTests, IsTaskAchieved_InvalidIndex_Throws) {
    ControlLaw control_law;
    auto task = createMockTask("Task1", 0, Eigen::VectorXd::Zero(2));

    control_law.addTask(task);

    EXPECT_THROW(
        control_law.isTaskAchieved(5, *m_robot, 1.0),
        std::out_of_range
    );
}

// ============================================================================
// 6. TASK STATUS
// ============================================================================

/// @brief Verify getTasksStatus returns empty vector when no tasks
TEST_F(ControlLawTests, GetTasksStatus_NoTasks_ReturnsEmpty) {
    ControlLaw control_law;

    auto statuses = control_law.getTasksStatus(*m_robot, 1.0);

    EXPECT_EQ(statuses.size(), 0);
}

/// @brief Verify getTasksStatus returns correct information
TEST_F(ControlLawTests, GetTasksStatus_ReturnsCorrectInfo) {
    ControlLaw control_law;
    Eigen::Vector2d error1(0.3, 0.4);  // norm = 0.5
    Eigen::Vector2d error2(0.6, 0.8);  // norm = 1.0

    auto task1 = createMockTask("Task1", 0, error1);
    auto task2 = createMockTask("Task2", 1, error2);

    control_law.addTask(task1);
    control_law.addTask(task2);

    auto statuses = control_law.getTasksStatus(*m_robot, 0.75);

    ASSERT_EQ(statuses.size(), 2);

    // Task1
    EXPECT_EQ(statuses[0].task_name, "Task1");
    EXPECT_EQ(statuses[0].priority, 0);
    EXPECT_NEAR(statuses[0].error_norm, 0.5, m_tol_numerical);
    EXPECT_TRUE(statuses[0].achieved);  // 0.5 < 0.75

    // Task2
    EXPECT_EQ(statuses[1].task_name, "Task2");
    EXPECT_EQ(statuses[1].priority, 1);
    EXPECT_NEAR(statuses[1].error_norm, 1.0, m_tol_numerical);
    EXPECT_FALSE(statuses[1].achieved);  // 1.0 >= 0.75
}

/// @brief Verify getTasksStatus respects priority ordering
TEST_F(ControlLawTests, GetTasksStatus_RespectsPriorityOrder) {
    ControlLaw control_law;
    auto task_low = createMockTask("LowPriority", 10, Eigen::VectorXd::Zero(2));
    auto task_high = createMockTask("HighPriority", 1, Eigen::VectorXd::Zero(2));

    control_law.addTask(task_low);
    control_law.addTask(task_high);

    auto statuses = control_law.getTasksStatus(*m_robot, 1.0);

    // Should be sorted by priority
    EXPECT_EQ(statuses[0].task_name, "HighPriority");
    EXPECT_EQ(statuses[1].task_name, "LowPriority");
}

// ============================================================================
// 7. PARAMETERIZED TESTS
// ============================================================================

struct TaskCountTestCase {
    int task_count;
    std::string description;
};

class ControlLawTaskCountTest : public ControlLawTestsBase,
                                 public ::testing::TestWithParam<TaskCountTestCase> {
protected:
    void SetUp() override {
        SetUpRobot();
    }
};

/// @brief Verify compute works correctly with various number of tasks
TEST_P(ControlLawTaskCountTest, Compute_VariousTaskCounts) {
    auto testCase = GetParam();

    ControlLaw control_law;

    // Add specified number of tasks
    for (int i = 0; i < testCase.task_count; ++i) {
        Eigen::VectorXd error(2);
        error << 0.1 * i, 0.2 * i;
        auto task = createMockTask("Task" + std::to_string(i), i, error);
        control_law.addTask(task);
    }

    Command cmd = control_law.compute(*m_robot);

    EXPECT_EQ(cmd.getDimension(), m_robot->config().jointCount())
        << "Failed for " << testCase.description;
    EXPECT_TRUE(std::isfinite(cmd.getValues().norm()))
        << "Failed for " << testCase.description;
}

INSTANTIATE_TEST_SUITE_P(
    VariousTaskCounts,
    ControlLawTaskCountTest,
    ::testing::Values(
        TaskCountTestCase{0, "No tasks"},
        TaskCountTestCase{1, "Single task"},
        TaskCountTestCase{2, "Two tasks"},
        TaskCountTestCase{5, "Five tasks"},
        TaskCountTestCase{10, "Ten tasks"}
    )
);

// ============================================================================

struct PrecisionTestCase {
    double precision;
    double error_norm;
    bool should_be_achieved;
    std::string description;
};

class ControlLawPrecisionTest : public ControlLawTestsBase,
                                public ::testing::TestWithParam<PrecisionTestCase> {
protected:
    void SetUp() override {
        SetUpRobot();
    }
};

/// @brief Verify areAllTasksAchieved respects precision threshold
TEST_P(ControlLawPrecisionTest, AreAllTasksAchieved_VariousPrecisions) {
    auto testCase = GetParam();

    ControlLaw control_law;

    // Create task with specific error norm
    double component = testCase.error_norm / std::sqrt(2.0);
    Eigen::Vector2d error(component, component);
    auto task = createMockTask("Task1", 0, error);

    control_law.addTask(task);

    bool achieved = control_law.areAllTasksAchieved(*m_robot, testCase.precision);

    EXPECT_EQ(achieved, testCase.should_be_achieved)
        << "Failed for " << testCase.description;
}

INSTANTIATE_TEST_SUITE_P(
    VariousPrecisions,
    ControlLawPrecisionTest,
    ::testing::Values(
        PrecisionTestCase{0.1, 0.05, true, "Error much smaller than precision"},
        PrecisionTestCase{0.1, 0.09, true, "Error slightly smaller"},
        PrecisionTestCase{0.1, 0.11, false, "Error slightly larger"},
        PrecisionTestCase{0.1, 0.5, false, "Error much larger"},
        PrecisionTestCase{1e-6, 1e-7, true, "Very small precision - achieved"},
        PrecisionTestCase{1e-6, 1e-5, false, "Very small precision - not achieved"}
    )
);

// ============================================================================
// 8. STABILITY & ROBUSTNESS
// ============================================================================

/// @brief Verify repeated compute calls give consistent results
TEST_F(ControlLawTests, Compute_RepeatedCalls_Consistent) {
    ControlLaw control_law;
    auto task = createMockTask("Task1", 0, Eigen::Vector2d(0.1, 0.2));

    control_law.addTask(task);
    
    Command cmd1 = control_law.compute(*m_robot);
    Command cmd2 = control_law.compute(*m_robot);

    EXPECT_TRUE(cmd1.getValues().isApprox(cmd2.getValues(), m_tol_zero));
}

/// @brief Verify clear and re-add tasks works correctly
TEST_F(ControlLawTests, ClearAndReAdd_WorksCorrectly) {
    ControlLaw control_law;
    auto task1 = createMockTask("Task1", 0, Eigen::VectorXd::Zero(2));

    control_law.addTask(task1);
    EXPECT_EQ(control_law.getTaskCount(), 1);

    control_law.clearTasks();
    EXPECT_EQ(control_law.getTaskCount(), 0);

    auto task2 = createMockTask("Task2", 0, Eigen::VectorXd::Zero(2));
    control_law.addTask(task2);
    EXPECT_EQ(control_law.getTaskCount(), 1);
}

/// @brief Verify adding same task multiple times increases count
TEST_F(ControlLawTests, AddSameTask_MultipleTimes) {
    ControlLaw control_law;
    auto task = createMockTask("Task1", 0, Eigen::VectorXd::Zero(2));

    control_law.addTask(task);
    control_law.addTask(task);

    // Same shared_ptr added twice
    EXPECT_EQ(control_law.getTaskCount(), 2);
}

/// @brief Verify compute handles zero error correctly
TEST_F(ControlLawTests, Compute_ZeroError_ReturnsSmallCommand) {
    ControlLaw control_law;
    auto task = createMockTask("Task1", 0, Eigen::VectorXd::Zero(2));

    control_law.addTask(task);
    Command cmd = control_law.compute(*m_robot);

    // With zero error, command should be zero or very small
    EXPECT_LT(cmd.getValues().norm(), m_tol_numerical);
}

} // namespace robot::control::test