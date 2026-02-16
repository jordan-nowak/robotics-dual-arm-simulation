#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <thread>
#include <chrono>

#include "core/robot.hpp"
#include "core/robot_config.hpp"
#include "core/robot_state.hpp"
#include "core/environment.hpp"
#include "task/task_reach_point.hpp"
#include "task/task_com.hpp"
#include "control/control_law.hpp"
#include "control/command.hpp"
#include "graphics/plot2d.hpp"

using namespace robot;

/**
 * @brief Print robot state information
 */
void printRobotState(const core::Robot& robot, int iteration) {
    std::cout << "\n========== Iteration " << iteration << " ==========\n";
    std::cout << "Joint Positions: ";
    for (const auto& q : robot.state().jointPositions) {
        std::cout << std::fixed << std::setprecision(3) << q << " ";
    }
    std::cout << "\n";
}

/**
 * @brief Print task status
 */
void printTaskStatus(const std::vector<control::TaskStatus>& statuses) {
    std::cout << "\nTask Status:\n";
    for (const auto& status : statuses) {
        std::cout << "  - " << status.task_name 
                  << " (priority " << status.priority << "): "
                  << "error = " << std::fixed << std::setprecision(4) 
                  << status.error_norm
                  << " | " << (status.achieved ? "|| ACHIEVED" : "|| IN PROGRESS")
                  << "\n";
    }
}

/**
 * @brief Draw robot configuration on plotter
 */
void drawRobot(
    robot::graphics::Plot2D& plotter,
    const robot::core::Robot& robot,
    const robot::modeling::geometric::DHModel& dh_model,
    sf::Color link_color = sf::Color::Black,
    sf::Color joint_color = sf::Color::Black
) {
    const auto& links = robot.config().links();
    const auto& joint_positions = robot.state().jointPositions;
    
    // Draw links
    for (size_t i = 0; i < links.size(); ++i) {
        // Draw joint
        modeling::geometric::Transform T = dh_model.baseToLink(i, joint_positions);
        plotter.addPoint(
            sf::Vector2f(T.position().x(), T.position().y()),
            joint_color
        );
        
        // Draw link (line to parent)
        if (links[i].ID_parent >= 0) {
            modeling::geometric::Transform T_parent = dh_model.baseToLink(links[i].ID_parent, joint_positions);
            plotter.addLine(
                sf::Vector2f(T_parent.position().x(), T_parent.position().y()),
                sf::Vector2f(T.position().x(), T.position().y()),
                link_color
            );
        }
    }
}

/**
 * @brief Main demo function
 */
int main() {
    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "       Dual-Arm Humanoid Robot Control Demo       " << std::endl;
    std::cout << "--------------------------------------------------" << std::endl << std::endl;
    
    // ========================================================================
    // 1. CREATE ENVIRONMENT
    // ========================================================================

    core::Environment env(
        {
            .worldSize = {800., 600.},
            .gravity = EARTH_GRAVITY,
            .timeStep = 0.01f // 10ms timestep
        }
    );

    std::cout << "|| Environment created (dt = " << env.timeStep() << "s)\n";
    
    // ========================================================================
    // 2. CREATE ROBOT
    // ========================================================================
    
    /* Create a dual-arm humanoid robot configuration (T-shape)
    * 
    * Structure:
    *   - Link 0: Base (fixed to ground)
    *   - Link 1-2: Intermediate frame to follow dh-modified convention (no motor)
    *   - Link 3: Center of the shoulder (no motor)
    *   - Link 4-6: Right arm (shoulder, elbow, wrist)
    *   - Link 7-9: Left arm (shoulder, elbow, wrist)
    */
    float d0 = env.worldSize().x/2.0; // pixels in the windows
    float d1 = 400.;
    float d2 = d1/4.;
    float d3 = d1/3.;
    float d4 = d1/3.;
    float m1 = 5.; // Kg
    float m2 = m1/3.;
    float m3 = m1/3.;
    core::RobotConfig config(
                "dual_arm_robot",
                {
                    // TORSO (base + body)
                    { .name = "j0", .minPosition = -M_PI, .maxPosition = M_PI, .maxVelocity = 1.0f},
                    { .name = "j1", .minPosition = -M_PI, .maxPosition = M_PI, .maxVelocity = 1.0f},
                    { .name = "j2", .minPosition = -M_PI, .maxPosition = M_PI, .maxVelocity = 1.0f},
                    { .name = "j3", .minPosition = -M_PI, .maxPosition = M_PI, .maxVelocity = 1.0f},
                    // RIGHT ARM (3 DOF)
                    { .name = "j4", .minPosition = -M_PI, .maxPosition = M_PI, .maxVelocity = 1.0f},
                    { .name = "j5", .minPosition = -M_PI, .maxPosition = M_PI, .maxVelocity = 1.0f},
                    { .name = "j6", .minPosition = -M_PI, .maxPosition = M_PI, .maxVelocity = 1.0f},
                    // LEFT ARM (3 DOF) - Mirror of right arm
                    { .name = "j7", .minPosition = -M_PI, .maxPosition = M_PI, .maxVelocity = 1.0f},
                    { .name = "j8", .minPosition = -M_PI, .maxPosition = M_PI, .maxVelocity = 1.0f},
                    { .name = "j9", .minPosition = -M_PI, .maxPosition = M_PI, .maxVelocity = 1.0f}
                },
                {
                    // TORSO (base + body)
                    { .name = "l0", .a = d0, .alpha = 0., .d = 0., .theta = 0.,     .mass =  m1, .com = Eigen::Vector3d(0.0, d1/2., 0.0), .ID_link = 0, .ID_parent = -1, .motor = true},
                    { .name = "l1", .a = 0., .alpha = 0., .d = 0., .theta = M_PI/2.,  .mass = 0., .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 1, .ID_parent =  0, .motor = false}, // no motor
                    { .name = "l2", .a =  d1, .alpha = 0., .d = 0., .theta = 0.,     .mass = 0., .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 2, .ID_parent =  1, .motor = false}, // no motor
                    { .name = "l3", .a = 0., .alpha = 0., .d = 0., .theta = -M_PI/2., .mass = 0., .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 3, .ID_parent =  2, .motor = false}, // no motor
                    // RIGHT ARM (3 DOF)
                    { .name = "l4", .a =  d2, .alpha = 0., .d = 0., .theta = 0.,     .mass =  m2, .com = Eigen::Vector3d(d3/2., 0.0, 0.0), .ID_link = 4, .ID_parent =  3, .motor = true},
                    { .name = "l5", .a =  d3, .alpha = 0., .d = 0., .theta = 0.,     .mass =  m3, .com = Eigen::Vector3d(d4/2., 0.0, 0.0), .ID_link = 5, .ID_parent =  4, .motor = true},
                    { .name = "l6", .a =  d4, .alpha = 0., .d = 0., .theta = 0.,     .mass = 0., .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 6, .ID_parent =  5, .motor = false}, // no motor
                    // LEFT ARM (3 DOF) - Mirror of right arm
                    { .name = "l7", .a = -d2, .alpha = 0., .d = 0., .theta = 0.,     .mass =  m2, .com = Eigen::Vector3d(-d3/2., 0.0, 0.0), .ID_link = 7, .ID_parent =  3, .motor = true},
                    { .name = "l8", .a = -d3, .alpha = 0., .d = 0., .theta = 0.,     .mass =  m3, .com = Eigen::Vector3d(-d4/2., 0.0, 0.0), .ID_link = 8, .ID_parent =  7, .motor = true},
                    { .name = "l9", .a = -d4, .alpha = 0., .d = 0., .theta = 0.,     .mass = 0., .com = Eigen::Vector3d(0.0, 0.0, 0.0), .ID_link = 9, .ID_parent =  8, .motor = false} // no motor
                }
            );

    core::Robot robot(config);
    modeling::geometric::CenterOfMass com(robot.config());

    // Initialise the limits for the command
    const auto& joints = robot.config().joints();
    const auto& links = robot.config().links();

    const std::size_t dof = joints.size();
    Eigen::VectorXd vel_lower(dof);
    Eigen::VectorXd vel_upper(dof);

    for (std::size_t i = 0; i < dof; ++i) {
        if (!links[i].motor) { // security
            vel_lower(i) = 0.0;
            vel_upper(i) = 0.0;
        } else {
            const double vmax = joints[i].maxVelocity;
            vel_lower(i) = -joints[i].maxVelocity;
            vel_upper(i) =  joints[i].maxVelocity;
        }
    }

    std::cout << "|| Robot created:\n";
    std::cout << "  -> " << robot.config().jointCount() << " joints\n";
    std::cout << "  -> " << robot.config().linkCount() << " links\n";
    std::cout << "  -> Total mass: " << com.getTotalMass() << " kg\n\n";
    
    // Initialize to home position
    std::vector<double> home_position(robot.config().jointCount(), 0.0);
    robot.state().jointPositions = home_position;

    std::cout << "|| Robot in the home position:\n";
    std::cout << robot.state() << "\n";
    
    // ========================================================================
    // 3. CREATE TASKS
    // ========================================================================
    
    std::cout << "Creating tasks...\n";

    // Task to reach desired point
    Eigen::Vector3d target_point(d0+d2+d3+d4-30, d1+50, 0.0); // according to the right wrist position

    auto task_reach_point = std::make_shared<task::TaskReachPoint>(
        target_point,
        robot.config(),
        6,  // Right hand is link 4 // TODO: auto select if is it right or left hand to control
        "Hand",
        0   // Priority level
    );
    std::cout << "  || Task: Reach desired point -> " << target_point.transpose() << "\n";
    
    // Task to maintain CoM stability
    Eigen::Vector3d target_com(com.getGlobalCoM(robot.state().jointPositions)(0)+10, 0.0, 0.0);  // Keep CoM centered

    auto task_stability = std::make_shared<task::TaskCoM>(
        target_com,
        robot.config(),
        "Stability",
        1
    );

    std::cout << "  || Task: CoM stability → " << target_com.transpose() << "\n\n";
    
    // ========================================================================
    // 4. CREATE CONTROL LAW AND COMMAND
    // ========================================================================
    
    control::ControlLaw controller(1e-6);
    controller.addTask(task_reach_point);
    controller.addTask(task_stability);

    // create command
        control::Command cmd = controller.compute(robot);
        cmd.setLimits(vel_lower, vel_upper);
        
        // Apply velocity limits
        cmd.saturate();
    
    std::cout << "|| Control law created with " << controller.getTaskCount() << " tasks\n\n";
    
    // ========================================================================
    // 5. SIMULATION PARAMETERS
    // ========================================================================
    
    const double precision = 1e-3;        // 1mm precision
    const int max_iterations = 1000;       // Max 10 seconds
    const int print_every = 50;           // Print every 50 iterations
    const double gain = 5.0;              // Control gain (damping)

    // ========================================================================
    // VISUALIZATION SETUP
    // ========================================================================
    
    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "Enable real-time visualization? (y/n): ";
    char viz_choice;
    std::cin >> viz_choice;
    
    bool enable_visualization = (viz_choice == 'y' || viz_choice == 'Y');
    
    std::unique_ptr<robot::graphics::Plot2D> plotter;
    modeling::geometric::DHModel dh_model(robot.config());
    
    if (enable_visualization) {
        plotter = std::make_unique<robot::graphics::Plot2D>(
            env.worldSize().x, 
            env.worldSize().y
        );
        std::cout << "|| Visualization enabled\n";
    }
    
    // ========================================================================
    // 6. CONTROL LOOP
    // ========================================================================
    
    std::cout << "Starting control loop..." << std::endl;
    std::cout << "--------------------------------------------------" << std::endl;
    
    bool all_achieved = false;
    int iteration = 0;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    while (iteration < max_iterations && !all_achieved) {
        
        // Check window close event
        if (enable_visualization && !plotter->isOpen()) {
            std::cout << "\nVisualization window closed. Stopping simulation.\n";
            break;
        }
        
        // Check if all tasks are achieved
        if (controller.areAllTasksAchieved(robot, precision)) {
            all_achieved = true;
            std::cout << "\n[ CONGRATULATION ] All tasks achieved at iteration " << iteration << "!\n";
            break;
        }
        
        // Print status periodically
        if (iteration % print_every == 0) {
            printRobotState(robot, iteration);
            auto statuses = controller.getTasksStatus(robot, precision);
            printTaskStatus(statuses);
        }
        
        // Compute control command
        cmd = controller.compute(robot);
        
        // Apply velocity limits and saturate motor if necessary
        cmd.saturate();
        
        // Apply gain and integrate (velocity to position)
        Eigen::VectorXd delta_q = gain * env.timeStep() * cmd.getValues();
        
        std::vector<double> new_positions = robot.state().jointPositions;
        for (size_t i = 0; i < new_positions.size(); ++i) {
            new_positions[i] += delta_q(i);
            
            // Clamp to joint limits
            new_positions[i] = std::clamp(
                new_positions[i],
                static_cast<double>(joints[i].minPosition),
                static_cast<double>(joints[i].maxPosition)
            );
        }

        robot.state().jointPositions = new_positions;

        // VISUALIZATION UPDATE
        if (enable_visualization && iteration % 5 == 0) {  // Update every 5 iterations
            plotter->clear();
            Eigen::Vector3d current_com = com.getGlobalCoM(new_positions);
            drawRobot(*plotter, robot, dh_model, sf::Color::Black, sf::Color::Black);
            plotter->addPoint(sf::Vector2f(target_point(0), target_point(1)), sf::Color::Blue);
            plotter->addPoint(sf::Vector2f(target_com(0), target_com(1)), sf::Color::Magenta);
            plotter->addPoint(sf::Vector2f(current_com(0), current_com(1)), sf::Color::Red);
            plotter->update();
        }
        
        ++iteration;
        
        // Real-time simulation
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time
    );
    
    // ========================================================================
    // 7. FINAL RESULTS
    // ========================================================================
    
    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "               SIMULATION COMPLETED               " << std::endl;
    std::cout << "--------------------------------------------------" << std::endl << std::endl;
    std::cout << "Total iterations: " << iteration << "\n";
    std::cout << "Computation time: " << duration.count() << " ms\n";
    std::cout << "Average time per iteration: " 
              << (duration.count() / (double)iteration) << " ms\n\n";
    
    // Final state
    printRobotState(robot, iteration);
    
    // Final task status
    auto final_statuses = controller.getTasksStatus(robot, precision);
    printTaskStatus(final_statuses);
    
    // Final robot positions
    std::cout << "\nFinal Robot Error:\n";
    std::cout << robot.state() << "\n\n";
    std::cout << "-> task_reach_point.error: " << task_reach_point->computeError(robot).transpose() << "\n";

    // ========================================================================
    // 8. KEEP VISUALIZATION OPEN
    // ========================================================================

    if (enable_visualization && plotter->isOpen()) {
        std::cout << "\nPress any key in terminal to close visualization...\n";
        
        // Keep window open until user closes it
        while (plotter->isOpen()) {
            plotter->update();
            std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 FPS
        }
    }
    
    std::cout << "\n|| Demo completed successfully!\n";
    
    return EXIT_SUCCESS;
}