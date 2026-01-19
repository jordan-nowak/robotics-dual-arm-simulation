#include "graphics/plot2d.hpp"

#include "core/robot.hpp"
#include "core/environment.hpp"


int main() {
    // Defined the environment
    robot::core::EnvironmentConfig env;
    env.worldSize = {800, 500};
    env.timeStep = 0.01f;

    robot::core::Environment environment(env);

    // Defined the robot's configuration
    sf::Vector2f pOrigine = {(environment.worldSize().x/2.f)+0.f, 0.f};
    sf::Vector2f pTorso = {0.f+pOrigine.x, 200.f+pOrigine.y};
    sf::Vector2f pTorsoRight = {50.f+pOrigine.x, 200.f+pOrigine.y};
    sf::Vector2f pTorsoLeft = {-50.f+pOrigine.x, 200.f+pOrigine.y};
    sf::Vector2f pJointRight = {150.f+pOrigine.x, 210.f+pOrigine.y};
    sf::Vector2f pEffectorRight = {250.f+pOrigine.x, 190.f+pOrigine.y};
    sf::Vector2f pJointLeft = {-150.f+pOrigine.x, 180.f+pOrigine.y};
    sf::Vector2f pEffectorLeft = {-250.f+pOrigine.x, 220.f+pOrigine.y};

    // Create the window
    robot::graphics::Plot2D plot(environment.worldSize().x, environment.worldSize().y);
    
    // Add base
    plot.addPoint(pOrigine);
    plot.addPoint(pTorso);
    plot.addLine(pOrigine, pTorso);
    
    plot.addPoint(pTorsoRight);
    plot.addLine(pTorso, pTorsoRight);
    plot.addPoint(pTorsoLeft);
    plot.addLine(pTorso, pTorsoLeft);

    // Add right joint
    plot.addPoint(pJointRight);
    plot.addLine(pTorsoRight, pJointRight);
    plot.addPoint(pEffectorRight);
    plot.addLine(pJointRight, pEffectorRight);

    // Add left joint
    plot.addPoint(pJointLeft);
    plot.addLine(pTorsoLeft, pJointLeft);
    plot.addPoint(pEffectorLeft);
    plot.addLine(pJointLeft, pEffectorLeft);
    
    // Display the robot
    plot.run();

    return EXIT_SUCCESS;
}