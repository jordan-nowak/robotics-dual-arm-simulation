#include "graphics/plot2d.hpp"

int main() {
    // Environment
    sf::Vector2u windowSize = {800, 500};    
    sf::Vector2f pObject = {(windowSize.x/2.f)+0.f, 0.f};
    
    // Robot
    sf::Vector2f pOrigine = pObject;
    sf::Vector2f pTorso = {0.f+pOrigine.x, 200.f+pOrigine.y};
    sf::Vector2f pTorsoRight = {50.f+pOrigine.x, 200.f+pOrigine.y};
    sf::Vector2f pTorsoLeft = {-50.f+pOrigine.x, 200.f+pOrigine.y};
    sf::Vector2f pJointRight = {150.f+pOrigine.x, 210.f+pOrigine.y};
    sf::Vector2f pEffectorRight = {250.f+pOrigine.x, 190.f+pOrigine.y};
    sf::Vector2f pJointLeft = {-150.f+pOrigine.x, 180.f+pOrigine.y};
    sf::Vector2f pEffectorLeft = {-250.f+pOrigine.x, 220.f+pOrigine.y};

    // Window
    robot::graphics::Plot2D plot(windowSize.x, windowSize.y);
    
    // Base
    plot.addPoint(pOrigine);
    plot.addPoint(pTorso);
    plot.addLine(pOrigine, pTorso);
    
    plot.addPoint(pTorsoRight);
    plot.addLine(pTorso, pTorsoRight);
    plot.addPoint(pTorsoLeft);
    plot.addLine(pTorso, pTorsoLeft);

    // Right joint
    plot.addPoint(pJointRight);
    plot.addLine(pTorsoRight, pJointRight);
    plot.addPoint(pEffectorRight);
    plot.addLine(pJointRight, pEffectorRight);

    // Left joint
    plot.addPoint(pJointLeft);
    plot.addLine(pTorsoLeft, pJointLeft);
    plot.addPoint(pEffectorLeft);
    plot.addLine(pJointLeft, pEffectorLeft);

    plot.run();

    return EXIT_SUCCESS;
}