#include <iostream>
#include "graphics/plot2d.hpp"

robot::graphics::Plot2D::Plot2D(unsigned int width, unsigned int height)
: m_window(sf::VideoMode({width, height}), "2D Plot"),
  m_view(sf::FloatRect({0.f, 0.f}, {static_cast<float>(width), static_cast<float>(height)}))
{
    // Origin at bottom left and Y axis upward 
    m_view.setCenter({static_cast<float>(width) / 2.f, static_cast<float>(height) / 2.f});
    m_view.setSize({static_cast<float>(width), -static_cast<float>(height)});

    m_window.setView(m_view);
}

void robot::graphics::Plot2D::addPoint(sf::Vector2f _point, sf::Color _color) {
    // TODO: error handling with status warning or error
    if (!isInsideWindow(_point)) {
        std::cout << "--> ERROR: This point is outside the window that defines the display environment." << std::endl;
        return;
    }

    sf::CircleShape p(4.f);
    p.setFillColor(_color);
    p.setOrigin({4.f, 4.f});
    p.setPosition(_point);
    m_points.push_back(p);
}

void robot::graphics::Plot2D::addLine(sf::Vector2f p1, sf::Vector2f p2, sf::Color _color) {
    // TODO: error handling with status warning or error
    if (!isInsideWindow(p1)) {
        std::cout << "--> ERROR: The first point is outside the window that defines the display environment." << std::endl;
        return;
    }

    // TODO: error handling with status warning or error
    if (!isInsideWindow(p2)) {
        std::cout << "--> ERROR: The second point is outside the window that defines the display environment." << std::endl;
        return;
    }

    sf::VertexArray line(sf::PrimitiveType::Lines, 2);
    line[0].position = p1;
    line[1].position = p2;
    line[0].color = _color;
    line[1].color = _color;

    m_lines.push_back(line);
}

void robot::graphics::Plot2D::run() {
    while (m_window.isOpen()) {
        if (!update())
            break;
    }
}

bool robot::graphics::Plot2D::update() {
    // Handle events
    while (auto event = m_window.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            m_window.close();
            return false;
        }
    }
    
    // Render
    m_window.clear(sf::Color::White);
    
    for (const auto& l : m_lines)
        m_window.draw(l);
    for (const auto& p : m_points)
        m_window.draw(p);
    
    m_window.display();
    
    return m_window.isOpen();
}

void robot::graphics::Plot2D::clear() {
    m_points.clear();
    m_lines.clear();
}

bool robot::graphics::Plot2D::isInsideWindow(sf::Vector2f point) const {
    const auto size = m_window.getSize();

    return point.x >= 0.f &&
           point.y >= 0.f &&
           point.x <= static_cast<float>(size.x) &&
           point.y <= static_cast<float>(size.y);
}

std::size_t robot::graphics::Plot2D::pointCount() const {
    return m_points.size();
}

std::size_t robot::graphics::Plot2D::lineCount() const {
    return m_lines.size();
}

const std::vector<sf::CircleShape>& robot::graphics::Plot2D::points() const {
    return m_points;
}

const std::vector<sf::VertexArray>& robot::graphics::Plot2D::lines() const {
    return m_lines;
}
