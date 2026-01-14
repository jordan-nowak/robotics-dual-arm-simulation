#pragma once

#include <SFML/Graphics.hpp>
#include <vector>

namespace robot::graphics {
    class Plot2D {
        public:
            //! \brief This constructor define 'm_window'.
            //! \param The number of pixel for the width and the height.
            //! \return Nothing.
            Plot2D(unsigned int _width, unsigned int _height);
            
            //! \brief This function add a new point in the 'm_points' vector.
            //! \param The point to add.
            //! \return Nothing.
            void addPoint(sf::Vector2f _point);
            
            //! \brief This function add a new line in the 'm_lines' vector.
            //! \param The two points (_point1 and _point2) that define the line to be added.
            //! \return Nothing.
            void addLine(sf::Vector2f _point1, sf::Vector2f _point2);
            
            //! \brief This function opens the 'm_window' and displays all points and lines contained in the associated vectors.
            //! \param Nothing.
            //! \return Nothing.
            void run();
            
        public:
            std::size_t pointCount() const; /*!< Return the number of points stored */
            std::size_t lineCount() const; /*!< Return the number of lines stored */
            const std::vector<sf::CircleShape>& points() const; /*!< Return the values contained in m_points (Read-only access) */
            const std::vector<sf::VertexArray>& lines() const; /*!< Return the values contained in m_lines (Read-only access) */

        private:
            sf::RenderWindow m_window; /*!< Display window */
            std::vector<sf::CircleShape> m_points; /*!< Point vectors to display */
            std::vector<sf::VertexArray> m_lines; /*!< Line vectors to display */
            sf::View m_view; /*!< Custom view */

            //! \brief This function checks if the point to be added is within the window boundaries.
            //! \param The point to test
            //! \return Boolean which say if the point is inside of the window
            bool isInsideWindow(sf::Vector2f point) const;
    };
}


