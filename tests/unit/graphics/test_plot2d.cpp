#include <gtest/gtest.h>
#include "graphics/plot2d.hpp"

/**
 * @brief Test the creation of empty plot
 */
TEST(Plot2DTest, EmptyPlot) {
    robot::graphics::Plot2D plot(100, 100);

    EXPECT_EQ(plot.pointCount(), 0);
    EXPECT_EQ(plot.lineCount(), 0);
}

/**
 * @brief Test adding points to the plot
 */
TEST(Plot2DTest, AddPoints) {
    robot::graphics::Plot2D plot(100, 100);
    plot.addPoint({10.f, 20.f});
    plot.addPoint({30.f, 40.f});

    EXPECT_EQ(plot.pointCount(), 2);

    const auto& points = plot.points();
    EXPECT_FLOAT_EQ(points[0].getPosition().x, 10.f);
    EXPECT_FLOAT_EQ(points[0].getPosition().y, 20.f);
}

/**
 * @brief Test adding lines to the plot
 */
TEST(Plot2DTest, AddLines) {
    robot::graphics::Plot2D plot(100, 100);
    plot.addLine({10.f, 20.f}, {30.f, 40.f});
    plot.addLine({50.f, 60.f}, {70.f, 80.f});

    EXPECT_EQ(plot.lineCount(), 2);

    const auto& line = plot.lines()[0];
    EXPECT_FLOAT_EQ(line[0].position.x, 10.f);
    EXPECT_FLOAT_EQ(line[1].position.y, 40.f);
}

/**
 * @brief Test the out of windows
 */
TEST(Plot2DTest, OutOfWindows) {
    robot::graphics::Plot2D plot(100, 100);

    plot.addPoint({10.f, 20.f});
    plot.addPoint({0.f, 20.f});
    plot.addPoint({10.f, 0.f});
    plot.addPoint({-30.f, 40.f});
    plot.addPoint({50.f, -60.f});
    
    plot.addLine({10.f, 20.f}, {30.f, 40.f});
    plot.addLine({0.f, 0.f}, {30.f, 40.f});
    plot.addLine({10.f, 20.f}, {0.f, 0.f});
    plot.addLine({-50.f, -60.f}, {70.f, 80.f});
    plot.addLine({50.f, 60.f}, {-70.f, -80.f});

    // TODO: error handling with status warning or error
    EXPECT_EQ(plot.pointCount(), 3);
    EXPECT_EQ(plot.lineCount(), 3);
}