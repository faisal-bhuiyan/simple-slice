#include <cmath>

#include <gtest/gtest.h>

#include "air_hockey_game/air_hockey_game.hpp"
#include "geometry/point.hpp"

using simple_slice::air_hockey_game::AirHockey;
using simple_slice::geometry::Point;

TEST(AirHockeyGame, ThrowsOnInvalidDimensions) {
    EXPECT_THROW(AirHockey(0., 1.), std::invalid_argument);
    EXPECT_THROW(AirHockey(1., 0.), std::invalid_argument);
    EXPECT_THROW(AirHockey(-1., 2.), std::invalid_argument);
}

TEST(AirHockeyGame, ThrowsOnOutOfBoundsStart) {
    AirHockey game(2., 2.);
    EXPECT_THROW(game.puck_hit_locations(Point{-0.1, 1., 0.}, 45.), std::out_of_range);
    EXPECT_THROW(game.puck_hit_locations(Point{2.1, 1., 0.}, 45.), std::out_of_range);
}

TEST(AirHockeyGame, ReturnsTenHits) {
    AirHockey game(2., 2.);
    auto hits = game.puck_hit_locations(Point{1., 1., 0.}, 45.);
    EXPECT_EQ(hits.size(), 10U);
}

TEST(AirHockeyGame, HitsStayWithinBounds) {
    AirHockey game(2., 2.);
    auto hits = game.puck_hit_locations(Point{1., 1., 0.}, 30.);

    for (const auto& p : hits) {
        EXPECT_GE(p.GetX(), 0.);
        EXPECT_LE(p.GetX(), 2.);
        EXPECT_GE(p.GetY(), 0.);
        EXPECT_LE(p.GetY(), 2.);
    }
}

TEST(AirHockeyGame, StraightRightBouncesBetweenWalls) {
    AirHockey game(/*length=*/2., /*width=*/1.);

    // Start at (0,1,0), angle 0° -> straight right.
    const auto hits = game.puck_hit_locations(Point{0., 1., 0.}, /*angle=*/0.);

    ASSERT_EQ(hits.size(), 10U);  // 10 hits expected

    // Expected alternating hits on x=2 and x=0 at y=1
    for (std::size_t i = 0; i < hits.size(); ++i) {
        const double expected_x = (i % 2 == 0) ? 2. : 0.;
        EXPECT_NEAR(hits[i].GetX(), expected_x, 1e-9);
        EXPECT_NEAR(hits[i].GetY(), 1., 1e-9);
        EXPECT_NEAR(hits[i].GetZ(), 0., 1e-9);
    }
}

TEST(AirHockeyGame, Diagonal45BouncesBetweenCorners) {
    AirHockey game(/*length=*/2., /*width=*/2.);

    // Start at center; 45° should hit alternating corners
    const auto hits = game.puck_hit_locations(Point{1., 1., 0.}, /*angle=*/45.);

    ASSERT_EQ(hits.size(), 10U);

    for (std::size_t i = 0; i < hits.size(); ++i) {
        const bool even = (i % 2 == 0);
        const double expected_x = even ? 2. : 0.;
        const double expected_y = even ? 2. : 0.;

        EXPECT_NEAR(hits[i].GetX(), expected_x, 1e-9);
        EXPECT_NEAR(hits[i].GetY(), expected_y, 1e-9);
        EXPECT_NEAR(hits[i].GetZ(), 0., 1e-9);
    }
}
