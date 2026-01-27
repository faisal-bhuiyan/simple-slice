#include <iostream>

#include "air_hockey_game/air_hockey_game.hpp"
#include "geometry/point.hpp"

int main() {
    using simple_slice::air_hockey_game::AirHockey;
    using simple_slice::geometry::Point;

    AirHockey game(/*length=*/2.0, /*width=*/1.0);
    Point start{1., 0.5};

    const auto hits = game.puck_hit_locations(start, /*angle=*/45.);

    std::cout << "First " << hits.size() << " wall contacts:\n";
    for (std::size_t i = 0; i < hits.size(); ++i) {
        std::cout << (i + 1) << ": " << hits[i] << "\n";
    }
    return 0;
}
