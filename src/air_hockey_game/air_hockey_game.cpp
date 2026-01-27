#include "air_hockey_game/air_hockey_game.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry/point.hpp"
#include "geometry/utilities.hpp"
#include "geometry/vector.hpp"

namespace simple_slice::air_hockey_game {

AirHockey::AirHockey(double length, double width) : length_{length}, width_{width} {
    if (!std::isfinite(length_) || !std::isfinite(width_)) {
        throw std::invalid_argument("AirHockey: provided length/width for table must be finite");
    }
    if (length_ <= 0. || width_ <= 0.) {
        throw std::invalid_argument("AirHockey: provided length/width for table must be > 0");
    }
}

std::array<Point, 10> AirHockey::puck_hit_locations(const Point& initial_position, double angle)
    const {
    //----------------------------------------------
    // Checks
    //----------------------------------------------
    auto in_bounds = [&](double x, double y) {
        return x >= 0. && x <= this->length_ && y >= 0. && y <= this->width_;
    };
    if (!in_bounds(initial_position.GetX(), initial_position.GetY())) {
        throw std::out_of_range("puck_hit_locations: provided initial_position is outside table");
    }

    //----------------------------------------------------
    // Calculate contact points (ideal reflections)
    // Algorithm:
    // - Compute time to next vertical wall and horizontal wall
    // - Take the smaller positive time as the next contact
    // - Advance the puck to the contact point
    // - Reflect velocity component(s) depending on which wall was hit
    // - Repeat for up to n contacts
    //----------------------------------------------------

    std::array<Point, 10> hits{};

    // Convert initial position to a working vector (XY plane)
    // Direction is a unit vector from the input angle; speed is 1
    Vector3D position = Vector3D{initial_position.GetX(), initial_position.GetY(), 0.};
    Vector3D velocity = DirectionVector(angle) * 1.;  // speed is 1 unit per time step

    for (std::size_t i = 0; i < hits.size(); ++i) {
        // Compute time to next wall along each axis
        const double time_to_next_x_wall = TimeToNextWall(position.x, velocity.x, 0., this->length_);
        const double time_to_next_y_wall = TimeToNextWall(position.y, velocity.y, 0., this->width_);
        const double time_to_next_wall = std::min(time_to_next_x_wall, time_to_next_y_wall);

        // If both time is infinite, the puck will never hit a wall -> stop
        if (!std::isfinite(time_to_next_wall)) {
            break;
        }

        // Advance the puck to the contact point (clamped to avoid small numerical drift)
        position = ClampToTable(position + velocity * time_to_next_wall);
        hits[i] = Point{position.x, position.y, 0.};

        // Check if the puck hit a vertical or horizontal wall and reflect the velocity accordingly
        // Hitting both walls is permitted i.e. for a corner hit
        const bool hit_vertical =
            std::abs(time_to_next_x_wall - time_to_next_wall) <= 10. * kEpsilon;
        const bool hit_horizontal =
            std::abs(time_to_next_y_wall - time_to_next_wall) <= 10. * kEpsilon;

        // Reflect velocity vector off wall
        if (hit_vertical) {
            velocity.x = -velocity.x;
        }
        if (hit_horizontal) {
            velocity.y = -velocity.y;
        }

        // Fallback: if numerical noise made neither true, use proximity to walls i.e.
        if (!hit_vertical && !hit_horizontal) {
            const bool on_left_or_right = (std::abs(position.x - 0.) <= 10. * kEpsilon) ||
                                          (std::abs(position.x - length_) <= 10. * kEpsilon);

            const bool on_bottom_or_top = (std::abs(position.y - 0.) <= 10. * kEpsilon) ||
                                          (std::abs(position.y - width_) <= 10. * kEpsilon);

            if (on_left_or_right) {
                velocity.x = -velocity.x;
            }
            if (on_bottom_or_top) {
                velocity.y = -velocity.y;
            }
        }
    }

    return hits;
}

Vector3D AirHockey::DirectionVector(double angle) {
    const double theta = angle * (std::numbers::pi / 180.);
    return Vector3D{std::cos(theta), std::sin(theta), 0.};
}

Vector3D AirHockey::ClampToTable(const Vector3D& position) const {
    return Vector3D{
        std::clamp(position.x, 0., length_),  // Clamp x-coordinate to table length
        std::clamp(position.y, 0., width_),   // Clamp y-coordinate to table width
        0.
    };
}

double AirHockey::TimeToNextWall(
    double position, double velocity, double min_bound, double max_bound
) const {
    // If the velocity is zero, the puck will never hit a wall on this axis -> return infinity
    if (std::abs(velocity) <= kEpsilon) {
        return std::numeric_limits<double>::infinity();
    }

    // velocity is positive -> puck heading to max_bound
    // velocity is negative -> puck heading to min_bound
    const double target = (velocity > 0.) ? max_bound : min_bound;

    // Compute the time to hit the wall
    const double t = (target - position) / velocity;

    // If time ~0, puck is already at the wall (after a contact) -> return infinity
    return (t > kEpsilon) ? t : std::numeric_limits<double>::infinity();
}

void AirHockey::PrintTable(const std::array<Point, 10>& hits, int cols, int rows) const {
    // Build an empty ASCII canvas (rows x cols)
    std::vector<std::string> canvas(rows, std::string(cols, ' '));

    // Draw top/bottom borders
    for (int x = 0; x < cols; ++x) {
        canvas[0][x] = '-';
        canvas[rows - 1][x] = '-';
    }
    // Draw left/right borders
    for (int y = 0; y < rows; ++y) {
        canvas[y][0] = '|';
        canvas[y][cols - 1] = '|';
    }
    // Draw corners
    canvas[0][0] = canvas[0][cols - 1] = canvas[rows - 1][0] = canvas[rows - 1][cols - 1] = '+';

    // Map physical coordinates -> ASCII grid column
    auto x_to_col = [&](double x) {
        const double t = std::clamp(x / length_, 0., 1.);
        return static_cast<int>(std::lround(t * (cols - 1)));
    };

    // Map physical coordinates -> ASCII grid row (inverted Y axis)
    auto y_to_row = [&](double y) {
        const double t = std::clamp(y / width_, 0., 1.);
        return static_cast<int>(std::lround((1. - t) * (rows - 1)));
    };

    // Plot hit points using digits 1..9, 0 (for 10th)
    for (std::size_t i = 0; i < hits.size(); ++i) {
        const int col = x_to_col(hits[i].GetX());
        const int row = y_to_row(hits[i].GetY());

        if (row >= 0 && row < rows && col >= 0 && col < cols) {
            const char label = static_cast<char>('0' + ((i + 1) % 10));
            canvas[row][col] = label;
        }
    }

    // Render to stdout
    std::cout << "\nTable (ASCII):\n";
    for (const auto& line : canvas) {
        std::cout << line << "\n";
    }
    std::cout << "Legend: digits = hit index (1..9, 0 = 10th)\n";
}

}  // namespace simple_slice::air_hockey_game
