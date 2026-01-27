#include "air_hockey_game/air_hockey_game.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

#include "geometry/point.hpp"
#include "geometry/utilities.hpp"
#include "geometry/vector.hpp"

namespace simple_slice::air_hockey_game {

AirHockey::AirHockey(double length, double width) : length_{length}, width_{width} {
    if (!std::isfinite(length_) || !std::isfinite(width_)) {
        throw std::invalid_argument("AirHockey: length/width must be finite");
    }
    if (length_ <= 0. || width_ <= 0.) {
        throw std::invalid_argument("AirHockey: length/width must be > 0");
    }
}

std::array<Point, 10> AirHockey::puck_hit_locations(const Point& initial_pos, double angle) const {
    auto in_bounds = [&](double x, double y) {
        return x >= 0. && x <= length_ && y >= 0. && y <= width_;
    };

    if (!in_bounds(initial_pos.GetX(), initial_pos.GetY())) {
        throw std::out_of_range("puck_hit_locations: initial_pos outside table");
    }

    std::array<Point, 10> hits{};
    Vector3D position = ToPositionVector(initial_pos);
    Vector3D direction = DirectionVector(angle);
    for (size_t i = 0; i < hits.size(); ++i) {
        const double tx = TimeToVerticalWall(position.x, direction.x);
        const double ty = TimeToHorizontalWall(position.y, direction.y);
        const double t = std::min(tx, ty);

        // If the time is infinite, the puck will never hit a wall
        if (!std::isfinite(t)) {
            break;
        }

        position = ClampToTable(position + direction * t);
        hits[i] = ToPoint(position);

        const bool hit_vertical = std::abs(tx - t) <= 10. * kEpsilon;
        const bool hit_horizontal = std::abs(ty - t) <= 10. * kEpsilon;

        if (hit_vertical) {
            direction.x = -direction.x;
        }
        if (hit_horizontal) {
            direction.y = -direction.y;
        }

        // Corner case: if numerical noise made neither true, use proximity to walls.
        if (!hit_vertical && !hit_horizontal) {
            const bool on_left_or_right = (std::abs(position.x - 0.) <= 10. * kEpsilon) ||
                                          (std::abs(position.x - length_) <= 10. * kEpsilon);

            const bool on_bottom_or_top = (std::abs(position.y - 0.) <= 10. * kEpsilon) ||
                                          (std::abs(position.y - width_) <= 10. * kEpsilon);

            if (on_left_or_right) {
                direction.x = -direction.x;
            }
            if (on_bottom_or_top) {
                direction.y = -direction.y;
            }
        }
    }

    return hits;
}

Vector3D AirHockey::ToPositionVector(const Point& p) {
    return Vector3D{p.GetX(), p.GetY(), 0.};
}

Point AirHockey::ToPoint(const Vector3D& v) {
    return Point{v.x, v.y};
}

Vector3D AirHockey::DirectionVector(double angle) {
    const double theta = angle * (std::numbers::pi / 180.);
    return Vector3D{std::cos(theta), std::sin(theta), 0.};
}

Vector3D AirHockey::ClampToTable(const Vector3D& position) const {
    return Vector3D{
        std::clamp(position.x, 0., length_),  // Clamp x-coordinate to table length
        std::clamp(position.y, 0., width_),   // Clamp y-coordinate to table width
        0.,
    };
}

double AirHockey::TimeToVerticalWall(double x, double dx) const {
    if (std::abs(dx) <= kEpsilon) {
        return std::numeric_limits<double>::infinity();
    }
    const double target_x = (dx > 0.) ? length_ : 0.;
    const double t = (target_x - x) / dx;
    return (t > kEpsilon) ? t : std::numeric_limits<double>::infinity();
}

double AirHockey::TimeToHorizontalWall(double y, double dy) const {
    if (std::abs(dy) <= kEpsilon) {
        return std::numeric_limits<double>::infinity();
    }
    const double target_y = (dy > 0.) ? width_ : 0.;
    const double t = (target_y - y) / dy;
    return (t > kEpsilon) ? t : std::numeric_limits<double>::infinity();
}

}  // namespace simple_slice::air_hockey_game
