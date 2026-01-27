#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

/*
 * A simple class to represent a 2D point in Cartesian coordinates
 */
class Point {
public:
    Point() : x_{0.}, y_{0.} {}
    Point(double x, double y) : x_{x}, y_{y} {}

    /// Get the x-coordinate of the point
    double GetX() const { return x_; }

    /// Get the y-coordinate of the point
    double GetY() const { return y_; }

private:
    double x_;  //< x-coordinate of the point
    double y_;  //< y-coordinate of the point
};

/*
 * Overload the << operator to print the point
 */
std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << "(" << p.GetX() << ", " << p.GetY() << ")";
    return os;
}

/*
 * A simple class to represent an air hockey game
 */
class AirHockey {
public:
    AirHockey(double length, double width) : length_{length}, width_{width} {}

    /*
     * Coordinate system / angle conventions:
     * - Rectangular Cartesian coordinates (RCC)
     * - Origin (0,0) is the bottom-left corner of the table
     * - Table spans x in [0, length_], y in [0, width_]
     * - angle_deg is measured in degrees CCW from +x axis:
     *     0°   -> +x (right)
     *     90°  -> +y (up)
     *     180° -> -x (left)
     *     270° -> -y (down)
     *
     * Physics assumptions:
     * - Constant speed, straight-line motion between wall hits
     * - Perfectly elastic reflections on walls (mirror reflection)
     * - No friction, no spin, no energy loss
     */
    std::array<Point, 10> puck_hit_locations(const Point& initial_position, double angle) const {
        std::array<Point, 10> hits;

        // Direction unit vector from angle
        const double pi{std::acos(-1.)};
        const double theta = angle * (pi / 180.);
        double dx{std::cos(theta)};
        double dy{std::sin(theta)};

        // Current puck location
        double x{initial_position.GetX()};
        double y{initial_position.GetY()};

        // Small epsilon to handle floating point comparisons / corner hits.
        constexpr double eps{1e-12};

        for (size_t bounce = 0; bounce < hits.size(); ++bounce) {
            // Time to reach the next vertical wall (x = 0 or x = length_).
            double tx{std::numeric_limits<double>::infinity()};
            if (std::abs(dx) > eps) {
                const double target_x = (dx > 0.) ? length_ : 0.;
                tx = (target_x - x) / dx;  // dx sign ensures tx should be >= 0
            }

            // Time to reach the next horizontal wall (y = 0 or y = width_).
            double ty{std::numeric_limits<double>::infinity()};
            if (std::abs(dy) > eps) {
                const double target_y = (dy > 0.) ? width_ : 0.;
                ty = (target_y - y) / dy;
            }

            // If we are numerically on a wall and heading out, tx/ty can be ~0.
            // Require a strictly positive step.
            if (tx <= eps) {
                tx = std::numeric_limits<double>::infinity();
            }
            if (ty <= eps) {
                ty = std::numeric_limits<double>::infinity();
            }

            const double t = std::min(tx, ty);
            if (!std::isfinite(t)) {
                // Degenerate case: no movement or starting outside the table.
                break;
            }

            // Advance to the wall contact point.
            x += dx * t;
            y += dy * t;

            // Clamp to boundaries to avoid tiny floating drift.
            if (x < 0.) {
                x = 0.;
            }
            if (x > length_) {
                x = length_;
            }
            if (y < 0.) {
                y = 0.;
            }
            if (y > width_) {
                y = width_;
            }

            hits[bounce] = Point{x, y};

            // Determine which wall was hit; corner hit reflects both components.
            const bool hit_vertical = std::abs(tx - t) <= 1e-9;
            const bool hit_horizontal = std::abs(ty - t) <= 1e-9;

            if (hit_vertical)
                dx = -dx;
            if (hit_horizontal)
                dy = -dy;

            // If neither compares equal due to tolerance weirdness, fall back:
            if (!hit_vertical && !hit_horizontal) {
                // Compare actual proximity to walls.
                const double dist_left = std::abs(x - 0.0);
                const double dist_right = std::abs(x - length_);
                const double dist_bottom = std::abs(y - 0.0);
                const double dist_top = std::abs(y - width_);
                const double min_dist = std::min({dist_left, dist_right, dist_bottom, dist_top});
                if (min_dist == dist_left || min_dist == dist_right)
                    dx = -dx;
                if (min_dist == dist_bottom || min_dist == dist_top)
                    dy = -dy;
            }
        }

        return hits;
    }

private:
    double length_;  //< length of the table
    double width_;   //< width of the table
};

int main() {
    AirHockey game(/*length=*/2.0, /*width=*/1.0);

    Point start{0.3, 0.2};
    const auto hits = game.puck_hit_locations(start, /*angle_deg=*/0.);

    std::cout << "First " << hits.size() << " wall contacts:\n";
    for (std::size_t i = 0; i < hits.size(); ++i) {
        std::cout << (i + 1) << ": " << hits[i] << "\n";
    }
    return 0;
}
