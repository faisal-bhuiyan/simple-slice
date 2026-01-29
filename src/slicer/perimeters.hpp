#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "slicer/shapes.hpp"

namespace simple_slice::slicer {

/**
 * @brief Generate inward offset rectangle perimeters
 *
 * Each perimeter is a closed polyline with 5 points (start repeats at end).
 * Spacing is the inward offset between successive perimeters.
 *
 * Intent: approximate a 3D-printer perimeter toolpath for a rectangular layer
 * by emitting concentric rectangular loops from the outer boundary inward.
 *
 * @param rectangle Axis-aligned rectangle bounds (XY plane)
 * @param spacing Inward offset step between successive perimeters (typically by the nozzle width)
 */
inline std::vector<Path> generate_rectangle_perimeters(const Rectangle& rectangle, double spacing) {
    std::vector<Path> paths{};

    // Early return if spacing is not positive
    if (spacing <= 0.) {
        return paths;
    }

    // Get the rectangle bounds
    double min_x = rectangle.min_x;
    double min_y = rectangle.min_y;
    double max_x = rectangle.max_x;
    double max_y = rectangle.max_y;

    // Generate perimeters until the rectangle is exhausted
    while (min_x < max_x && min_y < max_y) {
        Path path{};
        path.reserve(5);                      // 5 points for a rectangle
        path.emplace_back(min_x, min_y, 0.);  // bottom left
        path.emplace_back(max_x, min_y, 0.);  // bottom right
        path.emplace_back(max_x, max_y, 0.);  // top right
        path.emplace_back(min_x, max_y, 0.);  // top left
        path.emplace_back(min_x, min_y, 0.);  // bottom left
        paths.emplace_back(std::move(path));

        // Move the rectangle inward by the spacing for the next perimeter
        min_x += spacing;
        min_y += spacing;
        max_x -= spacing;
        max_y -= spacing;
    }

    return paths;
}

/**
 * @brief Generate inward offset circle perimeters
 *
 * Each perimeter is a closed polyline with (segments + 1) points.
 * Spacing is the inward radial offset between successive perimeters.
 *
 * Intent: approximate a 3D-printer perimeter toolpath for a circular layer
 * by emitting concentric circular loops from the outer boundary inward.
 *
 * @param circle Circle center and radius (XY plane)
 * @param spacing Inward radial offset between successive perimeters
 * @param num_segments Number of line segments per perimeter (>= 3)
 */
inline std::vector<Path> generate_circle_perimeters(
    const Circle& circle, double spacing, std::size_t num_segments
) {
    std::vector<Path> paths{};

    // Early return if spacing is not positive or num_segments is less than 3
    if (spacing <= 0. || num_segments < 3) {
        return paths;
    }

    // Generate perimeters until the circle is exhausted
    double radius = circle.radius;
    while (radius > 0.) {
        Path path{};
        path.reserve(num_segments + 1);

        // Generate the points for the perimeter
        for (std::size_t i = 0; i < num_segments; ++i) {
            const double theta =
                2. * std::numbers::pi * static_cast<double>(i) / static_cast<double>(num_segments);
            const double x = circle.center_x + radius * std::cos(theta);
            const double y = circle.center_y + radius * std::sin(theta);
            path.emplace_back(x, y, 0.);
        }
        path.emplace_back(path.front());  // close the path
        paths.emplace_back(std::move(path));

        radius -= spacing;
    }

    return paths;
}

//----------------------------------------------------
// Layer perimeter generation
//----------------------------------------------------

/**
 * @brief Compute 2D bounding box from all paths in a layer
 *
 * Finds the axis-aligned bounding rectangle that contains all points
 * from all paths in the layer (XY plane only).
 *
 * @param layer Layer containing paths
 * @return Rectangle bounding box (min_x, min_y, max_x, max_y)
 */
inline Rectangle compute_layer_bounding_box(const Layer& layer) {
    if (layer.paths.empty()) {
        return Rectangle{0., 0., 0., 0.};
    }

    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& path : layer.paths) {
        for (const auto& point : path) {
            min_x = std::min(min_x, point.GetX());
            min_y = std::min(min_y, point.GetY());
            max_x = std::max(max_x, point.GetX());
            max_y = std::max(max_y, point.GetY());
        }
    }

    // Handle degenerate case where all points are the same
    if (min_x > max_x || min_y > max_y) {
        return Rectangle{0., 0., 0., 0.};
    }

    return Rectangle{min_x, min_y, max_x, max_y};
}

/**
 * @brief Generate perimeters for all paths in a layer
 *
 * Computes the bounding box of all paths in the layer and generates
 * rectangle perimeters from that bounding box.
 *
 * @param layer Input layer with outer contours
 * @param spacing Inward offset spacing
 * @return New layer with perimeter paths
 */
inline Layer generate_layer_perimeters(const Layer& layer, double spacing) {
    // Compute bounding box from all paths in the layer
    const Rectangle bbox = compute_layer_bounding_box(layer);

    // Generate rectangle perimeters
    const auto perimeter_paths = generate_rectangle_perimeters(bbox, spacing);

    return Layer{layer.z, perimeter_paths};
}

}  // namespace simple_slice::slicer
