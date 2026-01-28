#pragma once

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "slicer/shapes.hpp"

namespace simple_slice::slicer {

/**
 * @brief Format polylines into a G-code-like string (G0/G1)
 *
 * Each path emits:
 *  - G0 to the first point
 *  - G1 for each subsequent point
 *
 * Intent: emit a minimal, deterministic toolpath for a single 2D layer.
 *
 * @param paths Polylines to format
 * @param precision Decimal precision for coordinates (>= 0)
 */
inline std::string format_toolpath_gcode(const std::vector<Path>& paths, int precision = 16) {
    std::ostringstream out;
    if (precision < 0) {
        precision = 0;
    }
    out << std::fixed << std::setprecision(precision);

    // Emit the toolpath for each path
    for (const auto& path : paths) {
        if (path.empty()) {
            continue;
        }

        // Emit the start point of the path
        const auto& start = path.front();
        out << "G0 X" << start.GetX() << " Y" << start.GetY() << "\n";

        // Emit the subsequent points of the path
        for (std::size_t i = 1; i < path.size(); ++i) {
            const auto& p = path[i];
            out << "G1 X" << p.GetX() << " Y" << p.GetY() << "\n";
        }
    }
    return out.str();
}

/**
 * @brief Format layered polylines into a G-code-like string (G0/G1 with Z).
 *
 * Each layer emits:
 *  - G0 Z<z> to set height
 *  - G0 to the first point of each path
 *  - G1 for each subsequent point
 *
 * Intent: emit a minimal toolpath for multiple layers, inserting Z moves
 * between layer toolpaths.
 *
 * @param layers Layered paths to format
 * @param precision Decimal precision for coordinates (>= 0)
 */
inline std::string format_toolpath_gcode(const std::vector<Layer>& layers, int precision = 16) {
    std::ostringstream out;
    if (precision < 0) {
        precision = 0;
    }
    out << std::fixed << std::setprecision(precision);

    // Emit the toolpath for each layer
    for (const auto& layer : layers) {
        out << "G0 Z" << layer.z << "\n";
        out << format_toolpath_gcode(layer.paths, precision);
    }

    return out.str();
}

}  // namespace simple_slice::slicer
