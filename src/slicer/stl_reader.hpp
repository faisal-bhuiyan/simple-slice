#pragma once

#include <fstream>
#include <istream>
#include <string>
#include <vector>

#include "slicer/shapes.hpp"

namespace simple_slice::slicer {

/**
 * @brief Parse an ASCII STL stream into triangles.
 *
 * Intent: read a minimal ASCII STL (facet/vertex blocks) and extract triangles.
 *
 * @param input Input stream containing ASCII STL data
 * @return List of triangles parsed from the stream
 */
inline std::vector<Triangle> parse_ascii_stl(std::istream& input) {
    std::vector<Triangle> triangles{};

    std::string token;
    Point vertices[3]{Point{}, Point{}, Point{}};  // three vertices of a triangle
    std::size_t vertex_count{0};
    while (input >> token) {
        // ASCII STL lines with geometry are prefixed by "vertex"
        if (token == "vertex") {
            double x{0.};
            double y{0.};
            double z{0.};
            if (!(input >> x >> y >> z)) {
                break;
            }

            // Collect exactly three vertices per triangle
            if (vertex_count < 3) {
                vertices[vertex_count] = Point{x, y, z};
                ++vertex_count;
            }
            if (vertex_count == 3) {
                // Emit a triangle and reset for the next facet
                triangles.emplace_back(vertices[0], vertices[1], vertices[2]);
                vertex_count = 0;
            }
        }
    }
    return triangles;
}

/**
 * @brief Read an ASCII STL file into triangles
 *
 * Intent: convenience wrapper around parse_ascii_stl().
 *
 * @param path Path to an ASCII STL file
 * @return List of triangles parsed from the file (empty on failure)
 */
inline std::vector<Triangle> read_ascii_stl_file(const std::string& path) {
    std::ifstream file(path);
    if (!file) {
        return {};
    }
    return parse_ascii_stl(file);
}

}  // namespace simple_slice::slicer
