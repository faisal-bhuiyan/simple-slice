#include "stl_io.hpp"

#include <array>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

namespace simple_slice::mesh {

std::vector<Triangle> parse_ascii_stl(std::istream& input) {
    std::vector<Triangle> triangles{};

    // Streaming parser state: collect three vertices per triangle
    std::array<Point, 3> triangle_vertices{};  // three vertices of a triangle
    std::size_t triangle_vertex_count{0};      // collected vertices count for current triangle
    double coord_buffer[3]{};                  // buffer for x, y, z of a vertex
    std::size_t coord_index{0};                // index into coord_buffer
    std::size_t remaining_coords{0};  // number of coordinates left to read for the current vertex

    // Read chunks to avoid loading the full file in memory
    std::string buffer;
    buffer.reserve(1 << 16);  // 64KB buffer
    char chunk[1 << 16];      // 64KB chunk

    // Lambda: consume tokens from the buffer and parse triangles
    auto consume_buffer = [&](bool eof) {
        std::size_t cursor = 0;
        const std::size_t buffer_size = buffer.size();

        // Lambda: skip whitespace and advance the cursor to the start of the next token
        auto skip_space = [&]() {
            while (cursor < buffer_size &&
                   std::isspace(static_cast<unsigned char>(buffer[cursor]))) {
                ++cursor;
            }
        };

        // Main token parsing loop: extract tokens and look for "vertex" to identify geometry lines
        while (cursor < buffer_size) {
            skip_space();
            if (cursor >= buffer_size) {
                break;
            }

            // Token starts at the current cursor position; find the end of the token by looking for
            // the next whitespace
            const std::size_t token_start = cursor;
            while (cursor < buffer_size &&
                   !std::isspace(static_cast<unsigned char>(buffer[cursor]))) {
                ++cursor;
            }

            // If not at EOF, we may have an incomplete token; keep it in the buffer for next chunk
            if (cursor == buffer_size && !eof) {
                buffer.erase(0, token_start);
                return;
            }

            const std::size_t token_len = cursor - token_start;

            // If we're in the middle of parsing vertex coordinates, parse the token as a number
            if (remaining_coords > 0) {
                // Parse coordinate value
                char saved = buffer[cursor];
                buffer[cursor] = '\0';
                char* endptr = nullptr;
                double value = std::strtod(buffer.data() + token_start, &endptr);
                buffer[cursor] = saved;

                if (endptr != buffer.data() + cursor) {
                    // Malformed number -> stop parsing
                    buffer.erase(0, token_start);
                    return;
                }

                // Store the parsed coordinate value in the buffer
                coord_buffer[coord_index++] = value;
                if (--remaining_coords == 0) {
                    triangle_vertices[triangle_vertex_count++] =
                        Point{coord_buffer[0], coord_buffer[1], coord_buffer[2]};
                    coord_index = 0;

                    // If we have collected three vertices -> emit a triangle and reset
                    if (triangle_vertex_count == 3) {
                        triangles.push_back(
                            {triangle_vertices[0], triangle_vertices[1], triangle_vertices[2]}
                        );
                        triangle_vertex_count = 0;
                    }
                }
            } else if (token_len == 6 &&
                       std::memcmp(buffer.data() + token_start, "vertex", 6) == 0) {
                // Geometry lines are prefixed by "vertex" -> parse the next three tokens as x y z
                remaining_coords = 3;
                coord_index = 0;
            }
        }
        // All tokens consumed -> clear the buffer
        buffer.clear();
    };

    // Read the input stream in chunks and consume tokens to parse triangles
    while (input.read(chunk, sizeof(chunk)), input.gcount() > 0) {
        buffer.append(chunk, static_cast<std::size_t>(input.gcount()));
        consume_buffer(false);
    }

    // Flush any remaining tokens at EOF
    consume_buffer(true);

    return triangles;
}

void write_triangle_in_ascii_stl(std::ostream& out, const Triangle& t) {
    // Compute edge vectors
    double v1[3]{t.b[0] - t.a[0], t.b[1] - t.a[1], t.b[2] - t.a[2]};
    double v2[3]{t.c[0] - t.a[0], t.c[1] - t.a[1], t.c[2] - t.a[2]};

    // Cross product to get normal
    double nx = v1[1] * v2[2] - v1[2] * v2[1];
    double ny = v1[2] * v2[0] - v1[0] * v2[2];
    double nz = v1[0] * v2[1] - v1[1] * v2[0];

    // Emit facet
    out << "  facet normal " << nx << ' ' << ny << ' ' << nz << '\n';
    out << "    outer loop\n";

    out << "      vertex " << t.a[0] << ' ' << t.a[1] << ' ' << t.a[2] << '\n';
    out << "      vertex " << t.b[0] << ' ' << t.b[1] << ' ' << t.b[2] << '\n';
    out << "      vertex " << t.c[0] << ' ' << t.c[1] << ' ' << t.c[2] << '\n';

    out << "    endloop\n";
    out << "  endfacet\n";
}

void write_ascii_stl(
    std::ostream& out, std::string_view solid_name, std::span<const Triangle> triangles
) {
    out << "solid " << solid_name << '\n';
    // Emit the triangles
    for (const Triangle& t : triangles) {
        write_triangle_in_ascii_stl(out, t);
    }
    out << "endsolid " << solid_name << '\n';
}

void convert_binary_stl_to_ascii(const std::string& binary_path, const std::string& ascii_path) {
    std::ifstream in(binary_path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Failed to open binary STL: " + binary_path);
    }

    std::ofstream out(ascii_path);
    if (!out) {
        throw std::runtime_error("Failed to open ASCII STL: " + ascii_path);
    }

    // Skip 80-byte header
    char header[80];
    in.read(header, 80);

    // Read triangle count
    std::uint32_t num_triangles{0};
    in.read(reinterpret_cast<char*>(&num_triangles), sizeof(num_triangles));

    out << "solid converted\n";
    for (std::uint32_t i{0}; i < num_triangles; ++i) {
        float normal[3];
        float v[9];
        std::uint16_t attr;

        in.read(reinterpret_cast<char*>(normal), sizeof(normal));
        in.read(reinterpret_cast<char*>(v), sizeof(v));
        in.read(reinterpret_cast<char*>(&attr), sizeof(attr));

        out << "  facet normal 0 0 0\n";
        out << "    outer loop\n";
        out << "      vertex " << v[0] << " " << v[1] << " " << v[2] << "\n";
        out << "      vertex " << v[3] << " " << v[4] << " " << v[5] << "\n";
        out << "      vertex " << v[6] << " " << v[7] << " " << v[8] << "\n";
        out << "    endloop\n";
        out << "  endfacet\n";
    }
    out << "endsolid converted\n";
}

}  // namespace simple_slice::mesh
