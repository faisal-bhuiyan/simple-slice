#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "slicer/mesh_slicer.hpp"
#include "slicer/shapes.hpp"
#include "slicer/stl_reader.hpp"
#include "slicer/toolpath.hpp"

int main(int argc, char** argv) {
    using simple_slice::slicer::format_toolpath_gcode;
    using simple_slice::slicer::read_ascii_stl_file;
    using simple_slice::slicer::slice_triangle_mesh_layers;

    //----------------------------------------------------
    // STL mesh slicing demo
    // Edit the values below to change the output geometry
    //----------------------------------------------------

    constexpr double layer_height_mm{0.2};  // layer height (mm)
    constexpr double spacing_mm{2.};        // perimeter spacing (mm)
    const std::string default_stl_path{"src/apps/cube_sample.stl"};

    if (layer_height_mm <= 0.) {
        std::cerr << "Layer height must be positive.\n";
        return 1;
    }
    if (spacing_mm <= 0.) {
        std::cerr << "Spacing must be positive.\n";
        return 1;
    }

    //----------------------------------------------------
    // Load STL mesh
    //----------------------------------------------------

    std::string stl_path = (argc > 1) ? argv[1] : default_stl_path;
    std::vector<simple_slice::slicer::Triangle> triangles = read_ascii_stl_file(stl_path);

    // Try alternative paths if the default path fails (e.g., when running from build/)
    if (triangles.empty() && stl_path == default_stl_path) {
        const std::vector<std::string> alternative_paths{
            "../src/apps/cube_sample.stl",  // when running from build/
            "cube_sample.stl"               // when running from src/apps/
        };
        for (const auto& alt_path : alternative_paths) {
            triangles = read_ascii_stl_file(alt_path);
            if (!triangles.empty()) {
                stl_path = alt_path;
                break;
            }
        }
    }

    if (triangles.empty()) {
        std::cerr << "Failed to read ASCII STL from: " << stl_path << "\n";
        return 1;
    }

    //----------------------------------------------------
    // Slice mesh and output G-code
    //----------------------------------------------------

    const auto layers = slice_triangle_mesh_layers(triangles, layer_height_mm);

    const std::string output_filename{"slicer_mesh.gcode"};
    std::ofstream gcode_out(output_filename);
    if (!gcode_out) {
        std::cerr << "Failed to open " << output_filename << " for writing.\n";
        return 1;
    }
    gcode_out << format_toolpath_gcode(layers);

    return 0;
}
