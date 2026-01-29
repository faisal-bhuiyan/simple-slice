#include <gtest/gtest.h>

#include "slicer/perimeters.hpp"
#include "slicer/shapes.hpp"
#include "slicer/toolpath.hpp"

using namespace simple_slice::slicer;

// Test fixture for toolpath tests
class ToolpathTest : public ::testing::Test {
protected:
    static constexpr double kTestEpsilon = 1e-9;
};

//----------------------------------------------
// Format toolpath tests
//----------------------------------------------

TEST_F(ToolpathTest, OutputFormat) {
    const Rectangle rectangle(0., 0., 4., 4.);
    const auto paths = generate_rectangle_perimeters(rectangle, 2.);
    const std::string output = format_toolpath_gcode(paths);

    ASSERT_FALSE(output.empty());
    EXPECT_NE(output.find("G0"), std::string::npos);
    EXPECT_NE(output.find("G1"), std::string::npos);

    // Each path emits a G0 + G1 lines for the remaining points
    const std::size_t expected_lines = paths[0].size();
    std::size_t line_count = 0;
    for (char ch : output) {
        if (ch == '\n') {
            ++line_count;
        }
    }
    EXPECT_GE(line_count, expected_lines);
}

TEST_F(ToolpathTest, FormatWithPrecision) {
    Path path;
    path.emplace_back(1.23456789, 9.87654321, 0.);
    path.emplace_back(2.34567890, 8.76543210, 0.);

    const std::string output_default = format_toolpath_gcode({path}, 16);
    const std::string output_precision2 = format_toolpath_gcode({path}, 2);

    // With precision 16, numbers are formatted with trailing zeros
    // Check that the coordinate values appear in the output (check for prefix)
    EXPECT_NE(output_default.find("X1.234567"), std::string::npos);  // Check X coordinate prefix
    EXPECT_NE(output_default.find("Y9.876543"), std::string::npos);  // Check Y coordinate prefix

    // Precision 2 should round to 2 decimal places
    EXPECT_NE(output_precision2.find("X1.23"), std::string::npos);
    EXPECT_NE(output_precision2.find("Y9.88"), std::string::npos);  // 9.87654321 rounds to 9.88

    // Verify precision 2 doesn't have more than 2 decimal places
    std::size_t dot_pos = output_precision2.find("X1.23");
    if (dot_pos != std::string::npos) {
        std::size_t space_pos = output_precision2.find(" ", dot_pos);
        if (space_pos != std::string::npos) {
            std::string x_value = output_precision2.substr(dot_pos + 1, space_pos - dot_pos - 1);
            // Count decimal places after the dot
            std::size_t dot_in_value = x_value.find(".");
            if (dot_in_value != std::string::npos) {
                std::string decimals = x_value.substr(dot_in_value + 1);
                EXPECT_LE(decimals.length(), 2U);
            }
        }
    }
}

TEST_F(ToolpathTest, FormatEmptyPaths) {
    const std::vector<Path> empty_paths{};
    const std::string output = format_toolpath_gcode(empty_paths);

    EXPECT_TRUE(output.empty());
}

TEST_F(ToolpathTest, FormatEmptyPathInVector) {
    Path empty_path;
    const std::vector<Path> paths{empty_path};
    const std::string output = format_toolpath_gcode(paths);

    // Empty paths should be skipped
    EXPECT_TRUE(output.empty());
}

TEST_F(ToolpathTest, FormatSinglePointPath) {
    Path path;
    path.emplace_back(1., 2., 0.);

    const std::string output = format_toolpath_gcode({path});

    // Single point should emit G0 only
    EXPECT_NE(output.find("G0"), std::string::npos);
    EXPECT_EQ(output.find("G1"), std::string::npos);
}

TEST_F(ToolpathTest, FormatMultiplePaths) {
    Path path1;
    path1.emplace_back(0., 0., 0.);
    path1.emplace_back(1., 0., 0.);

    Path path2;
    path2.emplace_back(2., 2., 0.);
    path2.emplace_back(3., 2., 0.);
    path2.emplace_back(3., 3., 0.);

    const std::string output = format_toolpath_gcode({path1, path2});

    // Should contain G0 commands for each path start
    std::size_t g0_count = 0;
    std::size_t pos = 0;
    while ((pos = output.find("G0", pos)) != std::string::npos) {
        ++g0_count;
        pos += 2;
    }
    EXPECT_EQ(g0_count, 2U);
}

TEST_F(ToolpathTest, FormatNegativePrecision) {
    Path path;
    path.emplace_back(1.5, 2.5, 0.);

    // Negative precision should be clamped to 0
    const std::string output = format_toolpath_gcode({path}, -5);

    EXPECT_FALSE(output.empty());
    // Should still format correctly with precision 0
    EXPECT_NE(output.find("G0"), std::string::npos);
}

//----------------------------------------------
// Format layers tests
//----------------------------------------------

TEST_F(ToolpathTest, FormatLayersWithZMoves) {
    Path path1;
    path1.emplace_back(0., 0., 0.);
    path1.emplace_back(1., 0., 0.);

    Path path2;
    path2.emplace_back(0., 0., 0.);
    path2.emplace_back(0., 1., 0.);

    Layer layer1{0.2, {path1}};
    Layer layer2{0.4, {path2}};

    const std::string output = format_toolpath_gcode({layer1, layer2});

    // Should contain Z moves
    EXPECT_NE(output.find("G0 Z0.2"), std::string::npos);
    EXPECT_NE(output.find("G0 Z0.4"), std::string::npos);
}

TEST_F(ToolpathTest, FormatLayersEmpty) {
    const std::vector<Layer> empty_layers{};
    const std::string output = format_toolpath_gcode(empty_layers);

    EXPECT_TRUE(output.empty());
}

TEST_F(ToolpathTest, FormatLayersWithEmptyPaths) {
    Layer layer{0.2, {}};
    const std::string output = format_toolpath_gcode({layer});

    // Should still emit Z move even if no paths
    EXPECT_NE(output.find("G0 Z0.2"), std::string::npos);
}

TEST_F(ToolpathTest, FormatLayersPrecision) {
    Path path;
    path.emplace_back(1.23456789, 2.34567890, 0.);

    Layer layer{0.123456789, {path}};
    const std::string output = format_toolpath_gcode({layer}, 3);

    // Z coordinate should use specified precision
    EXPECT_NE(output.find("G0 Z0.123"), std::string::npos);
}
