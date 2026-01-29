# 2D and 3D Slicer Demos

This repository contains demos for generating perimeter toolpaths:
- **2D Slicer**: Generates toolpaths for rectangles and circles
- **3D Slicer**: Slices triangle meshes (STL files) into layers and generates toolpaths

## Build

From the repo root:

- `cmake -S . -B build`
- `cmake --build build`

## 2D Slicer

This demo generates 2D perimeter toolpaths for a rectangle and a circle and writes G-code-like output to separate files.

### Run

From the repo root:

- `./build/slicer2d_main`

This will write:

- `slicer2d_rectangle.gcode`
- `slicer2d_circle.gcode`

### Customize the shapes

Edit the parameters in `src/apps/slicer2d_main.cpp`:

- Rectangle bounds (`rect_min_x`, `rect_min_y`, `rect_max_x`, `rect_max_y`)
- Circle parameters (`circle_center_x`, `circle_center_y`, `circle_radius`)
- Toolpath spacing (`spacing`)
- Circle polygon resolution (`segments`)

### Output preview

#### Rectangle

![Rectangle toolpath](images/rectangle_toolpath.png)

#### Circle

![Circle toolpath](images/circle_toolpath.png)

## 3D Slicer

This demo slices 3D triangle meshes (STL files) into horizontal layers and generates G-code toolpaths.

### Run

From the repo root:

- `./build/slicer_main [stl_file]`

**Arguments**:
- `stl_file`: Path to ASCII STL file (optional, default: `src/apps/cube_sample.stl`)

**Examples**:
- `./build/slicer_main` - Uses default cube STL with default parameters
- `./build/slicer_main my_model.stl` - Load custom STL file

This will write:

- `slicer_mesh.gcode`

### Customize Parameters

Edit the default values in `src/apps/slicer_main.cpp`:

- `layer_height_mm`: Vertical spacing between layers (default: 0.50 mm)
- `spacing_mm`: Horizontal spacing between perimeters (default: 0.1 mm)
- `default_stl_path`: Default STL file path

### Output preview

#### Cube Mesh

![Cube mesh](images/cube_mesh.png)

#### Cube Toolpath

![Cube toolpath](images/cube_toolpath.png)

## Capabilities and Limitations

### What the Slicer Can Do

- **2D Shapes**: Generate perimeter toolpaths for rectangles and circles
- **3D Mesh Slicing**: Slice triangle meshes from ASCII STL files into horizontal layers
- **Contour Extraction**: Extract outer contours from mesh intersections with horizontal planes
- **Perimeter Generation**: Generate inward-offset rectangular perimeters for each layer
- **G-code Output**: Export toolpaths in G-code-like format (G0/G1 commands with X, Y, Z coordinates)

### Limitations

- **STL Format**: Only supports ASCII STL files (binary STL not supported)
- **Simplified Perimeters**: Perimeters use axis-aligned bounding boxes rather than following exact mesh contours. This means perimeters may extend beyond the actual shape boundaries.
- **Horizontal Slicing Only**: Fixed horizontal layer heights (no adaptive layer heights or angled slicing)
- **Coplanar Handling**: Triangles or edges that lie exactly on a slicing plane are ignored to avoid ambiguous segments
- **Basic G-code**: Outputs minimal G-code (G0/G1 moves only, no temperature, speed, or other printer settings)

