# 2D Slicer Demo

This demo generates 2D perimeter toolpaths for a rectangle and a circle and
writes G-code-like output to separate files.

## Build

From the repo root:

- `cmake -S . -B build`
- `cmake --build build`

## Run

From the repo root:

- `./build/slicer2d_main`

This will write:

- `slicer2d_rectangle.gcode`
- `slicer2d_circle.gcode`

## Customize the shapes

Edit the parameters in `src/apps/slicer2d_main.cpp`:

- Rectangle bounds (`rect_min_x`, `rect_min_y`, `rect_max_x`, `rect_max_y`)
- Circle parameters (`circle_center_x`, `circle_center_y`, `circle_radius`)
- Toolpath spacing (`spacing`)
- Circle polygon resolution (`segments`)

## Output preview

### Rectangle

![Rectangle toolpath](images/rectangle_toolpath.png)

### Circle

![Circle toolpath](images/circle_toolpath.png)


