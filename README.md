# simple-slice

A simple slicing library for 3D geometries, very much work in progress.

This repository is currently a sandbox for developing a slicing library for 3D geometries. It currently contains a simple demo executable that simulates an air hockey game and prints the trajectory of the puck as it bounces off the walls. The software development approach provides a window into my code writing process highlighting clean coding, readability, maintainability, and testing.

## Roadmap

- Add more computational geometry logic in src/geometry/
- Add slicing logic in src/slicer/

## Build

This project uses CMake and builds:
- `air_hockey_game` demo executable
- unit tests for the geometry library

## Instructions

```bash
# From the repository root, build the project with:
cmake -S . -B build
cmake --build build
```

## Run

```bash
# From the repository root, run the demo executable:
./build/air_hockey_game
```

## Test

```bash
# From the repository root, run the unit tests:
ctest --test-dir build --output-on-failure
```