# Air Hockey Game Demo

This executable simulates an ideal puck on a rectangular table and prints the
first 10 wall-contact points plus an ASCII table visualization.

## Build

From the repository root:

```bash
cmake -S . -B build
cmake --build build
```

## Run

```bash
./build/air_hockey_game
```

## Inputs (current defaults)

The demo values are set in `src/air_hockey_game/air_hockey_game_main.cpp`:
- Table size: length = 2.0, width = 2.0
- Initial position: (1.0, 1.0, 0.0)
- Angle: 45 degrees (CCW from +x)

## Example output

```
First 10 wall contacts in (x, y, z) format:
1: (2.00, 1.41, 0.00)
2: (0.59, 2.00, 0.00)
3: (0.00, 1.76, 0.00)
4: (2.00, 0.93, 0.00)
5: (0.00, 0.10, 0.00)
6: (0.24, 0.00, 0.00)
7: (2.00, 0.73, 0.00)
8: (0.00, 1.56, 0.00)
9: (1.07, 2.00, 0.00)
10: (2.00, 1.62, 0.00)

Table (ASCII):
+----------------------2------------------9------------------------------------+
|                                                                              |
|                                                                              |
|                                                                              |
3                                                                              |
|                                                                              |
8                                                                              0
|                                                                              |
|                                                                              1
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              4
|                                                                              |
|                                                                              7
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
|                                                                              |
5                                                                              |
+---------6--------------------------------------------------------------------+
Legend: digits = hit index (1..9, 0 = 10th)
```