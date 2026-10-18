# Project

This project focuses on extracting positions from a GCode file, interpolating the positions using an S-curve trajectory for a 5-axis CNC machine and getting the forward and inverse kinematics governed by the given machine parameters.

## Machine Parameters

| Axis | Type | Position Range | Max. Velocity | Max. Acceleration |
| --- | --- | --- | --- | --- |
| **X** | Linear (mm) | -150 to +150 mm | 250 mm/s | 1510 mm/s² |
| **Y** | Linear (mm) | -180 to +180 mm | 500 mm/s | 1390 mm/s² |
| **Z** | Linear (mm) | -180 to 0 mm | 250 mm/s | 2340 mm/s² |
| **A** | Rotary (°) | -110° to +110° | 461 °/s | 9220 °/s² |
| **C** | Rotary (°) | -1e99° to 1e99° | 720 °/s | 7200 °/s² |