## Overview

This is an example project showcasing usage of my platform independent [MPU6050 Driver](https://github.com/KadePerrotti/MPU6050-Driver) on Arduino. Within aduino_example.ino the MPU6050 is initialized, then all 6 axes are repeatedly sampled and printed to serial in the Arduino Plotter format.

## Arduino Caveats
The string builder functions from TEST_FUNCTIONS appear to use too much memory on arduinos and cause random chars to be printed. A fix is in progress.

## Arduino Setup with ArduinoIDE


1. Create your Arduino project
2. Within the directory your .ino file is in, create the directory `src/`.
3. Clone the [MPU6050 Driver](https://github.com/KadePerrotti/MPU6050-Driver) into the `src/` directory.
4. Include the required files in your project:
```c
#include "src/MPU6050-Driver/MPU6050.h"
#include "src/MPU6050-Driver/REG_OPTIONS.h"
#include "src/MPU6050-Driver/TEST_FUNCTIONS.h" //not necessary, only include if you want to run tests
```
5. The build_string functions in TEST_FUNCTIONS use sprintf to include floats in the strings. Make sure your linker flags include `-u _printf_float` if you're trying to use the build_string functions. 