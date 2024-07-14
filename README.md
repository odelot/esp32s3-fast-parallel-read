# ESP32S3 Fast Parallel Read

This code provides a straightforward example of how to read 16 bits in parallel using DMA on the ESP32S3.

Please note that this example is simplified and some shortcuts were taken for the sake of testing and understanding the functionality.

Currently, the code does not use a queue to send the data read from the GDMA interrupt. During testing, no issues have been encountered when reading from GDMA memory.

## How to Use/Test the Code

The goal is to read data in parallel quickly and without CPU load using General Purpose DMA (GDMA).

This example demonstrates how to use the camera peripheral to read data in parallel and transfer it to memory using GDMA.

The speed is set to a low rate (157,480 Hz) to make the code easier to understand and debug. If you want to increase the read speed by adjusting the clock configuration and decreasing the div_n value, you will need to modify the number of DMA descriptors and increase the size of each descriptor. Additionally, you will need to remove any code used for debugging and testing.

The configuration of cam_bit_order and cam_byte_order, combined with the fact that the ESP32 is a little-endian chip, results in the data being read as unsigned words. In this configuration, the first input pin (GPIO_NUM_1) is read as the most significant bit of the word.

Example with this code configuration (cam_bit_order = 1 and cam_byte_order = 0):
```
  16 bit word to read: 0x0F51 (or 3921 in decimal)
    Binary to be read: 0   0   0   0   1   1   1   1   0   1   0   1   0   0   0   1 
            GPIO pins: 1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16
```

## Folder contents

The project contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```

## License

CC Attribution-ShareAlike 4.0 International
