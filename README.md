# ESP32S3 Fast Parallel Read

This code provides a simple example of how to read 16 bits in parallel using DMA on the ESP32S3 model.

Please note that this is a work in progress (WIP), and some shortcuts have been taken for testing and understanding the functionality.

Currently, the code does not use a queue to send the data read from the GDMA interrupt. So far, no issues have been encountered when reading from GDMA memory during tests.

## How to use 

The project was built and tested with ESP-IDF v5.2.1

WIP

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