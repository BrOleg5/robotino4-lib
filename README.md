# Robotino4 library

There is Festo [Robotino API2](https://wiki.openrobotino.org/index.php?title=API2) wrapper.

## Requirements

- CMake v3.0 or later
- Robotino API2 v1.1.14 or later
- compiler: gcc 9.3.0 or later

## Configure, build and install library

### Ubuntu 20.04

```
# Create build directory
mkdir build/

# Configure
cmake -S robotino4-lib/ -B build/

# Build certain configuration <cfg>: Debug (default), Release
cmake --build build/ --config <cfg>

# Install certain configuration <cfg>: Debug (default), Release
# <prefix> is installation path (default /usr/local)
sudo cmake --install build/ --config <cfg> --prefix <prefix>
```

### Windows 10

#### x64

```
# Create build directory
mkdir .\build\

# Configure. Build system generator <generator-name>: for MSVC 19 (Visual Studio 2017) is "Visual Studio 15 Win64".
# Command "cmake --help" print full lust of generators that are available on your platform
cmake -S .\robotino4-lib\ -B .\build\ -G <generator-name>

# Build certain configuration <cfg>: Debug (default), Release
cmake --build .\build\ --config <cfg>

# Install certain library configuration <cfg>: Debug (default), Release
# <prefix> is installation path (default SystemPartition:\Program Files (x86)\<project name>)
cmake --install .\build\ --config <cfg> --prefix <prefix>
```

### Extra options

You can build and install test programs. For this add option `-D BUILD_TESTS=ON`.
```
# Configure
cmake -S robotino4-lib/ -B build/ -DBUILD_TESTS=ON
```

## Using Robotino4 with gcc and CMake

Add this strings in your CMakeLists.txt file:
```
find_package(Robotino4 1.2 REQUIRED)
target_link_libraries(<ProjectName> Robotino4Lib)
# if nessesary, add include directories to target
target_include_directories(<ProjectName> ${Robotino4_INCLUDE_DIRS})
```