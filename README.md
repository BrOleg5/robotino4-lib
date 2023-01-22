# Robotino4 library

There is Festo [Robotino API2](https://wiki.openrobotino.org/index.php?title=API2) wrapper.

## Requirements

- CMake v3.0 or later
- Robotino API2 v1.1.14 or later
- compiler: gcc 9.3.0 or later; MSVC 19 (Visual Studio 2017) or later

## Configure, build and install library

```bash
# Create build directory
mkdir build/

# Configure
cmake -S robotino4-lib/ -B build/

# Build certain configuration <cfg>: Debug (default), Release
cmake --build build/ --config <cfg>

# Install certain configuration <cfg>: Debug (default), Release
# <prefix> is installation path
sudo cmake --install build/ --config <cfg> --prefix <prefix>
```

### Extra options

You can build and install example programs. For this add option `-D BUILD_EXAMPLES=ON`.

```bash
# Configure
cmake -S ./robotino4-lib/ -B ./build/ -BUILD_EXAMPLES=ON
```

## Using Robotino4 with CMake

Add this strings in your CMakeLists.txt file:

```CMake
find_package(Robotino4Wrapper 1.3.0 REQUIRED)
target_link_libraries(<ProjectName> robotino4)
# if nessesary, add include directories to target
target_include_directories(<ProjectName> ${Robotino4Wrapper_INCLUDE_DIRS})
```
