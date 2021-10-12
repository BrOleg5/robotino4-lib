# Robotino4 library

There is Festo [Robotino API2](https://wiki.openrobotino.org/index.php?title=API2) wrapper.

## Requirements

- CMake v3.0 or later
- Robotino API2 v1.1.14 or later
- gcc compiler 9.3.0 or later

## Configure, build and install library

```
# Create build directory
mkdir build

# Configure as static library
cmake -S robotino4-lib/ -B build/

# Build
cmake --build build/

# Install
sudo cmake --install build/
```

You can also build and install shared library:
```
# Configure as shared library
cmake -S robotino4-lib/ -B build/ -DBUILD_SHARED_LIBS=ON
```

You can build and install tests:
```
# Configure as static library
cmake -S robotino4-lib/ -B build/ -DBUILD_TESTS=ON -DINSTALL_TESTS=ON
```

## Using Robotino4 with gcc and CMake

Add this strings in your CMakeLists.txt file:
```
find_package(Robotino4 1.2 REQUIRED)
target_link_libraries(<ProjectName> Robotino4Lib)
# if nessesary, add include directories to target
target_include_directories(<ProjectName> ${Robotino4_INCLUDE_DIRS})
```