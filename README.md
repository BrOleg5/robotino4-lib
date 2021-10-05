# Robotino4 library

There is Festo [Robotino API2](https://wiki.openrobotino.org/index.php?title=API2) wrapper.

## Requirements

- CMake v3.16 or later
- Robotino API2 v1.1.14 or later
- gcc compiler 9.3.0 or later

## Building and install the library

### Configure Robotino4 library as static library

```
# Create build directory
mkdir build

# Configure
cmake -S robotino4-lib/ -B build/ -DCMAKE_INSTALL_PREFIX="<path_to_install>"
```

### Configure the library as shared library

```
# Create build directory
mkdir build

# Configure
cmake -S robotino4-lib/ -B build/ -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX="<path_to_install>"
```

### Build Robotino4 library

```
# Build
cmake --build build/
```

### Install Robotino4 library

```
cmake --install build/
```

Also you can build and install library at the same time:
```
cmake --build build/ --target install
```