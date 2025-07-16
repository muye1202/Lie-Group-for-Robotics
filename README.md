# Lie-Group-for-Robotics

This repository provides a small C++ library implementing Lie group and Lie algebra utilities commonly used in robotics. The library comes with a simple differential drive example demonstrating how the components fit together.

## Requirements

- C++17 compatible compiler (e.g. `g++`)
- [Eigen3](https://eigen.tuxfamily.org/) (>= 3.4)
- [nlohmann/json](https://github.com/nlohmann/json)
- CMake >= 3.10
- Python 3 (optional, for plotting the example results)

On Ubuntu the required packages can be installed with:

```bash
sudo apt-get install cmake g++ libeigen3-dev nlohmann-json3-dev python3 python3-pip
```

## Building

Create a build directory and compile the library and example:

```bash
mkdir build
cd build
cmake ..
make
```

This builds the shared library `liblie_lib` and the example executable `demo_lie_lib`.

## Running the Example

From the `build` directory run:

```bash
./demo_lie_lib
```

The program reads parameters from `diffdrive_example/diffdrive.json` and writes a trajectory to `diffdrive_example/robot_pos.json`.

To plot the resulting path with Python:

```bash
python3 ../diffdrive_example/plot.py
```

Edit the file `diffdrive_example/plot.py` if your paths differ.

## Repository Layout

- `include/lie_lib` – Header files for the Lie group utilities
- `src/` – Implementation files
- `diffdrive_example/` – Simple differential drive demo and plotting script
- `CMakeLists.txt` – Build configuration
