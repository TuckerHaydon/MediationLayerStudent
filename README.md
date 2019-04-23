# Mediation Layer

## Project Structure

## Installation
### Prerequisites 
1. [Eigen](https://eigen.tuxfamily.org)
2. [ROS](http://www.ros.org)
3. [P4 Requirements](https://github.com/tuckerhaydon/P4.git)
3. sudo apt install gnuplot

### Clone
```bash
git clone https://github.com/tuckerhaydon/MediationLayer.git
cd MediationLayer
git submodule update --init --recursive
```

### Build
```bash
mkdir build 
cd build
cmake ..
make -j4
```

## Running the Mediation Layer
The ML is comprised of a couple of executables. After building, you must ensure
that the following programs are running. It may be helpful to use a terminal
multiplexer like tmux or terminator and start each program in a separate pane.

### ROS Core
```bash
roscore
```

### Mediation Layer
```
cd MediationLayer/bin
./mediation_layer
```

### Physics Simulator
```
cd MediationLayer/bin
./physics_simulator
```

### Autonomy Protocol
```
cd MediationLayer/bin
./blue_autonomy_protocol
```

### Tests
```bash
cd build/test
./EXECUTABLE_OF_CHOICE
```
