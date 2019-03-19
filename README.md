# Mediation Layer
The mediation layer (ML) is a software layer that sits between a quadcopter's
controller and the path planner. Given a map of the environment and a proposed
quadcopter trajectory, the ML modifies the quadcopter trajectory to
ensure that it does not run into any obstacles. In this sense, the ML acts as a
padded room and allows any user to fly the quadcopters without worrying about
their safety. 

## Installation
### Prerequisites 
1. [Eigen](https://eigen.tuxfamily.org)
2. [ROS](http://www.ros.org)
3. [P4 Requirements](https://github.com/tuckerhaydon/P4.git)
3. sudo apt install gnuplot


### Build
```bash
mkdir build 
cd build
cmake ..
make -j4
```

### Examples
```bash
cd build/examples
./$EXECUTABLE_OF_CHOICE
```

### Tests
```bash
cd build/test
./EXECUTABLE_OF_CHOICE
```
