# Game Engine
## Project Structure
The Game Engine has three main components: the Mediation Layer (ML), the Physics
Simulator (PS), and the Autonomy Protocol (AP). These three programs interact
together to form the Game Engine (GE).

The PS and the AP are co-dependent --- neither can accomplish its task without
the other. The AP maps the current quadcopters' states to an intended
trajectory. The PS forward-simulates the AP's intended trajectories, injecting
distrubances, and returns the quadcopter's state at a future time point.

The ML simply displays the data that the PS and the AP are passing back and
forth.

As it stands, the Mediation Layer is a vestige of a previous iteration of the
Game Engine. Originally, the ML was supposed to forward-simulate the intended
trajectories output by the AP and inject disturbances that would force the
trajectories away from other static and dynamic objects. Since the Machine Games
rules changes, this integration is no longer necessary.

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

### Load ROS params
ROS params need only be loaded once. This must be run after roscore has been
started or re-run if any of the parameters have been changed
```bash
cd MediationLaster/run
rosparam load params.yaml /mediation_layer/
```

### ROS Visualizer
```
cd MediationLayer/run
rosrun rviz rviz config.rviz
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

## TODO
- Disentagle the visualization parts of the ML from the logical parts of the ML.
  Break them into two separate binaries 
- Complete the ML's intended purpose: forward-integrating the intended
  trajectory to create a mediated trajectory
- Visualize the intended trajectories of the quadcopters
- Create BalloonWatchdog: determines if a quadcopter has popped a balloon by
  comparing the distance between the quadcopter's state and the ballon's
  location
- Create StateWatchdog: determine if a quadcopter has flown into the no-fly-zone
  around obstacles. If so, end the game
- Create StateConstraints structure that lists the constraints of a quadcopter's
  actions. Pass an instance of this to the AutonomyProtocol.
- Create TrajectoryVetter. TrajectoryVetter determines if an intended trajectory
  are within the constraints set by StateConstraints. If not, rejects the
  trajectory.
- Change CMake libraries from interfaces into static libraries
