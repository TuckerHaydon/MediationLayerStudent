# Lab 4
## Assignment
In the lecture we discussed four types of path planning algorithms: 
1) Depth-First Search
2) Breadth-First Search
3) Dijkstra's Algorithm
4) A\* Algorithm

Now you must implement the algorithms in C++. First create a workspace directory
and download the path-planning library.
```bash
cd ~
mkdir Workspace && cd Workspace
git clone https://github.com/tuckerhaydon/MediationLayerStudent.git
cd MediationLayerStudent
```

You should have successfully cloned the MediationLayerStudent project into your
new Workspace directory. This project will be the basis of your work for the
next two labs and will build into the true mediation layer that the Machine
Games will use. Feel free to poke around the source code. There is a lot of it!
You will not have to use most of it, but it may be helpful to some of you to see
what a C++ library looks like. 

**Question**
> What differences stand out to you between Matlab and C++? Don't list every
> detail. Try to be general.

## Getting Started
Let's make sure the project builds. The first time you build the project will be
the longest. This may take ~5 minutes.
```bash
cd ~/Workspace/MediationLayerStudent/
mkdir build
cd build
cmake .. && make -j1
```

If there were no errors, then the project build just fine. If not, contact one
of the TA's to help you fix the error. 

Now, let's run an example. The example source code is in
"MediationLayerStudent/labs/lab4/src/examples.cc". Open the file and read the
comments at the top of the file to get an understanding of what it's doing.
Afterwards, run the example and see the output.

```bash
cd ~/Workspace/MediationLayerStudent/build/
./labs/lab4/src/examples ../labs/lab4/data/test_grid_medium
```

## Path Planning
The first half of the lab is dicrete path-planning. You will implement a
Depth-First Search (DFS), Dijkstra's Algorithm (Dijkstra's), and A\* Search
(A\*). The main function that you will be using is: labs/lab4/src/main.cc. **Do
not edit main.cc**. Instead, a number of helper files have been created for you.
You will put all of your code in these files.


### Depth-First Search
Complete the algorithm in labs/lab4/src/depth_first_search2d.cc. Your complete
implementation should be in this file only.

**Experiment**
> Depth-First Search is not guaranteed to find the shortest path. Construct an
> example grid where DFS does not find the longest path. Submit is picture of the
> grid and the found path.


### Dijkstra's Algorithm
Complete the algorithm in labs/lab4/src/dijkstra2d.cc. Your complete implementation
should be in this file only.


### A\* Algorithm
Complete the algorithm in labs/lab4/src/a_star2d.cc. Your complete implementation
should be in this file only.

**Experiment**
> Recall that an A\* heuristic function must be optimistic: it must always
> _underestimate_ the true cost of proceeding from a given node to the end node.
> Design and report a cost function that _overestimates_ the true cost. Run your
> A\* solver.  How does the A\* solver with the overestimate compare to that with
> the underestimate?

**Experiment**
> Recall that the performance of A\* depends on the hueristic function. Design and
> report three distict heuristic functions. Evaluate each one using your A\*
> solver. 

**Experiment**
> Evaluate the performance of A\* using a zero heuristic function: h(current, end)
> = 0. How does it compare to Dijkstra's?

**Question**
> The A\* algorithm that you have implemented solves for the position of
> waypoints for the quadopters, but your Matlab simulation requires position,
> velocity, acceleration, and yaw time histories. How would you modify the above
> A\* implementation to generate these? Why don't we do this?


## Polynomial Smoothing
A\* quickly finds a set of position waypoints for a quadcopter to follow, but
the simulator requires position, velocity, acceleration, and yaw time
histories to be specified. One could derive these values by assuming the
quadcopter will fly in a straight line from point to point and stopping at every
waypoint before continuing onto the next. 

**Question**
> Describe a disadvantage of using the point-to-point, line-based approach.

One could also fit a set of piece-wise smooth polynomials over the set of
position waypoints. Polynomials are particularly nice because their derivatives
are nice and analytical --- it's easy to get the velocity and acceleration time
histories from the position polynomial. Moreover, if the degree of the
polynomials is high-enough, then continuity in velocity, acceleration, and
higher derivatives can be maintained throughout the trajectory. 

An optimization problem can be formulated with the piecewise polynomials,
minimizing one of the derivatives of the polynomials. For example, if one were
worried about the total force the quadcopter might have to generate throughout
the trajectory, one might want to minimize the acceleration.

The theory of the polynomial solver is described in detail in LaTex in the [P4
repository](https://github.com/tuckerhaydon/P4/doc/tex). Run the makefile to
build the corresponding PDF.  Writing the code to solve the for the polynomials
is mostly an exercise in bookkeeping so you are not required to write any of
code.

Instead, you will write and run a series of experiments using the polynomial
solver. The intention is that you will build up a more intuitive understanding
of how the polynomial solver fits into the general path-planning framework.

**Experiment**
> Design a 2D square trajectory. Run the polynomial solver over the waypoints,
> minimizing derivatives 0 through 4. Describe how the trajectory history
> changes as you increase the derivative.

**Experiment**
> Design a 2D square trajectory. Run the polynomial solver over the waypoints,
> minimizing snap, and varying the arrival time in three ways: reasonable
> arrival time, unreasonably short arrival time, unreasonably long arrival time.
> How does varying the arrival time affect the shape of the trajectory? What
> happens when the arrival time is too short?

**Experiment**
> Design a 2D circular trajectory. Run the polynomial solver over the waypoints,
> minimizing snap, and varying the number of waypoints on the edge of the
> circle. How does increasing or decreasing the number of waypoints affect the
> final trajectory?

## Putting it all together
**Experiment**
> Feed the output of your A\* path planner into the P4 solver. Print out the
> resulting polynomial coefficients and load them into Matlab. Sample the
> polynomial in Matlab and insert them into quadcopter simulator. Can the
> quadcopter follow the trajectory? Overlay plots of the intended and actual
> position, velocity, and acceleration time histories.

## Appendix 1: Reference
### Build The Lab
```bash
cd ~/Workspace/MediationLayerStudent/
mkdir build
cd build
cmake .. && make -j1
```

### Run the Lab
```bash
cd ~/Workspace/MediationLayerStudent/build
./labs/lab4/src/main ../labs/lab4/data/test_grid_medium
```

### Create your own data
Example data has been created in labs/data. Inside this folder, you will find
two files: test_grid_empty and test_grid_full. These are plain-text files that
define occupancy grids that you can run your path planner on. An occupancy grid
is structured as follows:

- The first line contains two integers separated by a space. These represent the
  number of rows and columns in the occupancy grid
- The following lines define a matrix of 0s and 1s. This is the raw data for the
  occupancy grid. A zero represents a free cell and a 1 represents an occupied
  cell. The number of rows and columns in the matrix must match the reported
  number of rows and columns in the first line of the file.
- The top-left cell is considered (0,0) with rows increasing down and columns
  increasing to the right.

For example, a 3x3 occupancy grid with an occupied center could be constructed
as follows:
```
  3 3 
  0 0 0
  0 1 0
  0 0 0
```

### Appendix 2: FAQ
No FAR so far.
