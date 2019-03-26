# Lab 4
## Preamble
Questions and experiments to be completed and reported in the submitted lab
report are designated as follows:

**Question: Q0**
> This box contains a question that you must answer.

Point values for each part of the lab are specified. 25 points are reserved for
the lab report. Remember to write for busy readers \-\-\- _concisely_ answer all
the questions.

To submit your lab, zip the entire MediationLayerStudent directory and submit it
to canvas.

## Assignment
In the lecture we discussed four types of path planning algorithms: 
1) Depth-First Search
2) Breadth-First Search
3) Dijkstra's Algorithm
4) A\* Algorithm

Now you must implement the algorithms in C++. First create a workspace directory
and download the MediationLayerStudent repository. Open the terminal by clicking
'Activities' in the upper-left corner and searching for 'Terminal'. Then enter
the following commands.
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

**Question: Q1**
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

If there were no errors, then the project built just fine. If not, contact one
of the TAs to help you fix the error. 

Now, let's run an example. The example source code is in
"MediationLayerStudent/labs/lab4/src/examples.cc". Open the file and read the
comments at the top of the file to get an understanding of what it's doing.
Afterwards, run the example and see the output. When running the program, you
must also specify the path to an occupancy grid. Some example occupancy grid
files are located in lab4/data/. You may examine those. The structure of these
files is explained in the appendix at the end of this document.

```bash
cd ~/Workspace/MediationLayerStudent/build/
./labs/lab4/src/examples ../labs/lab4/data/test_grid_medium
```

## Path Planning
The first half of the lab is discrete path-planning. You will implement a
Depth-First Search (DFS), Dijkstra's Algorithm (Dijkstra's), and A\* Search
(A\*). The main function that you will be using is: path_planning.cc. **Do not
edit path_planning.cc**. Instead, a number of helper files have been created for
you.  You will put all of your code in these files.


### Depth-First Search (15 Points)
Complete the algorithm in lab4/src/depth_first_search2d.cc. Your complete
implementation should be fully contained in this file -- do not put your code
anywhere else. Run the executable by following the instructions for the
path_planning executable in the appendix.

**Question: Q2**
> Depth-First Search is not guaranteed to find the shortest path. Construct an
> example grid where DFS does not find the longest path. Report the example
> grid, the path that DFS found, the length of the found path, and the optimal
> length of the path. Hint: use a small grid with a trivial solution.


### Dijkstra's Algorithm (15 Points)
Complete the algorithm in lab4/src/dijkstra2d.cc. Your complete implementation should be
fully contained in this file -- do not put your code anywhere else. Run the
executable by following the instructions for the path_planning executable in the
appendix.

**Question: Q3**
> Run Dijkstra's and DFS over the same grid. Report which grid you used. Which
> finds the shortest path?  Which explores the fewest nodes? Which runs the
> fastest?


### A\* Algorithm (15 Points)
Complete the algorithm in lab4/src/a_star2d.cc. Your complete implementation
should be fully contained in this file --- do not put your code anywhere else.
Run the executable by following the instructions for the path_planning
executable in the appendix.

**Question: Q4**
> Recall that an A\* heuristic function must be optimistic: it must always
> _underestimate_ the true cost of proceeding from a given node to the end node.
> Design a cost function that _overestimates_ the true cost. Run your
> A\* solver.  How does the A\* solver with the overestimate compare to that with
> the underestimate?

**Question: Q5**
> Recall that the performance of A\* depends on the heuristic function. Design and
> report two distinct heuristic functions. Evaluate each one using your A\*
> solver. 

**Question: Q6**
> Evaluate the performance of A\* using a zero heuristic function: h(current, end)
> = 0. How does it compare to Dijkstra's?

**Question: Q7**
> The A\* algorithm that you have implemented solves for the position of
> waypoints for the quadopters, but your Matlab simulation requires position,
> velocity, acceleration, and yaw time histories. How would you modify the 2D
> position-only A\* implementation to generate position, velocity, and
> acceleration time histories? Why isn't this done in practice?


## Polynomial Smoothing (15 Points)
A\* quickly finds a set of position waypoints for a quadcopter to follow, but
the simulator requires position, velocity, acceleration, and yaw time
histories to be specified. One could derive these values by assuming the
quadcopter will fly in a straight line from point to point and stopping at every
waypoint before continuing onto the next. 

**Question: Q8**
> Describe a disadvantage of using the point-to-point, line-based approach.

One could also fit a set of piece-wise smooth polynomials over the set of
position waypoints. Polynomials are particularly nice because their derivatives
are nice and analytical --- it's easy to get the velocity and acceleration time
histories from the position polynomial. Moreover, if the degree of the
polynomials is high-enough, then continuity in velocity, acceleration, and
higher derivatives can be maintained throughout the trajectory. 

An optimization problem may be formulated with the piecewise polynomials that
minimizes one of the derivatives of the polynomials. For example, if one were
worried about the total force the quadcopter might have to generate throughout
the trajectory, one might want to minimize the acceleration.

The theory of the polynomial solver is described in detail in LaTex in the [P4
repository](https://github.com/TuckerHaydon/P4/tree/master/doc/tex). Run the
makefile to build the corresponding PDF.  Writing the code to solve the for the
polynomials is mostly an exercise in bookkeeping, so you are not required to
write any of it.

Instead, you will write and run a series of experiments using the polynomial
solver. The intention is that you will build up a more intuitive understanding
of how the polynomial solver fits into the general path-planning framework.

For the following questions, complete the helper functions in
lab4/src/polynomial_planning.cc. Run the executable by following the
instructions for polynomial_planning in the appendix.

**Question: Q9**
> Create a 2D trajectory by placing four waypoints in a square layout. Run the
> polynomial solver over the waypoints, minimizing derivatives 0 through 4.
> Describe how the trajectory changes as you increase the derivative.

**Question: Q10**
> Create a 2D trajectory by placing four waypoints in a square layout. Run the
> polynomial solver over the waypoints, minimizing acceleration, and varying the
> arrival time in three ways: reasonable arrival time, unreasonably short
> arrival time, unreasonably long arrival time.  How does varying the arrival
> time affect the shape of the trajectory? How does it affect the velocity and
> acceleration profiles?

**Question: Q11**
> Create a 2D trajectory by placing N waypoints uniformly around a circle. Run
> the polynomial solver over the waypoints, minimizing acceleration, and varying
> the number of waypoints on the edge of the circle. How does increasing or
> decreasing the number of waypoints affect the final trajectory?

## Putting it all together (15 points)
For the following question, complete the helper functions in
lab4/src/full_stack_planning.cc. Run the executable by following the
instructions for full_stack_planning in the appendix.

**Question: Q12**
> Run your A\* solver over lab4/data/full_stack_grid from (0,0) to (9,9). Feed
> the waypoints generated by A\* directly into the polynomial solver. Give the
> solver 1 second to go from waypoint to waypoint. Minimize acceleration. Sample
> the position, velocity, and acceleration of the trajectory at 200Hz and figure
> out a way to load the data into Matlab. Maybe you can save it to a CSV file?
> Load the position, velocity, acceleration, and time histories into your Matlab
> simulator and execute it (assume z=0 for all time). Does your quadcopter
> follow the trajectory? Include plots and commentary in your report.

Note that you may have to rework portions of the topSimulateControl.m file to work with
the incoming data.

## Appendix 1: Reference
### Build The Lab
```bash
cd ~/Workspace/MediationLayerStudent/
mkdir build
cd build
cmake .. && make -j1
```

### Run an executable
Run the path_planning executable. You must specify an occupancy grid as well as
the start and end positions.
```bash
cd ~/Workspace/MediationLayerStudent/build
./labs/lab4/src/path_planning ../labs/lab4/data/grid row1 col1 row2 col2
```

Run the polynomial_planning executable.
```bash
cd ~/Workspace/MediationLayerStudent/build
./labs/lab4/src/polynomial_planning
```

Run the full_stack_planning executable. You must specify an occupancy grid as
well as the start and end positions.
```bash
cd ~/Workspace/MediationLayerStudent/build
./labs/lab4/src/path_planning ../labs/lab4/data/grid row1 col1 row2 col2
```

### Create your own data
Example data has been created in labs/data. These are plain-text files that
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
**The TA said a new update was available. How do I update my repository?**
```bash
cd ~/Workspace/MediationLayer/
git pull origin master
```

If a window pops open asking for a merge message, just accept the default merge
message by quitting the screen. For the default editor, just press control-x.

**I got a merge conflict error when I tried to update. What do I do?**
Merge conflicts occur when the update you are pulling modifies the same code
that you have been working on. Git doesn't know which modification is the 'truth', so it asks you to clarify. You can do some research and figure out how to fix the merge conflict yourself, or email/visit one of the TA's for help fixing it.


