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

***
Q: What differences stand out to you between Matlab and C++? Don't list every
   detail. Try to be general.
***

Let's make sure the project builds.
```bash
cd ~/Workspace/MediationLayerStudent/
mkdir build
cd build
cmake .. && make -j4
```

If there were no errors, then the project build just fine. If not, contact one
of the TA's to help you fix the error. 

Now, let's run an example. The example source code is in
"MediationLayerStudent/labs/examples.cc". Open the file and read the
comments at the top of the file to get an understanding of what it's doing.
Afterwards, run the example and see the output.

```bash
cd ~/Workspace/MediationLayerStudent/build/
./labs/examples
```

