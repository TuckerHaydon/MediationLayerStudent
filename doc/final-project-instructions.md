# Final Project Instructions
## Goal
The goal is simple: the teams must command a quadcopter to find and pop two
balloons and return to the start position as quickly as possible.

## Interface
Students must complete the function StudentAutonomyProtocol::UpdateTrajectories
located in the file src/autonomy_protocol/student_autonomy_protocol.h. This
function provides access to map data, balloon data, and the current state of the
quadcopter and requires students to specify trajectories for the quadcopters. 

Note that the interface only specifies the input and output (quad state -> quad
trajectory), but not how to accomplish it. It is up to students' discretion how
they want to complete the function.

While the entire project boils down to one function in one file, understand that
the problem set before you is not trivial. You will find that the problem of
prescribing a time-optimal trajectory through a cluttered environment in the
presence of disturbances is very difficult. 

## Notes
For lab 4, students planned paths through a cluttered 2D environment. For that
lab, much of the 2D files were already written, tested, debugged, and vetted.
For this project, students are provided with little 3D support. It is completely
up to them to determine how they will tackle this problem.

Students are allowed to add any additional files, functions, or enhancements to
their GameEngine code base. However, students are not allowed to modify or
delete current functionality without the explicit permission of one of the TAs.

Students may request new functionality from the GameEngine from Tucker. Should
he have time, he will endeavor to complete as many additional features as he can.

## Code Restrictions
1) Students must not alter any code in the src/exe folder
2) Students must not alter any of the AutonomyProtocol interfaces
3) Students must not change or remove any core functionalities of the GameEngine
without explicit permission of one of the TAs.

## Running the final project
To run the final project, follow the build/run instructions in README in the
top-level GameEngine directory. Student autonomy protocols are compiled into an
executable called, "student_autonomy_protocol". Instead of running
example_autonomy_protocol, run the student_autonomy_protocol.

