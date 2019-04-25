# Final Project Rules
## Goal
The goal is simple: the teams must command a quadcopter to find and pop two
balloons and return to the start position as quickly as possible.

## Scoring
Tournament scoring is determined as follows:
1) Team scores are first ordered by the number of balloons they pop. The greater
the number of balloons popped, the greater the score.
2) Team scores are secondly ordered by the time it took for both balloons to be
popped and for the quadcopter to return to the start position. The smaller the
run time, the greater the score.

Thus, the scoring reflects a law of software design: 
1) get it working
2) get it working right
3) get it working right and fast

It is paramount that teams pop both balloons, even if it takes a while for it to
occur. Once both balloons are popped, focus on refining the algorithm to pop
them ever faster.

## Number of Attempts
Each team will have a single attempt to pop both the red and blue balloons on
game day. If the instructors determine that a team's attempt failed due a
failure of course materials (ex: balloon gets sucked into quadcopter's motor),
the attempt is refunded and the team is permitted to try again.

## Path Constraints
The following subsections introduce path constraints that teams must adhere to.

### No-Fly Zone
Quadcopters must not fly into obstacles or exceed the bounds of the map. This
constraint manifests in two ways:

1) If the Mediation Layer determines that a proposed trajectory intersects an
obstacle or exceeds the bounds of the map, the proposed trajectory is rejected
and the quadcopter will continue to follow the most recent, valid trajectory.

2) If the Mediation Layer determines that a quadcopter has intersected or
touched an obstacle during the flight as a result of disturbances, the
quadcopter is frozen, forced to land, and the balloon-popping attempt is
forfeit. This constraint has a strict punishment because violations imperil the
quadcopters. Teams should take care not to fly too close to obstacles in case
the wind or another disturbance forces the quadcopter into the obstacle.

### Maximum Velocity
Quadcopters must not fly in any direction with a velocity magnitude greater than
or equal to **2.0 m/s**. If the Mediation Layer determines that a proposed
trajectory will exceed the above bound, the proposed trajectory is rejected and
the quadcopter will continue to follow the most recent, valid trajectory.

### Maximum Acceleration
Quadcopters must not fly in any direction with an acceleration magnitude greater
than **0.4 m/s^2**. If the Mediation Layer determines that a proposed trajectory
will exceed the above bound, the proposed trajectory is rejected and the
quadcopter will continue to follow the most recent, valid trajectory.

### Time
Teams will have up to seven (7) minutes to complete their balloon-popping
mission. The timer starts as soon as a team's AutonomyProtocol is engaged. At
the end of the time period, the quadcopter is frozen and forced to land. The
attempt will be scored normally.

