# masseuse

Pose Graph Relaxation Library

Dependencies:
- Ceres-Solver
- Eigen
- Sophus

To build the example, which has a GUI for pose graph visualization, the following is also required:
- Pangolin
- SceneGraph
- GFlags
- CVars

Input file:

Currently Masseuse takes in a binary input file with poses in the following format
x y z p q r

The file should contain relative pose constraints and loop closure constraints
It should start with the origin, in world coordinates.
Then it should have the number of relative poses.
Then the number of loop closure constraints
then, in order, all of the relative pose constraints and all of the loop closure constraints.
See "LoadPoseGraphAndLCC" in the source code for more details on the expected format.

Useful Commands:
- CMD + S -> Save pose graph to binary file
