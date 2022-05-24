Ajay Anand and Taylor Shelby
ESE 650 Final Project

![Alt Text](https://github.com/ajyanand/LQRMinSnapQuadrotor/blob/main/OverUnderAlone.gif)

Python Quadrotor Sim used is sourced from the class materials of MEAM 620 - Advanced Robotics at the University of Pennsyvania

sandbox.py: Runs the teacher provided simulation on a specified map file using the student provided control (se3_control.py) path planning (graph_search.py) and trajectory creation (world_traj.py). Graphs and animates results, including path planning results vs. actual trajectory, system state along trajectory, and more. 

se3_control.py: Uses physical parameters provided for quad-rotor to define control response (motor speeds, thrust, moment, state) using a geometric nonlinear controller. Some outputs only used for lab hardware, some only used for simulation.

graph_search.py: Uses provided map, resolution, and margin to search for a path from the provided start to the provided goal. Uses Dijkstra if optional final argument is not set to true, uses A* when it is true. 

world_traj.py: Calls graph_search.py to get a path if one exists, then reduce the dense path to sparse path. Sparse path used to generate constraints for cvxopt quadratic program(QP), QP results used to generate polynomials representing desired quad-rotor state at particular time. 

Other files (flightsim, occupancy_map, etc) are teacher provided unless otherwise specified.

open_loop_template.py implements an open loop controller
LQR.py implements the Linear Quadratic Regulator.


Other sources:
MEAM 517 HW4 (General QP set up)
https://ilya.puchka.me/douglas-peucker-algorithm/ (Reducing dense path to sparse path)
https://math.stackexchange.com/questions/128991/how-to-calculate-the-area-of-a-3d-triangle (Implementing Douglas Peucker)
http://www-personal.acfr.usyd.edu.au/spns/cdm/papers/Mellinger.pdf (Setting up corridor constraints for QP)
