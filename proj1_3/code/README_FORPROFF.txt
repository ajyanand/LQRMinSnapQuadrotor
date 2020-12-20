Taylor Shelby
MEAM 630 Project 1_3

Additional Functions:
dougPueck: Uses the douglas peucker algorithm to downsample the A* path with the specified margin
furthestPoint: Helper function for dougPueck, returns the furthest point in an array from a line between 2 other points
Got the corridor constraints/minsnap working! Relevant functions:
calcAb_i1: equality constraints on endpoints (position)
calcAbi2k: equality constraints on enpoints (derivatives)
addCorridorConstraints: adds corridor constraints



Collaborators: Ray Bjorkman, Josh Rembas

Other sources:
MEAM 517 HW4
https://ilya.puchka.me/douglas-peucker-algorithm/
https://math.stackexchange.com/questions/128991/how-to-calculate-the-area-of-a-3d-triangle
http://www-personal.acfr.usyd.edu.au/spns/cdm/papers/Mellinger.pdf
