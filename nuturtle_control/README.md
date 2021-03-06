# Nuturtle Control 
A package for reading sensor information and publish robot's pose in the odometry frame.

# Nodes
- turtle_interface - Reads sensor information and commanded body twist velocity to update robot's location and publish wheel angular velocities.
- odometry - Publishes robot's position in odometry frame
- circle - Offers services to control robot on circular trajectories.

# Conceptual Questions

## Translation Motion
[![Translation Video](https://img.youtube.com/vi/1c8Anr-mO-A/maxresdefault.jpg)](https://youtu.be/1c8Anr-mO-A)

Final location of robot:
- x: 0.17
- y: 0
- theta(degrees): 0

## Rotation Motion

[![Rotation Video](https://img.youtube.com/vi/pNwh_RnRQ_s/maxresdefault.jpg)](https://youtu.be/pNwh_RnRQ_s)

Final location of robot:
- x: -4.689e-16
- y: 2.357e-16
- theta(degrees): -4.092

## Circular Video

[![Circular Video](https://img.youtube.com/vi/rX1QRrciBjo/maxresdefault.jpg)](https://youtu.be/rX1QRrciBjo)

Final location of robot:
- x: 0.179
- y: 0.059
- theta(degrees): 36.435 

## Better driving Video

For this motion I drove the robot slowly to try to avoid any slip from switching direction. 

[![Better Driving Video](https://img.youtube.com/vi/_L6rtyroIZM/maxresdefault.jpg)](https://youtu.be/_L6rtyroIZM)

Final location of the robot
- x: -0.000564
- y: 0
- theta(degrees): 0 
