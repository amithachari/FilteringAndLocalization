# Filtering And Localization
Implemented MonteCarloLocalization

Particle Filter Particles: 1000, Sensor Limit :25, 4 sides
https://www.youtube.com/watch?v=AAfQPT1FK_Y

MP3 Particle Filter Particles: 1000, Sensor Limit :20, 8 sides
https://www.youtube.com/watch?v=VQsxATw5GBI

![image](https://user-images.githubusercontent.com/64373075/176821674-77c7fe33-ce2d-4028-abb4-685a75d74f48.png)

## Lidar Processing

This module takes the raw point cloud data from the Lidar simulator in gazebo and process the point cloud data so they be used to look at 4 or 8 directions and will return the distance between the vehicle and wall in those directions.

## Vehicle model and controlle
These two modules are implemented in vehicle.py and controller.py. These two modules drive the
vehicle constantly in the Gazebo simulator through a series of waypoints by computing the current position and orientation of the vehicle and send the information to Gazebo simulator.
