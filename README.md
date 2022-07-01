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

 # Monte Carlo Localization
This module is located in the particle_filter.py file. This module contains the implementation of the Monte Carlo Localization that is based on the sensor reading from the robot, sensor reading from each particle from the map, and the control signal from vehicle model and controller. In addition, the MCL will hold a list of particles. The output from this module is the
estimated position of the vehicle in the ECEB environment. The MCL resides in the runFilter function.

### Working
![image](https://user-images.githubusercontent.com/64373075/176822525-1eb97460-9559-49f8-8605-8895b8028836.png)

MCL can be used to approximate the posterior probability distribution of current location based on motion models and measurement
updates. The algorithm should holds a list of uniformly random generated particles in initialization. Then
as the vehicle moving, the algorithm will shifts the particles to predict its new state after the movement.
With the sensor reading from the vehicle, the particles are weighted and resampled based on how well the
actual sensed data correlated with the predicted state. The algorithm will run iteratively and ultimately,
most of the particles should converge toward the actual state of the vehicle.
