# FilteringAndLocalization
Implemented MonteCarloLocalization

Particle Filter Particles: 1000, Sensor Limit :25, 4 sides
https://www.youtube.com/watch?v=AAfQPT1FK_Y

MP3 Particle Filter Particles: 1000, Sensor Limit :20, 8 sides
https://www.youtube.com/watch?v=VQsxATw5GBI

![image](https://user-images.githubusercontent.com/64373075/176821674-77c7fe33-ce2d-4028-abb4-685a75d74f48.png)

## Lidar Processing
![Uploading image.png…]()
This module takes the raw point cloud data from the
The provided LidarProcessing module will only look at 4 directions, (front, rear, left, right) directions of the of the vehicle and will return the distance between the vehicle and wall in those directions.
