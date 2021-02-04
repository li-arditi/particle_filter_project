# particle_filter_project

__Team Members: Li Arditi and Victoria Villalba__

### Implementation Plan

* initialize particle cloud (`initialize_particle_cloud()`)
  * plan: create Particle() instance and assign pose values with a random number generator. We will be setting values for the Pose msg parameters Pose.position.x (range being the width of the map), Pose.position.y (range being height of map), Pose.orientation.z (range 0-360 in radians). Put these Particle() instances in an array
  * testing: run the vizualize_particles.launch to see if particle distribution looks appropriate

* update position of particles based on robot movement (`update_particles_with_motion_model()`)
  * plan: Loop through the particle cloud and calculate and store/update Pose values for the particle based on robot movement. 
  * testing: run visualize_particles.launch to see of the particles are actively moving according to the robot's movement.

* compute importance weights (`update_particle_weights_with_measurement_model()`)
  * plan: First we need to calculate the theoretical sensor measurements for each particle using the room map, then we can compare theoretical x,y,theta with actual robot scan data. Then use the weight equation, so take the inverse of the sums of differences in x,y,theta
  * testing: take a few particles and manually calculate the data to see if it matches up.

* normalize particles' importance weights (`normalize_particles()`)
  * plan: First we need to loop through the particle important weights to find the min and max values. Then to calculate normalized value, use the equation x_normalized = (x - x_min)/(x_max - x_min) [or is there a built-in or math function?]
  * testing: can be tested like the previous bullet, take a few and make sure the results match up. Or check to see if the highest normalization values are with particles that have similar information to the robot's scan data.

* resample (`resample_particles()`)
  * plan: use the `draw_random_sample(choices, probabilities)` function provided where choices is a list of the particles and probabilities are the normalized weights
  * testing: run visualize_particles.launch to see if particles are more concentrated, and more concentrated at areas that the robot could possibly be in.

* update estimated pose of robot (`update_estimated_robot_pose()`)
  * plan: update the estimated pose by taking the previous weights and estimates and adding the weights and probabilities that were just calculated. Taking the previous data and the current data will narrow down the location of the robot.
  * testing: see if the estimate is closer to the actual location than it was previously.

* incorporate noise into particle filter
  * Once the robot moves and we get the scan measurements, we can take multiple sensor measurements for the same position and average the measurements


### Timeline

* Mon. Feb 1: have most of if not all of initializing cloud and normalize weights done

* Wed Feb 3: start update particle position based on robot movement and update weights

* Sat Feb 5: start resample particles and update estimated robot position

* Mon. Feb 8: have code finished; give time to clean-up code/comments and finalize write-up

* work on write-up throughout/once a function is complete

### Writeup

The objective of this project is to implement the particle filter algorithm in order to localize the robot on a map. We create a large number of particles and compare the relative surroundings of the particles to the robot sensor data and then weight the particles to get a higher amount of more accurate particles when resampling. Repeating this process will gather all the particles at or near the
 robot and help us determine the location.

description:

location/description of 3 main steps:

challenges:

future work:

takeaways:

