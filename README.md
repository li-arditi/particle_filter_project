# particle_filter_project

__Team Members: Li Arditi and Victoria Villalba__

## Implementation Plan (initial)

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


## Timeline

* Mon. Feb 1: have most of if not all of initializing cloud and normalize weights done

* Wed Feb 3: start update particle position based on robot movement and update weights

* Sat Feb 5: start resample particles and update estimated robot position

* Mon. Feb 8: have code finished; give time to clean-up code/comments and finalize write-up

* work on write-up throughout/once a function is complete

## Writeup

### Objective

The objective of this project is to implement the particle filter algorithm in order to localize the robot on a map. We create a large number of particles and compare the relative surroundings of the particles to the robot sensor data and then weight the particles to get a higher amount of more accurate particles when resampling. Repeating this process will gather all the particles at or near the robot and help us determine the location.

### Description

In order to do this, we made a particle cloud initialized with random coordinates and orientations, then when the robot moves far away enough from its original location, the particles are all updated to follow the same movement and orientation change that the robot
went through. Based on the robot's scan data and the particle's distance from objects/landmarks, each particle gets a weight based on
 how similar the data is from the robot's data, which allows for a higher proportion of more accurate particles during resampling. Wi
th these weights, a new set of particle clouds are chosen to represent the location of the robot. Based on the locations of these new
 particles, the averages are taken to estimate the robot's actual location. This repeats and the particle locations become more accur
ate.

### The Code 

#### Movement

__Code Location__

Movement is implemented in lines 373-396, under the function update_particles_with_motion_model.

__Code Description__

(Li notes: movement basically involves two parts: determining the robot's actual movemnt based on the robot's odometry and updating the position of each of the particles based on the movement of the robot)

This function moves all the particles the same way the robot moved by taking the last measurement of the robot and subtracting it from the current measurement of the robot's position and orientation. Then those changes that were calculated are applied to every particle in the particle cloud by adding the changes to the old positions and orientations.

#### Computation of Importance Weights

__Code Location__

The function implementing the likelihood field algorithm is on line 341, and the function that normalizes the weights is on line 193. A helper function is used for the likelihood field algorithm, which is found on line 56.

__Code Description__

First, we implemented the likelihood field algorithm in update_particle_weights_with_measurement_model to calculate how similar the surroundings of the particle are to the surroundings of the robot. Then, a new weight is assigned to each particle based on how small the difference is. Any particle that goes outside the map gets a weight very close to zero. In the normalize_particles function, the weights of all the particles are added up and then each weight for each particle is divided by that sum.

#### Resampling

__Code Location__

The function, resample_particles, is on line 225. This function calls on a helper function on line 39.

__Code Description__

resample_particles makes a list of all the weights to use them as probabilities. The function draw_random_sample is called to porportionately create a new particle cloud according to the weights. The new particle cloud then replaces the old particle cloud.

### Challenges

[Li will write this up]
(Li notes: i found myself screaming "where the fuck are my particles!?" a lot)

### Future Work

[Li will write this up, Victoria you can add anything if you have some ideas]
(Li notes: figure out how to deal with places that "look" the same; being able to completely change the particle cloud if the current robot scan really doesn;t match the estimated robot position "view"; maybe make it so the robot can explore the environment by itself?)

### Takeaways

* expect to be doing a lot of trial and error
  * 

* 
  * 

* 
  * 

### GIF of particle filter

![gif of working particle filter]([particle_filter].gif)



