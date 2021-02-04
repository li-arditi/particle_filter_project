#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math
from copy import deepcopy

from random import randint, random, choice, uniform

from likelihood_field import LikelihoodField



def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(choices, probabilities, n):
    """ Return a random sample of n elements from the set choices with the specified probabilities
        choices: the values to sample from represented as a list
        probabilities: the probability of selecting each element in choices represented as a list
        n: the number of samples
    """
    values = np.array(range(len(choices)))
    probs = np.array(probabilities)
    bins = np.add.accumulate(probs)
    inds = values[np.digitize(random_sample(n), bins)]
    samples = []
    for i in inds:
        samples.append(deepcopy(choices[int(i)]))
    return samples


# function provided in measurement_update_likelihood_field.py from class
def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and occupancy field
        self.map = OccupancyGrid()
        self.occupancy_field = None

        self.likelihood_field = LikelihoodField()


        # the number of particles used in the particle filter
        self.num_particles = 10

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):

        self.map = data
        

        # self.occupancy_field = OccupancyField(data)

        # rosmsg echo OccupancyGrid gices:
        # width = 384, height = 384
        # origin position x,y,z = -10.0, -10.0, 0.0; orientation x,y,z,w = 0, 0, 0, 1
        # resolution = 0.05 in m/cell

    

    def initialize_particle_cloud(self):
        # TODO

        unocc = []
        res = self.map.info.resolution # 0.05 m/cell
        width = self.map.info.width # 384 cells
        height = self.map.info.height # 384 cells

        for i in range(0, len(self.map.data)):
            if self.map.data[i] == 0:
                unocc.append(i)


        for part in range(0, self.num_particles):
            cell = choice(unocc)
            cell_x = cell % width
            cell_y = math.floor(cell/height)
            x = (cell_x + random()) * res - 10.0    # get x coord wrt center origin  
            y = (cell_y + random()) * res - 10.0    # get y coord wrt center origin
            
            p = Pose()
            p.position.x = x
            p.position.y = y

            theta = math.radians(random() * 360)    # euler orientation angle in radians 
            quat = quaternion_from_euler(0.0, 0.0, theta) # quaternion orientation

            p.orientation.x = quat[0]
            p.orientation.y = quat[1]
            p.orientation.z = quat[2]
            p.orientation.w = quat[3]

            self.particle_cloud.append(Particle(p, 1/self.num_particles))


        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        
        # TODO
        # rospy.loginfo(sum(p.w for p in self.particle_cloud))
        total_w = sum(p.w for p in self.particle_cloud)
        if round(total_w, 2) != 1:
            for particle in self.particle_cloud:
                particle.w = particle.w / total_w
        
        # rospy.loginfo(sum(p.w for p in self.particle_cloud))
        
        # if round(sum(p.w for p in self.particle_cloud), 2) != 1:
        #     weights = []
        #     for particle in self.particle_cloud:
        #         weights.append(particle.w)
        #     normalize = max(weights) - min(weights)
        #     for particle in self.particle_cloud:
        #         particle.w = (particle.w - min(weights)) / normalize
        # ^^^^ this has issues with divide by zero
        


    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            # rospy.loginfo(part.pose)
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):

        # TODO
        pass

        # self.normalize_particles()
        # probabilities = []
        # for particle in self.particle_cloud:
        #     probabilities.append(particle.w)
        # rospy.loginfo(len(probabilities))
        # rospy.loginfo(len(self.particle_cloud))
        
        # new_sample = draw_random_sample(self.particle_cloud, probabilities, self.num_particles)
        # self.particle_cloud = new_sample

        # self.publish_particle_cloud()




    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
        # TODO
        xAvg = 0
        yAvg = 0
        orientX = 0
        orientY = 0
        orientZ = 0
        orientW = 0
        for p in range(0,self.num_particles):
            xAvg += particle_cloud[p].pose.postion.x
            yAvg += particle_cloud[p].pose.postion.y
            orientX += particle_cloud[p].pose.orientation.x
            orientY += particle_cloud[p].pose.orientation.y
            orientZ += particle_cloud[p].pose.orientation.z
            orientW += particle_cloud[p].pose.orientation.w
        self.robot_estimate.pose.position.x = xAvg/self.num_particles
        self.robot_estimate.pose.position.y = yAvg/self.num_particles
        self.robot_estimate.pose.orientation.x = orientX/self.num_particles
        self.robot_estimate.pose.orientation.y = orientY/self.num_particles
        self.robot_estimate.pose.orientation.z = orientZ/self.num_particles
        self.robot_estimate.pose.orientation.w = orientW/self.num_particles

        pass


    
    def update_particle_weights_with_measurement_model(self, data):

        # TODO
        for particle in self.particle_cloud:
            rospy.loginfo(particle.pose)
            q = 1
            for a in range(0,360):
                z_kt = data.ranges[a]
                if z_kt > 3.5:
                    z_kt = 3.5

                # rospy.loginfo(z_kt)
                theta = get_yaw_from_pose(particle.pose)    # orientation of robot in rad
                # rospy.loginfo(theta)

                x_z_kt = particle.pose.position.x + z_kt * math.cos(theta + math.radians(a))
                y_z_kt = particle.pose.position.y + z_kt * math.sin(theta + math.radians(a))
                # rospy.loginfo(x_z_kt)
                # rospy.loginfo(y_z_kt)

                closest_obstacle_dist = self.likelihood_field.get_closest_obstacle_distance(x_z_kt, y_z_kt)

                prob = compute_prob_zero_centered_gaussian(closest_obstacle_dist, 0.1)
                # rospy.loginfo(prob)

                q = q * prob
                rospy.loginfo(q)

            particle.w = q
            # rospy.loginfo(particle.w)


        

    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # TODO
        dx = self.odom_pose.pose.position.x - self.odom_pose_last_motion_update.pose.position.x
        dy = self.odom_pose.pose.position.y - self.odom_pose_last_motion_update.pose.position.y
        dyaw = get_yaw_from_pose(self.odom_pose.pose) - get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
        # delta_yaw is in deg
        d = math.sqrt(dx**2 + dy**2) # distance robot travelled

        for particle in self.particle_cloud:
            part_theta = get_yaw_from_pose(particle.pose)
            particle.pose.position.x  += d * math.cos(part_theta)
            particle.pose.position.y  += d * math.sin(part_theta)
            new_quat = quaternion_from_euler(0.0, 0.0, part_theta + dyaw)
            particle.pose.orientation.x = new_quat[0]
            particle.pose.orientation.y = new_quat[1]
            particle.pose.orientation.z = new_quat[2]
            particle.pose.orientation.w = new_quat[3]





if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









