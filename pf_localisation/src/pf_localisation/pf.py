from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
import math
import cluster
import numpy.ma as ma
import copy
import util
from util import rotateQuaternion
import random
import datetime
import numpy as np

from time import time


class PFLocaliser(PFLocaliserBase):

    def __init__(self):
        # Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.03  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0.03  # Odometry model x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0.02  # Odometry model y axis (side-to-side) noise
        # Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20  # Number of readings to predict
        self.PARTICLES_COUNT = 200
        # Poses generated in the last iteration
        self.last_pose_array = []
        # Time of the last randomly generated batch of poses around the map (in seconds)
        self.last_generate_time = time()
        # Points (x, y) inside the actual map
        self.points_inside_map = []
        # Keep a record if the robot has moved
        self.has_moved = False

        self.printed = False

        self.last_estimated_pose = None

        self.weight_total = 0

        self.last_laser_scan = None

        # Variables needed for logging certainty
        self.data_file = None
        self.timestamp_init = None

        self.certainty_queue = []

    def initialise_particle_cloud(self, initialpose):
        self.points_inside_map = util.find_map_coords(self.occupancy_map)

        poses = PoseArray()  # create the initial particle cloud as a PoseArray

        i = 0
        while i < self.PARTICLES_COUNT:
            poses.poses.append(self.generate_pose_inside_map())
            i += 1

        print 'Particle cloud was successfully initialised...'

        self.data_file = open('/tmp/Certainty table ' + datetime.datetime.now().strftime("%c") + '.csv', 'w')
        print("CSV file: " + str(self.data_file))

        self.timestamp_init = time()

        return poses

    # Called whenever a new LaserScan message is received. This method does the actual particle filtering
    def update_particle_cloud(self, scan):
        # Noise added to each new particle
        noise = 0.20

        self.last_laser_scan = scan

        # Reset the check of the robot moving for each laser scan
        self.has_moved = False

        # Sum of all particle weights
        weight_total = 0
        # Cumulative weights for each particle in the particle cloud
        cumulative_weights = []
        # Variable for checking whether generating a small batch of
        # random particles uniformly around the map
        gen_rand_particles = False

        # Check if odometry updates have been applied to the last particle cloud
        # if yes, then update based on the laser scans
        # if no, then leave it as it is
        if len(self.last_pose_array) != 0:
            equal = True  # Variable used for checking if the current particle cloud and the previous one are the same
            for i in range(0, len(self.last_pose_array) - 1):
                equal = util.equal_poses(self.last_pose_array[i], self.particlecloud.poses[i])
                if not equal:
                    break
            if equal:
                return  # If they are equal, then exit the function without doing any updates based on laser scans
        else:
            # If we are in the first iteration and there is no previous pose array,
            # then store a copy of it and exit the function
            self.last_pose_array = copy.deepcopy(self.particlecloud.poses)
            return
        self.has_moved = True
        # Replace invalid ranges (ones which exceed maximum range of the laser scan)
        # with the maximum range value of the laser scanner
        scan.ranges = ma.masked_invalid(scan.ranges).filled(scan.range_max)

        # Get the likelihood weighting and the cumulative weights of each Pose
        for pose in self.particlecloud.poses:
            pose_weight = self.sensor_model.get_weight(scan, pose)
            weight_total += pose_weight
            cumulative_weights.append(weight_total)

        self.weight_total = weight_total

        # Check if there have passed more than 3 seconds since the last small batch
        # of particles have been randomly generated around the map
        if time() - self.last_generate_time > 3:
            gen_rand_particles = True
            self.last_generate_time = time()

        # Check if the small batch of random particles should be generated around the map
        max_particles = self.PARTICLES_COUNT \
            if not gen_rand_particles \
            else 0.80 * self.PARTICLES_COUNT
        # If the time interval is bigger than 3 seconds,
        # then sample only 80% of the particle cloud based on laser scans and
        # generate the rest randomly across the map

        particles_count = 0
        new_particle_cloud = PoseArray()
        while particles_count <= max_particles:
            rand_num = random.uniform(0, 1) * weight_total  # generate a random number between 0 and sum of the weights

            for i in range(0, len(cumulative_weights)):
                # Find the first pose whose cumulative weight is bigger than the random value generated
                if cumulative_weights[i] > rand_num:
                    source_pose = self.particlecloud.poses[i]
                    # Generate a new pose based of the found one
                    new_particle_cloud.poses.append(util.generate_sample_pose(source_pose, noise))
                    break

            particles_count += 1

        # If the time interval since the last randomly generated small batch of particles
        if gen_rand_particles:
            i = max_particles
            while i <= self.PARTICLES_COUNT:
                # Generate a small number of particles randomly across the map
                new_particle_cloud.poses.append(self.generate_pose_inside_map())
                i += 1

        # Store a copy of the new particle cloud
        self.last_pose_array = copy.deepcopy(new_particle_cloud.poses)
        self.particlecloud = new_particle_cloud

    def estimate_pose(self):
        if not self.has_moved and self.last_estimated_pose is not None:
            return self.last_estimated_pose

        # Get the biggest cluster of poses
        pose_cluster = cluster.get_biggest_cluster(self.particlecloud.poses, 4)

        estim_pose = Pose()  # The estimated pose of the robot

        # Calculate the sums of each value x, y, z, w for both position and orientation
        for pose in pose_cluster:
            estim_pose.position.x += pose.position.x
            estim_pose.position.y += pose.position.y
            estim_pose.position.z += pose.position.z
            estim_pose.orientation.x += pose.orientation.x
            estim_pose.orientation.y += pose.orientation.y
            estim_pose.orientation.z += pose.orientation.z
            estim_pose.orientation.w += pose.orientation.w

        # Calculate the average for each both position and orientation
        size = len(pose_cluster)
        estim_pose.position.x = estim_pose.position.x / size
        estim_pose.position.y = estim_pose.position.y / size
        estim_pose.position.z = estim_pose.position.z / size
        estim_pose.orientation.x = estim_pose.orientation.x / size
        estim_pose.orientation.y = estim_pose.orientation.y / size
        estim_pose.orientation.z = estim_pose.orientation.z / size
        estim_pose.orientation.w = estim_pose.orientation.w / size

        # Log the certainty
        # Get time in seconds since start
        time_since_init = (time() - self.timestamp_init)

        certainty = 0.0
        variance = 0.0
        sum_of_diff = 0.0
        for i in range(0, len(self.particlecloud.poses) - 1):
            a_squared = (estim_pose.position.x - self.particlecloud.poses[i].position.x) * (
            estim_pose.position.x - self.particlecloud.poses[i].position.x)
            b_squared = (estim_pose.position.y - self.particlecloud.poses[i].position.y) * (
            estim_pose.position.y - self.particlecloud.poses[i].position.y)
            sum_of_diff += (math.sqrt(a_squared + b_squared));
        variance = sum_of_diff / len(self.particlecloud.poses)
        certainty = (1 / (1 + variance))

        self.certainty_queue.append(certainty)

        if len(self.certainty_queue) >= 10:
            self.certainty_queue.remove(self.certainty_queue[0])

        certainty = np.mean(self.certainty_queue)

        if certainty < 0.5:
            if self.PARTICLES_COUNT == 200:
                self.PARTICLES_COUNT = 1000
                for i in range(200, self.PARTICLES_COUNT):
                    self.particlecloud.poses.append(self.generate_pose_inside_map())
        else:
            self.PARTICLES_COUNT = 200

        # Store the time and certainty in the data file, format is "time, certainty" (CSV format to evaluate it with Excel)
        print str(time_since_init) + ", " + str(certainty) + ' ' + str(self.PARTICLES_COUNT)
        self.data_file.write(str(time_since_init) + ", " + str(certainty) + "\n")
        self.data_file.flush()
        self.last_estimated_pose = copy.deepcopy(estim_pose)
        return estim_pose

    def generate_pose_inside_map(self):
        """
        Generates a pose randmly inside the map
        :return: (geometry_msgs.msg.Pose) A pose randomly placed inside the map
        """
        pose = Pose()
        # Gets a random pair of coordinates (x, y) inside the map and assigns them to the new pose
        idx = random.randint(0, len(self.points_inside_map) - 1)
        pose.position.x = self.points_inside_map[idx][0]
        pose.position.y = self.points_inside_map[idx][1]
        pose.position.z = 0
        pose.orientation.w = 1
        # Assigns a random orientation to the new pose by rotating a Quaternion
        # by a random angle between 0 and 360 degrees
        pose.orientation = rotateQuaternion(
            pose.orientation,
            math.radians(random.uniform(0, 360))
        )

        return pose

