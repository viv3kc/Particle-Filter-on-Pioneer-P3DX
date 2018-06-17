import math
import time
import random
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

def timed(fn):
    """ Decorator to time functions. For debugging time critical code """
    def timed(*args,  **kwargs):
        t = time.time()
        print "[", fn, __name__, "]Start: ",  t
        ret =  fn(*args, **kwargs)
        print "[", fn, __name__, "]End:", time.time(),  " = = = ",  time.time() - t
        return ret
    return timed

def rotateQuaternion(q_orig, yaw):
    """
    Converts a basic rotation about the z-axis (in radians) into the
    Quaternion notation required by ROS transform and pose messages.
    
    :Args:
       | q_orig (geometry_msgs.msg.Quaternion): to be rotated
       | yaw (double): rotate by this amount in radians
    :Return:
       | (geometry_msgs.msg.Quaternion) q_orig rotated yaw about the z axis
     """
    # Create a temporary Quaternion to represent the change in heading
    q_headingChange = Quaternion()

    p = 0
    y = yaw / 2.0
    r = 0
 
    sinp = math.sin(p)
    siny = math.sin(y)
    sinr = math.sin(r)
    cosp = math.cos(p)
    cosy = math.cos(y)
    cosr = math.cos(r)
 
    q_headingChange.x = sinr * cosp * cosy - cosr * sinp * siny
    q_headingChange.y = cosr * sinp * cosy + sinr * cosp * siny
    q_headingChange.z = cosr * cosp * siny - sinr * sinp * cosy
    q_headingChange.w = cosr * cosp * cosy + sinr * sinp * siny

    # Multiply new (heading-only) quaternion by the existing (pitch and bank) 
    # quaternion. Order is important! Original orientation is the second 
    # argument rotation which will be applied to the quaternion is the first 
    # argument. 
    return multiply_quaternions(q_headingChange, q_orig)


def multiply_quaternions( qa, qb ):
    """
    Multiplies two quaternions to give the rotation of qb by qa.
    
    :Args:
       | qa (geometry_msgs.msg.Quaternion): rotation amount to apply to qb
       | qb (geometry_msgs.msg.Quaternion): to rotate by qa
    :Return:
       | (geometry_msgs.msg.Quaternion): qb rotated by qa.
    """
    combined = Quaternion()
    
    combined.w = (qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z)
    combined.x = (qa.x * qb.w + qa.w * qb.x + qa.y * qb.z - qa.z * qb.y)
    combined.y = (qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x)
    combined.z = (qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w)
    return combined


def getHeading(q):
    """
    Get the robot heading in radians from a Quaternion representation.
    
    :Args:
        | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw


def find_map_coords(occupancy_map):
    """
    Finds the coordinates of all points that are inside the actual map area

    :param occupancy_map: (nav_msgs.OccupancyGrid) world map information
    :return: coord_list: (Array of float tuples) set of pairs of coordinates
    """
    map_data = occupancy_map.data
    size = occupancy_map.info.width
    origin = occupancy_map.info.origin
    resolution = occupancy_map.info.resolution

    # Find indices of all points inside the actual map
    found_indices = [i for i, map_data in enumerate(map_data) if 0 <= map_data <= 0.5]
    # Create an array containing them
    found_indices = np.array(found_indices)
    # Reduce the values to coordinates inside a size x size matrix
    actual_indices = found_indices % size
    # Compute two arrays containing X-axis and Y-axis indices for each point inside the map
    x_coords = ((actual_indices + 1) * resolution) + origin.position.x
    y_coords = ((found_indices - actual_indices) + size) / size * resolution + origin.position.y

    # Create a list of tuples (x, y)
    coord_list = zip(x_coords, y_coords)
    return coord_list


def equal_poses(first_pose, second_pose):
    """
    Checks if two poses are equal
    :param first_pose: (geometry_msgs.msg.Pose) First pose
    :param second_pose: (geometry_msgs.msg.Pose) Second pose
    :return: (Boolean) True if poses are equal, False otherwise
    """
    return first_pose.position.x == second_pose.position.x and \
           first_pose.position.y == second_pose.position.y and \
           first_pose.position.z == second_pose.position.z and \
           first_pose.orientation.x == second_pose.orientation.x and \
           first_pose.orientation.y == second_pose.orientation.y and \
           first_pose.orientation.z == second_pose.orientation.z and \
           first_pose.orientation.w == second_pose.orientation.w


def generate_sample_pose(source_pose, noise):
    """
    Generates a new pose around a source one, adding noise to it
    :param source_pose: (geometry_msgs.msg.Pose) the pose around the new one will be created
    :param noise: (float) noise added to the pose
    :return: (geometry_msgs.msg.Pose) The new pose
    """
    new_pose = Pose()

    # Generate random x and y based on a gaussian distribution, adding noise afterwards
    new_pose.position.x = random.gauss(source_pose.position.x, noise)
    new_pose.position.y = random.gauss(source_pose.position.y, noise)
    new_pose.position.z = 0
    heading = getHeading(source_pose.orientation)  # Get the heading of the source pose
    new_pose.orientation.w = 1
    # Set the orientation of the new pose by rotating an Pose with the heading of 0 by a random value
    # generated according to a Von Misses distribution (in radians)
    new_pose.orientation = rotateQuaternion(
        new_pose.orientation,
        random.vonmisesvariate(heading, 35)
    )
    return new_pose
