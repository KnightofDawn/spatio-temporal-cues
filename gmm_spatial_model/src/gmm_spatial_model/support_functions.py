#!/usr/bin/env python

import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from math import atan2, pi, cos, sin, radians
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy.spatial.distance import euclidean



def predicate_to_key(predicate):
    return predicate.name + "_" + str(predicate.arguments).replace(",","_")[1:-1]



def mkpose(x, y):
    """
    Helper to create a pose (needed for later ROS integration), when we only care
     about x,y
    """
    return Pose(Point(x,y,0), Quaternion(*quaternion_from_euler(0, 0, 0)))



def angle_to_point(landmark, target):
    """ Returns the angle from landmark to target in radians """
    dx = target.x - landmark.x
    dy = target.y - landmark.y
    rads = atan2(dy,dx)
    rads %= 2*pi    
    return rads



def quarternion_to_point(landmark, target):
    """ Returns the angle from landmark to target as a quarternion """
    rads = angle_to_point(landmark, target)
    return Quaternion(*quaternion_from_euler(0, 0, rads))



def quaternion_msg_to_np(q):
    """
    Converts a Quaternion msg to a numpy array for use tf.transformations methods
    """
    return np.array([q.x, q.y, q.z, q.w])



def map_range(start, end, step):
    while start <= end:
        yield start
        start += step



def distance(landmark, target):
    """ This just returns the distance between the centre points of the two objects
    """
    return [euclidean(pose_to_xy(landmark), pose_to_xy(target))]


def unit_circle_position(landmark, target):
    """ Because we want to work in Euclidean space, we map the single 
    angle into a point in x,y which is the target's position on a unit circle
    where the 0 positing is defined by the orientation of the landmark.
    http://blog.tomgibara.com/post/11016504425/clustering-angles
    https://en.wikipedia.org/?title=Unit_circle
    """ 

    # angle, in radians, that would make landmark point at target
    pointing_at_target = angle_to_point(landmark.position, target.position)
    # current angle of landmark in radians
    landmark_angle = euler_from_quaternion(quaternion_msg_to_np(landmark.orientation))[2]   
    # how much you'd need to rotate landmark by to point at target
    relative_angle = pointing_at_target - landmark_angle
    # now, turn that relative angle into an x,y point on a unit circle
    # this 1 * cos(angle) and 1 * sin(angle)
    x = cos(relative_angle)
    y = sin(relative_angle)
    return [x, y]


def to_spatial_relation(landmark, targets, fn):
    return np.array([fn(landmark, target) for target in targets])


def pose_to_xy(pose):
    return (pose.position.x, pose.position.y)