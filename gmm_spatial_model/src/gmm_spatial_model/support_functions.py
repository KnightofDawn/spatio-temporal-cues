#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from math import atan2, pi, cos, sin, radians
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy.spatial.distance import euclidean
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from matplotlib.colors import LogNorm, Normalize
import matplotlib.pyplot as plt




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


def angle_to_point_xy(landmark, target):
    """ Returns the angle from landmark to target in radians """
    dx = target[0] - landmark[0]
    dy = target[1] - landmark[1]
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

def distance_pose_xy(landmark, target):
    return euclidean(pose_to_xy(landmark), target)


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


def unit_circle_position_pose_xy(landmark, target):
    # angle, in radians, that would make landmark point at target
    pointing_at_target = angle_to_point_xy([landmark.position.x, landmark.position.y], target)
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


def pose_list_to_np(pose_list):
    """
    Convert a list of Pose objects into an np array of x,y coords
    """
    return np.array([pose_to_xy(p) for p in pose_list])


def model_to_pc2(model, x_start, y_start, resolution, width, height):
    """
    Creates a PointCloud2 by sampling a regular grid of points from the given model.
    """
    pc = PointCloud2()
    pc.header.stamp = rospy.get_rostime()
    pc.header.frame_id = 'map'
 
    xy_points = []
    for x in map_range(x_start, x_start + width, resolution):
        for y in map_range(y_start, y_start + height, resolution):
            xy_points.append([x, y])
    
    probs = model.score_samples(xy_points)
    
    # and normalise to range to make the visualisation prettier
    normaliser = Normalize()
    normaliser.autoscale(probs)
    probs = normaliser(probs)

    colour_map = plt.get_cmap('jet')    
    colours = colour_map(probs, bytes=True)

    cloud = []
    for i in range(len(probs)):
        cloud.append([xy_points[i][0], xy_points[i][1], 2*probs[i], pack_rgb(colours[i][0], colours[i][1], colours[i][2])])
 
    return create_cloud_xyzrgb(pc.header, cloud)


def pack_rgb(r,g,b):
    return r << 16 | g << 8 | b


def create_cloud_xyzrgb(header, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message with 3 float32 fields (x, y, z).

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param points: The point cloud points.
    @type  points: iterable
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 12, PointField.UINT32, 1)]
    return pc2.create_cloud(header, fields, points)