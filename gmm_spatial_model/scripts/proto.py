#!/usr/bin/env python


# First thing is to publish a map centred at the location of the robot and visualise it

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion 
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from sklearn import mixture
from matplotlib.colors import LogNorm, Normalize
import matplotlib.pyplot as plt


def map_range(start, end, step):
    while start <= end:
        yield start
        start += step




def gmm_to_pc2(gmm, x_start, y_start, resolution, width, height):
    """
    Creates a PointCloud2 by sampling a regular grid of points from the given gmm.
    """
    pc = PointCloud2()
    pc.header.stamp = rospy.get_rostime()
    pc.header.frame_id = 'map'
 
    xy_points = []
    for x in map_range(x_start, x_start + width, resolution):
        for y in map_range(y_start, y_start + height, resolution):
            xy_points.append([x, y])
    
    logprobs, responsibilities = gmm.score_samples(xy_points)
    
    # convert back to [0,1] for my sanity
    probs = np.exp(logprobs)

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

def pack_rgb(r,g,b):
    return r << 16 | g << 8 | b

if __name__ == '__main__':
    rospy.init_node("gmm_proto")

    # grid = OccupancyGrid()
    # # resolution of created grid in metres
    # grid.info.resolution = 0.05    
    # # with and height of grid in terms of cells
    # grid.info.width = 10
    # grid.info.height = 10
    # grid.info.origin.position = Point(x=-2.95, y=-0.75, z=0.0)
    # grid.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    # # fill the grid with values we can see
    # grid.data = [100 for i in range(grid.info.width * grid.info.width)]

    # #
    # grid.header.stamp = rospy.get_rostime()
    # grid.header.frame_id = 'map'


    # grid_publisher = rospy.Publisher('/gmm_model', OccupancyGrid, queue_size=1)

    # colour map stuff
    # plt.get_cmap(name)    

    r = rospy.Rate(1)
    # while not rospy.is_shutdown():
    #     grid_publisher.publish(grid)
    #     r.sleep()

    pub_cloud = rospy.Publisher("/gmm_points", PointCloud2, queue_size=1)



    gmm = mixture.GMM(n_components=2, covariance_type='spherical')
    gmm.weights_ = np.array([1,1])
    gmm.means_ = np.array([[1,1],[2,2]])
    gmm.covars_ = np.array([[0.2,0.2],[0.1,0.1]])

 
    pcloud = gmm_to_pc2(gmm, 0, 0, 0.02, 4, 4)
  
    while not rospy.is_shutdown():
        pub_cloud.publish(pcloud)
        r.sleep()
