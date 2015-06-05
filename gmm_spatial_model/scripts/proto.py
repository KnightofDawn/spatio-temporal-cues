#!/usr/bin/env python


# First thing is to publish a map centred at the location of the robot and visualise it

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion 
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField

def create_cloud_xyz32(header, points):
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


    r = rospy.Rate(1)
    # while not rospy.is_shutdown():
    #     grid_publisher.publish(grid)
    #     r.sleep()

    pub_cloud = rospy.Publisher("/gmm_points", PointCloud2, queue_size=1)

    pcloud = PointCloud2()
    pcloud.header.stamp = rospy.get_rostime()
    pcloud.header.frame_id = 'map'

    # make point cloud
    cloud = []
    for x in range (0, 10):
        for y in range (0, 10):
            cloud.append([x,y,1, pack_rgb(0,0,255)])

    pcloud = create_cloud_xyz32(pcloud.header, cloud)
  
    while not rospy.is_shutdown():
        pub_cloud.publish(pcloud)
        r.sleep()
