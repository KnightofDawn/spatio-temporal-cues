#!/usr/bin/env python


# First thing is to publish a map centred at the location of the robot and visualise it

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion 
from std_msgs.msg import Header


if __name__ == '__main__':
    rospy.init_node("gmm_proto")

    grid = OccupancyGrid()
    # resolution of created grid in metres
    grid.info.resolution = 0.05    
    # with and height of grid in terms of cells
    grid.info.width = 10
    grid.info.height = 10
    grid.info.origin.position = Point(x=-2.95, y=-0.75, z=0.0)
    grid.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    # fill the grid with values we can see
    grid.data = [100 for i in range(grid.info.width * grid.info.width)]

    #
    grid.header.stamp = rospy.get_rostime()
    grid.header.frame_id = 'map'


    grid_publisher = rospy.Publisher('/gmm_model', OccupancyGrid, queue_size=1)


    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        grid_publisher.publish(grid)
        r.sleep()
        