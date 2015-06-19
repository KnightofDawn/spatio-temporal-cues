#!/usr/bin/env python

import itertools

import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import matplotlib as mpl

from sklearn import mixture

from geometry_msgs.msg import Pose, Point, Quaternion 

from tf.transformations import euler_from_quaternion
import numpy as np
from math import cos, sin, radians


def quaternion_msg_to_np(q):
	"""
	Converts a Quaternion msg to a numpy array for use tf.transformations methods
	"""
	return np.array([q.x, q.y, q.z, q.w])


def draw_pose_arrow(ax, pose, arrow_length = 1):

	# ax.arrow(landmark_pose.position.x, landmark_pose.position.y, 
	# 	, 
	# 	head_width=0.05, head_length=0.1, fc='k', ec='k')

	start = (pose.position.x , pose.position.y)
	angle = euler_from_quaternion(quaternion_msg_to_np(landmark_pose.orientation))[2]	

	dx = arrow_length * cos(angle)
	dy = arrow_length * sin(angle)
	end = (pose.position.x + dx, pose.position.y + dy)

	ax.annotate("",
            xy=end, xycoords='data',
            xytext=start, textcoords='data',
            arrowprops=dict(arrowstyle="->",
                            connectionstyle="arc3"),
            )


def centre_plot_on_pose(pose, map_width = 3):
	plt.axis((pose.position.x - map_width / 2,
				pose.position.x + map_width / 2,
				pose.position.y - map_width / 2,
				pose.position.y + map_width / 2))

if __name__ == '__main__':

	# some arbitrary coords
	landmark_pose = Pose(position = Point(3.00, -0.46, 0), orientation = Quaternion(y = 4.568717849906534e-05, x = 7.467446266673505e-05, z = 0.7024875283241272, w = -0.7117186188697815))

	print landmark_pose


	# N = 50
	# x = np.random.rand(N)
	# y = np.random.rand(N)
	# colors = np.random.rand(N)
	# area = np.pi * (15 * np.random.rand(N))**2 # 0 to 15 point radiuses
	# plt.scatter(x, y, s=area, c=colors, alpha=0.5)
	
	centre_plot_on_pose(landmark_pose, 6)

	ax = plt.axes()

	draw_pose_arrow(ax, landmark_pose, arrow_length = 1)

	plt.show()

