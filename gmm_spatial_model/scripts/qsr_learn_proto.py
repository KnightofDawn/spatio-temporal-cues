#!/usr/bin/env python

import itertools

import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy import linalg
from sklearn import mixture

from geometry_msgs.msg import Pose, Point, Quaternion 

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from math import cos, sin, radians
from gmm_spatial_model.spatial_model_service import NearModel

from mongodb_store.message_store import MessageStoreProxy
from soma_msgs.msg import SOMAObject
from scipy.spatial.distance import euclidean




def quaternion_msg_to_np(q):
    """
    Converts a Quaternion msg to a numpy array for use tf.transformations methods
    """
    return np.array([q.x, q.y, q.z, q.w])


def draw_pose_arrow(pose, arrow_length = 1, annotation = ''):

    # ax.arrow(landmark_pose.position.x, landmark_pose.position.y, 
    #   , 
    #   head_width=0.05, head_length=0.1, fc='k', ec='k')

    start = (pose.position.x , pose.position.y)
    angle = euler_from_quaternion(quaternion_msg_to_np(landmark_pose.orientation))[2]   

    dx = arrow_length * cos(angle)
    dy = arrow_length * sin(angle)
    end = (pose.position.x + dx, pose.position.y + dy)

    plt.annotate(annotation,
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

soma_proxy = MessageStoreProxy(collection="soma")

def get_soma_object(id, soma_map='sapienza', soma_config='sapienza'):
    query = {'id': id, 'map': soma_map, 'config': soma_config}
    return soma_proxy.query(SOMAObject._type, message_query=query, single=True)[0]


def mkpose(x, y):
    """
    Helper to create a pose (needed for later ROS integration), when we only care
     about x,y
    """
    return Pose(Point(x,y,0), Quaternion(*quaternion_from_euler(0, 0, 0)))

def pose_to_xy(pose):
    return (pose.position.x, pose.position.y)

def pose_list_to_np(pose_list):
    """
    Convert a list of Pose objects into an np array of x,y coords
    """
    return np.array([pose_to_xy(p) for p in pose_list])


def distance(landmark, target):
    return euclidean(pose_to_xy(landmark), pose_to_xy(target))

def to_spatial_relation(landmark, targets, fn):
    return np.array([[fn(landmark, target)] for target in targets])

if __name__ == '__main__':

    # soma objects in base room
    table_id = '1'
    cabinet_id = '3'
    chair_id = '4'

    table = get_soma_object(table_id)   
    cabinet = get_soma_object(cabinet_id)   
    chair = get_soma_object(chair_id)   

    # some arbitrary coords
    landmark_pose = chair.pose

    # some feedback we've got from the user
    good_sample_poses =  [mkpose(1.8, 1), mkpose(1.9, 2.1), mkpose(1.4, 0.5)]
    bad_sample_poses = [mkpose(-2, 0.3), mkpose(-0.2, -0.5), mkpose(-2, 0.2)]
    all_sample_poses = good_sample_poses + bad_sample_poses


    good_samples = pose_list_to_np(good_sample_poses)
    bad_samples = pose_list_to_np(bad_sample_poses)
    

    # N = 50
    # x = np.random.rand(N)
    # y = np.random.rand(N)
    # colors = np.random.rand(N)
    # area = np.pi * (15 * np.random.rand(N))**2 # 0 to 15 point radiuses
    
    plt.figure(1)
    plt.subplot(211)
# 
    centre_plot_on_pose(landmark_pose, 6)


    draw_pose_arrow(landmark_pose, arrow_length = 0.5)
    obstacles = [cabinet, table]
    plt.scatter([p.pose.position.x for p in obstacles], 
                    [p.pose.position.y for p in obstacles],
                    s = 400,
                    c = 'pink', alpha = 0.8)

    plt.scatter(good_samples[:,0], good_samples[:,1], c='g', s = 40)
    plt.scatter(bad_samples[:,0], bad_samples[:,1], c='r', s = 40)
    

    distances = to_spatial_relation(cabinet.pose, all_sample_poses, distance)
    print distances

    ax1 = plt.subplot(212)
    n, bins, patches = ax1.hist(distances, 20, normed=1, facecolor='green', alpha=0.5)
    

    ax2 = ax1.twinx()

    clf = mixture.GMM(n_components=2, covariance_type='full')
    

    clf.fit(distances)

    # for mean, covar in zip(clf.means_, clf._get_covars()):
    x_lim = plt.xticks()
    
    x_range = [[x] for x in np.arange(x_lim[0][0], x_lim[0][-1], 0.01)]
    
    probs, resps = clf.score_samples(x_range)
    ax2.plot(x_range, probs)


    # clf = mixture.GMM(n_components=2, covariance_type='full')
    # # clf = mixture.DPGMM(n_components=2, covariance_type='full')

    # clf.fit(np.concatenate((good_samples,bad_samples)))

    # for mean, covar in zip(clf.means_, clf._get_covars()):
    #     v, w = linalg.eigh(covar)
    #     u = w[0] / linalg.norm(w[0])

    #     # as the DP will not use every component it has access to
    #     # unless it needs it, we shouldn't plot the redundant
    #     # components.
    #     # if not np.any(Y_ == i):
    #         # continue

    #     # Plot an ellipse to show the Gaussian component
    #     angle = np.arctan(u[1] / u[0])
    #     angle = 180 * angle / np.pi  # convert to degrees
    #     ell = mpl.patches.Ellipse(mean, v[0], v[1], 180 + angle, color='yellow')
    #     ell.set_clip_box(ax.bbox)
    #     ell.set_alpha(0.5)
    #     ax.add_artist(ell)

    # create a near model for testing
    # a good way of sanity checking things
    # model = NearModel(landmark_pose, 0.5) 
    # samples = model.sample(100)
    # plt.scatter(samples[:,0], samples[:,1])

    plt.show()

