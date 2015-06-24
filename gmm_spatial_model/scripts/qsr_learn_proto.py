#!/usr/bin/env python

import itertools

import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy import linalg
from sklearn import mixture
from matplotlib.colors import LogNorm

from geometry_msgs.msg import Pose, Point, Quaternion 

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from math import cos, sin, radians
from gmm_spatial_model.support_functions import angle_to_point

from mongodb_store.message_store import MessageStoreProxy
from soma_msgs.msg import SOMAObject
from scipy.spatial.distance import euclidean


from sklearn.metrics import log_loss

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


def build_relational_models(bad_sample_poses, good_sample_poses, landmarks, relation_fns):

    viz = True

    results = []

    for landmark in landmarks:
        for relation_name, relation_fn in relation_fns.iteritems():

            bad_relation_metrics = to_spatial_relation(landmark.pose, bad_sample_poses, relation_fn)
            good_relation_metrics = to_spatial_relation(landmark.pose, good_sample_poses, relation_fn)
            relation_metrics = np.concatenate((bad_relation_metrics, good_relation_metrics))


            # create the mixture model
            classifier = mixture.GMM(n_components=2, covariance_type='full', init_params='wc')
            # we can initialise the means based on our knowledge of the good/bad samples
            # this is from http://scikit-learn.org/stable/auto_examples/mixture/plot_gmm_classifier.html#example-mixture-plot-gmm-classifier-py
            classifier.means_ = np.array([bad_relation_metrics.mean(axis=0), good_relation_metrics.mean(axis=0)]) 

            # fit to data, starting from these means
            classifier.fit(relation_metrics)
            # use the built classifier to predict classifiers for our training data
            predictions = classifier.predict_proba(relation_metrics)
            # and see how good it is on the data
            classifier_loss = log_loss([0] * len(bad_relation_metrics) + [1] * len(good_relation_metrics), predictions)
            print 'point %s %s: %s' % (relation_name, landmark.type, classifier_loss)
            results.append((relation_name, landmark, relation_metrics, classifier, classifier_loss))

    
    return results


def visualise_relations(target, landmarks, bad_sample_poses, good_sample_poses, classifier_results):
    # to just x,y 
    good_samples = pose_list_to_np(good_sample_poses)
    bad_samples = pose_list_to_np(bad_sample_poses)


    plt.figure(1)
    plt.subplot(2, len(classifier_results), 1) 

    centre_plot_on_pose(target.pose, 6)

    draw_pose_arrow(target.pose, arrow_length = 0.5)
    
    plt.scatter([p.pose.position.x for p in landmarks], 
                    [p.pose.position.y for p in landmarks],
                    s = 400,
                    c = 'pink', alpha = 0.8)

    plt.scatter(bad_samples[:,0], bad_samples[:,1], c='r', s = 40)
    plt.scatter(good_samples[:,0], good_samples[:,1], c='g', s = 40)
    

    plot_n = len(classifier_results)
    for classifier_result in classifier_results:
        plot_n += 1
        
        # plot classifier relation metrics, e.g. distances or anges
        ax1 = plt.subplot(2, len(classifier_results), plot_n) 
        classifier_metrics = classifier_result[2]
        classifier = classifier_result[3]
        print classifier_metrics[0]
        # if this is a 1d data set, use a histogram
        if len(classifier_metrics[0]) == 1:
            n, bins, patches = ax1.hist(classifier_metrics, 20, normed=1, facecolor='green', alpha=0.5)

            ax2 = ax1.twinx()              

            x_lim = plt.xticks()
        
            x_range = [[x] for x in np.arange(x_lim[0][0], x_lim[0][-1], 0.01)]
        
            probs, resps = classifier.score_samples(x_range)
            ax2.plot(x_range, probs)

            ax2.set_title('rel %s %s: %s' % (classifier_result[0], classifier_result[1].type, classifier_result[4]))
        else:
            ax1.axis((-1.2,1.2,-1.2,1.2))
            ax1.scatter(classifier_metrics[:,0], classifier_metrics[:,1], c='b', s = 40)

            # ax2 = ax1.twinx()              

            # display predicted scores by the model as a contour plot
            x = np.linspace(-1.2, 1.2)
            y = np.linspace(-1.2, 1.2)
            X, Y = np.meshgrid(x, y)
            XX = np.array([X.ravel(), Y.ravel()]).T
            Z = -classifier.score_samples(XX)[0]
            Z = Z.reshape(X.shape)

            CS = ax1.contour(X, Y, Z, norm=LogNorm(vmin=1.0, vmax=1000.0),
                 levels=np.logspace(0, 3, 10))
            # CB = ax1.colorbar(CS, shrink=0.8, extend='both')

            # for mean, covar in zip(classifier.means_, classifier._get_covars()):
            #     print mean
            #     v, w = linalg.eigh(covar)
            #     u = w[0] / linalg.norm(w[0])

            #     print v[0], v[1]

            #     # Plot an ellipse to show the Gaussian component
            #     angle = np.arctan(u[1] / u[0])
            #     angle = 180 * angle / np.pi  # convert to degrees
            #     ell = mpl.patches.Ellipse(mean, v[0], v[1], 180 + angle, color='black')
            #     ell.set_clip_box(ax1.bbox)
            #     ell.set_alpha(0.5)
            #     ax1.add_artist(ell)

            ax1.set_title('rel %s %s: %6.3f' % (classifier_result[0], classifier_result[1].type, classifier_result[4]))

    plt.show()


if __name__ == '__main__':

    # soma objects in base room
    table_id = '1'
    cabinet_id = '3'
    chair_id = '2'

    table = get_soma_object(table_id)   
    cabinet = get_soma_object(cabinet_id)   
    chair = get_soma_object(chair_id)   

    # some arbitrary coords
    landmark_pose = chair.pose

    # some feedback we've got from the user
    bad_sample_poses = [mkpose(-2, 0.3), mkpose(-0.2, -0.5), mkpose(-2, 0.2)]
    good_sample_poses =  [mkpose(1.8, 1), mkpose(1.9, 2.1), mkpose(1.4, 0.5)]


    results =  build_relational_models(bad_sample_poses, good_sample_poses, [table, cabinet], {'near': distance,'relative_angle': unit_circle_position})
    visualise_relations(chair, [table, cabinet], bad_sample_poses, good_sample_poses, results)



    

    


