#!/usr/bin/env python

from scipy.stats import multivariate_normal
import numpy as np
from sklearn import mixture
import gmm_spatial_model.srv as sm_srv
from scipy.spatial.distance import euclidean
import support_functions

MULTIPLYING_FACTOR = 0.01

class AggregateModel(object):
    def __init__(self, model):
        super(AggregateModel, self).__init__()
        if type(model) is list:
            self.models_list = [PreferenceModel()] + model
        else:
            self.models_list = [PreferenceModel(), model]

    def score_samples(self, points):
        weighted_scores = self.models_list[0].score_samples(points)

        for model in self.models_list[1:]:
            score = model.score_samples(points)
            weighted_scores = np.multiply(weighted_scores, score)

        return weighted_scores

    def add_preference_point(self, point_x, point_y, good_bool):
        self.models_list[0].add_preference_point(point_x, point_y, good_bool)

    def __str__(self):
        string = ""
        for model in self.models_list:  
            string = string + str(model) + "\n"
        return string



class TransferModel(object):
    def __init__(self, landmark, gmm, classifier_loss, relation_name):
        super(TransferModel, self).__init__()
        self.gmm                = gmm
        self.landmark           = landmark
        self.classifier_loss    = classifier_loss
        self.positive           = multivariate_normal(gmm.means_[1], gmm.covars_[1])#, allow_singular=True)
        self.negative           = multivariate_normal(gmm.means_[0], gmm.covars_[0])#, allow_singular=True)

        if relation_name == 'near':
            self.scoring_function   = support_functions.distance_pose_xy
        elif relation_name == 'relative_angle':
            self.scoring_function   = support_functions.unit_circle_position_pose_xy
        else:
            raise rospy.ROSException("Unknown relation_name %s"%relation_name)

    def score_samples(self, points):
        #converted_points = self.scoring_function(points, self.landmark.pose)
        converted_points = np.array([self.scoring_function(self.landmark.pose, x) for x in points])

        positive_scores     = self.positive.pdf(converted_points)
        negative_scores     = self.negative.pdf(converted_points)

        weighted_scores     = positive_scores - (0.5 * negative_scores)
        below_zero_indices  = weighted_scores < 0  # Where values are below 0
        weighted_scores[below_zero_indices] = MULTIPLYING_FACTOR
        return weighted_scores

    def __str__(self):
        center_normal_string   = "multivariate_normal(pose_x=%s, pose_y=%s)"%(self.positive, self.positive)
        repulse_normal_string  = "multivariate_normal(pose_x=%s, pose_y=%s)"%(self.negative, self.negative)
        return "TransferModel:\n\tPositive Multivariate: %s\n\tNegative Multivariate: %s"%(center_normal_string, repulse_normal_string)





class PreferenceModel(object):
    def __init__(self):
        super(PreferenceModel, self).__init__()
        self.positive_gmm           = None
        self.negative_gmm           = None
        self.weight                 = 1
        self.covar                  = 0.2
        self.good_pref_rel_points   = []
        self.bad_pref_rel_points    = []

    def add_preference_point(self, point_x, point_y, good_bool):

        if (good_bool):
            self.good_pref_rel_points.append([point_x, point_y])

            # if this is the first gaussian we overwrite the dummy one
            if (self.positive_gmm == None):
                gmm             = mixture.GMM(n_components=1, covariance_type='spherical')
                gmm.weights_    = np.array([self.weight])
                gmm.means_      = np.array([[point_x, point_y]])
                gmm.covars_     = np.array([[self.covar,self.covar]])

            # otherwise append a new gaussian
            else:
                gmm             = mixture.GMM(n_components=self.positive_gmm.n_components + 1, covariance_type='spherical')
                gmm.weights_    = np.append(self.positive_gmm.weights_, [self.weight], axis=0)
                gmm.means_      = np.append(self.positive_gmm.means_, [[point_x, point_y]], axis=0)
                gmm.covars_     = np.append(self.positive_gmm.covars_, [[self.covar,self.covar]], axis=0)
            
            self.positive_gmm   = gmm

        else: 
            self.bad_pref_rel_points.append([point_x, point_y])

            # if this is the first gaussian we overwrite the dummy one
            if (self.negative_gmm == None):
                gmm             = mixture.GMM(n_components=1, covariance_type='spherical')
                gmm.weights_    = np.array([self.weight])
                gmm.means_      = np.array([[point_x, point_y]])
                gmm.covars_     = np.array([[self.covar,self.covar]])

            # otherwise append a new gaussian
            else:
                gmm             = mixture.GMM(n_components=self.negative_gmm.n_components + 1, covariance_type='spherical')
                gmm.weights_    = np.append(self.negative_gmm.weights_, [self.weight], axis=0)
                gmm.means_      = np.append(self.negative_gmm.means_, [[point_x, point_y]], axis=0)
                gmm.covars_     = np.append(self.negative_gmm.covars_, [[self.covar,self.covar]], axis=0)

            self.negative_gmm   = gmm


    def score_samples(self, points):
        if (self.positive_gmm == None):
            positive_scores = np.empty(len(points))
            positive_scores.fill(1)
            
        else:
            logprobs, responsibilities = self.positive_gmm.score_samples(points)
            positive_scores = np.exp(logprobs)

        if (self.negative_gmm == None):
            negative_scores = np.empty(len(points))
            negative_scores.fill(1)
        else:
            logprobs, responsibilities = self.negative_gmm.score_samples(points)
            negative_scores = np.exp(logprobs)

        weighted_scores = positive_scores - (0.5 * negative_scores)
        below_zero_indices = weighted_scores < 0  # Where values are below 0
        weighted_scores[below_zero_indices] = MULTIPLYING_FACTOR

        return weighted_scores

    def __str__(self):
        if self.positive_gmm == None:
            positive_gmm_string = "GMM()"
        else:
            positive_gmm_string = "GMM(gmm_components=%s, gmm_weights=%s, gmm_means=%s, gmm_covars=%s)"%(self.positive_gmm.n_components, self.positive_gmm.weights_, str(self.positive_gmm.means_).replace("\n",""), str(self.positive_gmm.covars_).replace("\n",""))
        
        if self.negative_gmm == None:
            negative_gmm_string = "GMM()"
        else:
            negative_gmm_string = "GMM(gmm_components=%s, gmm_weights=%s, gmm_means=%s, gmm_covars=%s)"%(self.negative_gmm.n_components, self.negative_gmm.weights_, str(self.negative_gmm.means_).replace("\n",""), str(self.negative_gmm.covars_).replace("\n",""))
        return "PreferenceModel:\n\tPositive GMM: %s\n\tNegative GMM: %s\n"%(positive_gmm_string, negative_gmm_string)






class NearModel(object):
    def __init__(self, pose, width):
        super(NearModel, self).__init__()        
        self.centre = multivariate_normal([pose.position.x, pose.position.y], [width, width], allow_singular=True)
        self.repulse = multivariate_normal([pose.position.x, pose.position.y], [width/2, width/2], allow_singular=True)
        self.pose = [pose.position.x, pose.position.y]

    def score_samples(self, points):
        centre_scores = self.centre.pdf(points)
        repulse_scores = self.repulse.pdf(points)

        weighted_scores = centre_scores - (0.5 * repulse_scores)
        below_zero_indices = weighted_scores < 0  # Where values are below 0
        weighted_scores[below_zero_indices] = MULTIPLYING_FACTOR
        return weighted_scores

    def __str__(self):
        center_normal_string   = "multivariate_normal(pose_x=%s, pose_y=%s)"%(self.centre, self.centre)
        repulse_normal_string  = "multivariate_normal(pose_x=%s, pose_y=%s)"%(self.repulse, self.repulse)
        return "NearModel:\n\tMultivariate Normals centered at (%s,%s)\n"%(self.pose[0], self.pose[1])


    def sample(self, size=1):
        """
            Samples points from the model. 
            Smaples n * 100 points from the positive model, scores them on the negative model, then returns the best n
            This is a bit hacky but ok for testing.
        """
        # sample points from the centre
        centre_samples = self.centre.rvs(size=(100 * size))
        # score them using the repulsive score
        repulse_scores = self.repulse.pdf(centre_samples)     
        # get the indexes of the sorted list, lowest to highest
        indexes = np.argsort(repulse_scores)
        # and return the first n
        return centre_samples[indexes[:size]]