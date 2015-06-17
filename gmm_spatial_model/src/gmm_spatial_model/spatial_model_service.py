import rospy
import gmm_spatial_model.srv as sm_srv
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from scipy.stats import multivariate_normal
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from matplotlib.colors import LogNorm, Normalize
import matplotlib.pyplot as plt
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy
from soma_roi_manager.soma_roi import SOMAROIQuery
from soma_msgs.msg import SOMAObject
from nav_goals_generator.srv import NavGoals
from mongodb_store.message_store import MessageStoreProxy
from math import atan2, pi
from tf.transformations import quaternion_from_euler
from sklearn import mixture


def predicate_to_key(predicate):
    return predicate.name + "_" + str(predicate.arguments).replace(",","_")[1:-1]

def angle_to_point(landmark, target):
    """ Returns the angle from landmark to target as a quarternion """
    dx = target.x - landmark.x
    dy = target.y - landmark.y
    rads = atan2(dy,dx)
    rads %= 2*pi    
    q = quaternion_from_euler(0, 0, rads)
    return Quaternion(*q)

def map_range(start, end, step):
    while start <= end:
        yield start
        start += step


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


class AggregateModel(object):
    def __init__(self, model):
        super(AggregateModel, self).__init__()
        self.models_list = [PreferenceModel(), model]

    def score_samples(self, points):
        weighted_scores = self.models_list[0].score_samples(points)

        for model in self.models_list[1:]:
            weighted_scores = np.multiply(weighted_scores, model.score_samples(points))

        print weighted_scores
        return weighted_scores

    def add_preference_point(self, point_x, point_y, good_bool):
        self.models_list[0].add_preference_point(point_x, point_y, good_bool)

    def __str__(self):
        string = ""
        for model in self.models_list:
            string = string + str(model) + "\n"
        return string


class PreferenceModel(object):
    def __init__(self):
        super(PreferenceModel, self).__init__()
        self.positive_gmm   = None
        self.negative_gmm   = None
        self.weight         = 1
        self.covar          = 0.2

    def add_preference_point(self, point_x, point_y, good_bool):
        if (good_bool):
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
            print positive_scores
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
        weighted_scores[below_zero_indices] = 0

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
        weighted_scores[below_zero_indices] = 0
        return weighted_scores

    def __str__(self):
        center_normal_string   = "multivariate_normal(pose_x=%s, pose_y=%s)"%(self.centre, self.centre)
        repulse_normal_string  = "multivariate_normal(pose_x=%s, pose_y=%s)"%(self.repulse, self.repulse)
        return "NearModel:\n\tMultivariate Normals centered at (%s,%s)\n"%(self.pose[0], self.pose[1])



class SpatialModelServer(object):

    def __init__(self, soma_map, soma_config, prefix='spatial_model'):
        super(SpatialModelServer, self).__init__()

        self.model_cloud        = rospy.Publisher("/spatial_model", PointCloud2, queue_size=1, latch=True)
        self.best_pose          = rospy.Publisher("/spatial_model_best_pose", PoseStamped, queue_size=1, latch=True)

        self.geospatial_store = GeoSpatialStoreProxy('geospatial_store', 'soma')
        self.soma_map = soma_map
        self.soma_config = soma_config
        self.soma_roi_query = SOMAROIQuery(self.soma_map, self.soma_config)
        self.soma_proxy = MessageStoreProxy(collection="soma")

        self.aggregate_models_dictionary = {}

        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service = getattr(self, attr)
                rospy.Service(prefix + "/" +attr[:-8], service.type, service)


    def get_soma_object(self, id):
        query = {'id': id, 'map': self.soma_map, 'config': self.soma_config}
        return self.soma_proxy.query(SOMAObject._type, message_query=query, single=True)[0]

    def get_best_pose(self, bounds, model, samples = 100):
        """ 
        Randomly samples poses within the bounds and returns the highest scoring sample according to the model. 
        """
   
        # subscribe to the service for generating nav goals
        rospy.loginfo('waiting for nav_goals')
        rospy.wait_for_service('nav_goals')
        rospy.loginfo('done')
        nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)

        inflation_radius = 0.5

        # generate legal poses in the bounds provided
        pose_array = nav_goals(samples, inflation_radius, bounds).goals.poses

        # score them with the model
        points = np.array([[pose.position.x, pose.position.y] for pose in pose_array])
        scores = model.score_samples(points)
        idx_of_max = scores.argmax()
        return pose_array[idx_of_max]

    def get_feedback_ros_srv(self, req):
        """
        Updates the preference model related to the spatial predicate and object id given a point and a good/bad feedback
        """

        key     = predicate_to_key(req.predicate)
        point_x = req.pose.pose.position.x
        point_y = req.pose.pose.position.y
        good    = req.good

        soma_obj = self.get_soma_object(req.predicate.arguments[0])

        if soma_obj is None:
            raise rospy.ROSException('The object with id %s is not in the SoMa message store' % landmark)

        if not (key in self.aggregate_models_dictionary):
            raise rospy.ROSException('Could not find aggregate model for key %s' % key)

        self.aggregate_models_dictionary[key].add_preference_point(point_x, point_y, good)   
        print "updated aggregate model:\n%s"%self.aggregate_models_dictionary[key]     

        if rospy.get_param('~visualise_model', True):
            map_width = 4
            model_pcloud = model_to_pc2(self.aggregate_models_dictionary[key], soma_obj.pose.position.x - map_width / 2, soma_obj.pose.position.y - map_width / 2, 0.02, map_width, map_width)
            self.model_cloud.publish(model_pcloud)    
             
        return True

    def get_pose_ros_srv(self, req):
        """
        Generates a pose that sastisfies the given spatial predicate
        """

        # assume just near for now
        name = req.predicate.name
        args = req.predicate.arguments
        assert name == 'near'
        assert len(args) == 1
        
        # and the landmark is the 
        landmark = args[0]

        rois = self.geospatial_store.rois_containing_obj(landmark, self.soma_map, self.soma_config)

        if len(rois) == 0:
            raise rospy.ROSException('The object with id %s is not in a SoMa ROI' % landmark)

        # just choose the first ROI
        roi = rois[0]

        # create the model centred at the object given
        soma_obj = self.get_soma_object(landmark)

        if soma_obj is None:
            raise rospy.ROSException('The object with id %s is not in the SoMa message store' % landmark)            

        model = AggregateModel(NearModel(soma_obj.pose, 0.5))
        self.aggregate_models_dictionary[predicate_to_key(req.predicate)] = model

        if rospy.get_param('~visualise_model', True):
            map_width = 4
            pcloud = model_to_pc2(model, soma_obj.pose.position.x - map_width / 2, soma_obj.pose.position.y - map_width / 2, 0.02, map_width, map_width)
            self.model_cloud.publish(pcloud)

        bounds = self.soma_roi_query.get_polygon(roi)
        
        pose = self.get_best_pose(bounds, model)

        # rotate to point
        pose.orientation = angle_to_point(pose.position, soma_obj.pose.position)

        # stamp it so that we know the frame
        stamped_pose = PoseStamped(pose = pose)
        stamped_pose.header.stamp = rospy.get_rostime()
        stamped_pose.header.frame_id = 'map'



        if rospy.get_param('~visualise_model', True):
            self.best_pose.publish(stamped_pose)

        return stamped_pose

    get_pose_ros_srv.type=sm_srv.GetPoseForPredicate
    get_feedback_ros_srv.type=sm_srv.AddPoseSampleForPredicate

