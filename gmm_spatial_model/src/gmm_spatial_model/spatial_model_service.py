#!/usr/bin/env python


import rospy
import gmm_spatial_model.srv as sm_srv
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
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
import models
import transfer
import support_functions



N_FEEDBACK_THRESHOLD = -1


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


    def get_sample_poses(self, pose, model_name):
        good_glob_poses = []
        bad_glob_poses  = []

        good_rel_poses  = self.aggregate_models_dictionary[model_name].models_list[0].good_pref_rel_points
        bad_rel_poses   = self.aggregate_models_dictionary[model_name].models_list[0].bad_pref_rel_points

        target_x, target_y = support_functions.pose_to_xy(pose)


        for pose in good_rel_poses:
            good_glob_poses.append(support_functions.mkpose(pose[0] + target_x,  pose[1] + target_y))

        for pose in bad_rel_poses:
            bad_glob_poses.append(support_functions.mkpose(pose[0] + target_x,  pose[1] + target_y))

        return (good_glob_poses, bad_glob_poses)


    def get_similar_room_model(self, name, landmark):
        for key in self.aggregate_models_dictionary:
            #FIXME also check that the landmark encoded in the key is in a room similar to the one in which the input landmark is located
            if key.startswith(name): 
                n_feedback_points = len(self.aggregate_models_dictionary[key].models_list[0].good_pref_rel_points) + len(self.aggregate_models_dictionary[key].models_list[0].bad_pref_rel_points)
                if n_feedback_points > N_FEEDBACK_THRESHOLD:
                    return key
        return None


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

        key     = support_functions.predicate_to_key(req.predicate)
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
            map_width = 5
            model_pcloud = support_functions.model_to_pc2(self.aggregate_models_dictionary[key], soma_obj.pose.position.x - map_width / 2, soma_obj.pose.position.y - map_width / 2, 0.02, map_width, map_width)
            self.model_cloud.publish(model_pcloud)    
             
        return True


    def get_pose_ros_srv(self, req):
        """
        Generates a pose that sastisfies the given spatial predicate
        """

        # used for debugging 
        transfer_model = True

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


        # if I know the model I use the previous one otherwise I instantiate a new model
        # if I am visiting a new room and the previous one has at least n points I transfer, otherwise I learn my usual model
        # transfer: with old model generate a bunch of good and bad poses

        similar_model_name = self.get_similar_room_model(name,landmark)

        if (support_functions.predicate_to_key(req.predicate) in self.aggregate_models_dictionary):
            # I already have a model for this
            print "Model %s already known. Retrieving the old one"%support_functions.predicate_to_key(req.predicate)
            model = self.aggregate_models_dictionary[support_functions.predicate_to_key(req.predicate)]

        elif (similar_model_name != None):
            # I can retrieve a similar model for the new room
            # FIXME change the hardcoded values
            print "Transferring model %s to model %s..."%(similar_model_name, support_functions.predicate_to_key(req.predicate))
            table               = self.get_soma_object('1')   
            chair               = self.get_soma_object('2')
            cabinet             = self.get_soma_object('3')  
            table1              = self.get_soma_object('6')   
            chair1              = self.get_soma_object('7')
            cabinet1            = self.get_soma_object('8') 
            # TODO: objects_list        = get_objects_list() 

            good_sample_poses, bad_sample_poses = self.get_sample_poses(soma_obj.pose, similar_model_name)


            bad_sample_poses = [support_functions.mkpose(-2, 0.3), support_functions.mkpose(-0.2, -0.5), support_functions.mkpose(-2, 0.2)]
            good_sample_poses =  [support_functions.mkpose(1.8, 1), support_functions.mkpose(1.9, 2.1), support_functions.mkpose(1.4, 0.5)]

            default_model      = transfer.build_relational_models(bad_sample_poses, good_sample_poses, [cabinet,table], [cabinet1, table1], {'near': support_functions.distance,'relative_angle': support_functions.unit_circle_position}, self)
            model              = models.AggregateModel(default_model)

            self.aggregate_models_dictionary[support_functions.predicate_to_key(req.predicate)] = model
            
            print "Generated new transferred model:\n%s"%default_model

        else:
            # I use the default near model
            print "Uknown model %s initializing a new one..."%support_functions.predicate_to_key(req.predicate)
            model = models.AggregateModel(models.NearModel(soma_obj.pose, 0.5))
            self.aggregate_models_dictionary[support_functions.predicate_to_key(req.predicate)] = model

        map_width = 10
        pcloud = support_functions.model_to_pc2(model, soma_obj.pose.position.x - map_width / 2, soma_obj.pose.position.y - map_width / 2, 0.02, map_width, map_width)
        self.model_cloud.publish(pcloud)
        #raw_input("Showing model for target position...")

        bounds = self.soma_roi_query.get_polygon(roi)
        
        pose = self.get_best_pose(bounds, model)

        # rotate to point
        pose.orientation = support_functions.quarternion_to_point(pose.position, soma_obj.pose.position)

        # stamp it so that we know the frame
        stamped_pose = PoseStamped(pose = pose)
        stamped_pose.header.stamp = rospy.get_rostime()
        stamped_pose.header.frame_id = 'map'



        if rospy.get_param('~visualise_model', True):
            self.best_pose.publish(stamped_pose)

        return stamped_pose


    get_pose_ros_srv.type=sm_srv.GetPoseForPredicate
    get_feedback_ros_srv.type=sm_srv.AddPoseSampleForPredicate

