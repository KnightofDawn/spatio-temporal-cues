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
import spatial_relation_graph
import support_functions


N_FEEDBACK_THRESHOLD = 2


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


    def get_target_object(self, id):
        query = {'id': id, 'map': self.soma_map, 'config': self.soma_config}
        return self.soma_proxy.query(SOMAObject._type, message_query=query, single=True)[0]


    def get_sample_poses(self, pose, model_name):
        good_glob_poses = []
        bad_glob_poses  = []

        good_rel_poses  = self.aggregate_models_dictionary[model_name].models_list[0].good_pref_rel_points
        bad_rel_poses   = self.aggregate_models_dictionary[model_name].models_list[0].bad_pref_rel_points

        target_x, target_y = support_functions.pose_to_xy(pose)


        for pose in good_rel_poses:
            good_glob_poses.append(support_functions.mkpose(pose[0],  pose[1]))

        for pose in bad_rel_poses:
            bad_glob_poses.append(support_functions.mkpose(pose[0],  pose[1]))

        return (good_glob_poses, bad_glob_poses)


    def get_similar_room_model_name(self, name, target_id):
        rois = self.geospatial_store.rois_containing_obj(target_id, self.soma_map, self.soma_config)

        if len(rois) == 0:
            raise rospy.ROSException('The object with id %s is not in a SoMa ROI' % target_id)

        for key in self.aggregate_models_dictionary:
            #FIXME also check that the target_id encoded in the key is in a room similar to the one in which the input target_id is located
            if key.startswith(name): 
                n_feedback_points = len(self.aggregate_models_dictionary[key].models_list[0].good_pref_rel_points) + len(self.aggregate_models_dictionary[key].models_list[0].bad_pref_rel_points)
                if n_feedback_points > N_FEEDBACK_THRESHOLD:
                    return key
        return None

    def get_similar_object(self, target_obj, objects_in_similar_room):
        print objects_in_similar_room
        for obj in objects_in_similar_room:
            print obj
            if obj.type == target_obj.type:
                return obj
        raise rospy.ROSException('Could not find a similar object of type %s in similar room'%target_obj.type)


    def get_objects_in_room(self, room_id):
        area_roi = self.geospatial_store.geom_of_roi(room_id, self.soma_map, self.soma_config)
        obj_list = self.geospatial_store.objs_within_roi(area_roi, self.soma_map, self.soma_config)

        if (obj_list == None):
            raise rospy.ROSException('Could not find any object in %s SoMa ROI' %room_id)

        result = []
        for obj in obj_list:
            result.append(self.get_target_object(obj["soma_id"]))

        return result

    def get_room_id_containing_object_id(self, target_id):
        print "retrieving room with object id '%s'"%target_id
        rois = self.geospatial_store.rois_containing_obj(target_id, self.soma_map, self.soma_config)

        if len(rois) == 0:
            raise rospy.ROSException('The object with id %s is not in a SoMa ROI' % target_id)

        # just choose the first ROI
        return rois[0]

    def get_room_id_from_model_name(self, model_name):
        object_id   = model_name.split("_")[1]
        return self.get_room_id_containing_object_id(object_id)

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

        target_obj = self.get_target_object(req.predicate.arguments[0])

        if target_obj is None:
            raise rospy.ROSException('The object with id %s is not in the SoMa message store' % target_id)

        if not (key in self.aggregate_models_dictionary):
            raise rospy.ROSException('Could not find aggregate model for key %s' % key)

        self.aggregate_models_dictionary[key].add_preference_point(point_x, point_y, good)   
        print "updated aggregate model:\n%s"%self.aggregate_models_dictionary[key]     

        if rospy.get_param('~visualise_model', True):
            map_width = 5
            model_pcloud = support_functions.model_to_pc2(self.aggregate_models_dictionary[key], target_obj.pose.position.x - map_width / 2, target_obj.pose.position.y - map_width / 2, 0.02, map_width, map_width)
            self.model_cloud.publish(model_pcloud)    
             
        return True


    def get_pose_ros_srv(self, req):
        """
        Generates a pose that sastisfies the given spatial predicate
        """
        # assume just near for now
        assert req.predicate.name == 'near'
        assert len(req.predicate.arguments) == 1

        name            = req.predicate.name
        target_id       = req.predicate.arguments[0]

        target_room_id  = self.get_room_id_containing_object_id(target_id)

        # create the model centred at the object given
        target_obj = self.get_target_object(target_id)

        if target_obj is None:
            raise rospy.ROSException('The object with id %s is not in the SoMa message store' % target_id)            

        model_name          = support_functions.predicate_to_key(req.predicate)
        similar_model_name  = self.get_similar_room_model_name(name, target_id)

        # if I know the model I use the previous one otherwise I instantiate a new model
        # if I am visiting a new room and the previous one has at least n points I transfer, otherwise I learn starting from my default model

        if (model_name in self.aggregate_models_dictionary):
            # I already have a model for this
            print "Model %s already known. Retrieving the old one..."%model_name
            model = self.aggregate_models_dictionary[model_name]

        elif (similar_model_name != None):
            # I can retrieve a similar model for the new room

            similar_room_id                     = self.get_room_id_from_model_name(similar_model_name)
            objects_in_similar_room             = self.get_objects_in_room(similar_room_id)
            objects_in_target_room              = self.get_objects_in_room(target_room_id)
            similar_target_obj                  = self.get_similar_object(target_obj, objects_in_similar_room)

            good_sample_poses, bad_sample_poses = self.get_sample_poses(similar_target_obj.pose, similar_model_name)

            objects_in_target_room              = support_functions.get_reordered_object_lists(objects_in_similar_room, objects_in_target_room)

            print "lista 1: "
            for i in objects_in_similar_room:
                print i.id

            print "lista 2:"
            for i in objects_in_target_room:
                print i.id


            print "Transferring model from room %s to room %s..."%(similar_room_id, target_room_id)
            
            default_model      = transfer.build_relational_models(similar_target_obj, target_obj, bad_sample_poses, good_sample_poses, objects_in_similar_room, objects_in_target_room, {'near': support_functions.distance,'relative_angle': support_functions.unit_circle_position}, self)

            model              = models.AggregateModel(default_model)

            self.aggregate_models_dictionary[model_name] = model

        else:
            # I use the default near model
            print "Uknown model %s initializing a new one..."%model_name
            model = models.AggregateModel(models.NearModel(target_obj.pose, 1.5))
            self.aggregate_models_dictionary[model_name] = model

        map_width   = 10
        pcloud      = support_functions.model_to_pc2(model, target_obj.pose.position.x - map_width / 2, target_obj.pose.position.y - map_width / 2, 0.04, map_width, map_width)
        self.model_cloud.publish(pcloud)

        bounds  = self.soma_roi_query.get_polygon(target_room_id)
        pose    = self.get_best_pose(bounds, model)

        # rotate to point
        pose.orientation = support_functions.quarternion_to_point(pose.position, target_obj.pose.position)

        # stamp it so that we know the frame
        stamped_pose = PoseStamped(pose = pose)
        stamped_pose.header.stamp = rospy.get_rostime()
        stamped_pose.header.frame_id = 'map'

        self.best_pose.publish(stamped_pose)

        return stamped_pose


    get_pose_ros_srv.type=sm_srv.GetPoseForPredicate
    get_feedback_ros_srv.type=sm_srv.AddPoseSampleForPredicate

