#!/usr/bin/env python
import itertools
import support_functions
from math import atan, degrees
from soma_msgs.msg import SOMAObject

# Distance considered optimal for relation near. We will consider neal all the objects that are inside NEAR_OPTIMAL_DISTANCE +/- DELTA_NEAR
NEAR_OPTIMAL_DISTANCE = 1
DELTA_NEAR = 0.5


class SpatialRelationGraph(object):
	"""docstring for SpatialRelationGraph"""
	def __init__(self, graph = []):
		super(SpatialRelationGraph, self).__init__()
		self.graph = graph
		
	def add_relation_to_graph(self, relation):
		self.graph.append(relation)

	def __str__(self):
		return "SpatialRelationGraph: %s"%(self.graph)



def get_angular_relation(soma_obj1, soma_obj2):
	x, y 	= support_functions.unit_circle_position(soma_obj1.pose, soma_obj2.pose)
	angle 	= atan(y/x)

	if (angle < 45) or (angle >= 315):
		angular_relation = "right"
	elif (angle < 135) and (angle >= 45):
		angular_relation = "behind"
	elif (angle < 225) and (angle >= 135):
		angular_relation = "left"
	else:
		angular_relation = "front"

	return [angular_relation, soma_obj1.type, soma_obj2.type]


def get_distance_relation(soma_obj1, soma_obj2):
	distance = support_functions.distance(soma_obj1.pose, soma_obj2.pose)

	if (distance >= NEAR_OPTIMAL_DISTANCE - DELTA_NEAR) and (distance <= NEAR_OPTIMAL_DISTANCE + DELTA_NEAR):
		distance_relation = "near"
	else:
		distance_relation = "far"

	return [distance_relation, soma_obj1.type, soma_obj2.type]


def create_spatial_relation_graph_from_roi(server, roi):
	obj_list = server.geospatial_store.objs_within_roi(roi, server.soma_map, server.soma_config)

	if (obj_list == None):
		raise rospy.ROSException('Could not find any object in %s SoMa ROI' %roi)
	elif (obj_list.count() == 1):
		raise rospy.ROSException('Could find only one object in %s SoMa ROI' %roi)

	graph = SpatialRelationGraph()

	for pair in itertools.permutations(obj_list, r=2):
		soma_obj1 = server.soma_proxy.query(SOMAObject._type, message_query={'id': pair[0]["soma_id"], 'map': server.soma_map, 'config': server.soma_config}, single=True)[0]
		soma_obj2 = server.soma_proxy.query(SOMAObject._type, message_query={'id': pair[1]["soma_id"], 'map': server.soma_map, 'config': server.soma_config}, single=True)[0]

		graph.add_relation_to_graph(get_angular_relation(soma_obj1, soma_obj2))
		graph.add_relation_to_graph(get_distance_relation(soma_obj1, soma_obj2))

	return graph

def transpose_function(classifier):
	for i in xrange(len(classifier.means_)):
		classifier.means_[i][1] = -1 * classifier.means_[i][1]


def identity_function(classifier):
	pass 


def get_spatial_relation_graph_function(relation, srg1, srg2, obj1, obj2):
	if (relation == "relative_angle"):
		return transpose_function
	else:
		return identity_function