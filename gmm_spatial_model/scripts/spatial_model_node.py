#!/usr/bin/env python

import rospy
from gmm_spatial_model.spatial_model_service import SpatialModelServer
import argparse
import sys

if __name__ == '__main__':
    rospy.init_node("spatial_model_node")


    # TODO: add list command
    
    parser = argparse.ArgumentParser(prog='spatial_model_node.py')
    parser.add_argument("map", nargs=1, help='Name of the 2D map used')
    parser.add_argument("conf", nargs=1, help='Name of the soma configuration')    
    
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    
    sm = SpatialModelServer(args.map[0], args.conf[0])
    
    rospy.spin()

