#!/usr/bin/env python


# First thing is to publish a map centred at the location of the robot and visualise it

import rospy
from gmm_spatial_model.msg import SpatialPredicate
import gmm_spatial_model.srv as sm_srv
import sys
if __name__ == '__main__':
    rospy.init_node("spatial_model_client")

    if len(sys.argv) < 3:
        rospy.logfatal('Must have at least 3 args: spatial_model_client <predicate> <arg1> ... <argn>')
    else:    
        srv_name = '/spatial_model/get_pose'

        rospy.wait_for_service(srv_name)
        rospy.loginfo('waiting for %s' % srv_name)
        rospy.loginfo('        got %s' % srv_name)

        get_pose = rospy.ServiceProxy(srv_name, sm_srv.GetPoseForPredicate)
        try:
            pred = sys.argv[1]
            args = sys.argv[2:]
            rospy.loginfo('getting pose for: %s(%s)' % (pred, args))                        
            get_pose(SpatialPredicate(pred, args))
        except rospy.ServiceException as exc:
            print("Error calling service: " + str(exc))