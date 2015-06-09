#!/usr/bin/env python


# First thing is to publish a map centred at the location of the robot and visualise it

import rospy
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
            print pred
            print args
            # resp1 = get_pose(x, y)
        except rospy.ServiceException as exc:
            print("Error calling service: " + str(exc))