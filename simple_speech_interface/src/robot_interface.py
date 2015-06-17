#!/usr/bin/env python

import sys
import rospy
from prolog_interface.srv import *
from prolog_interface.msg import *
from std_msgs.msg import String

def frame_to_event(frame):
    return frame.replace(" ", "").replace(",","_").replace("(","_").replace(")","")

def parse(command):
    concepts_unknown    = []
    command             = str(command.split(" ")).replace("\"","").replace("\'","").replace(" ","")

    try:
        result = prolog_service_handle ('parse', [command, "Frame"])

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return

    if len(result.ris) == 0:
        print "Robot: I did not understand your command"
        return

    Frame = result.ris[0].atoms[1]

    print "Frame: ", Frame
    pnp_event = frame_to_event(Frame)
    pnp_event_publisher.publish(pnp_event)
    print "Published %s"%pnp_event

if __name__ == "__main__":
    pnp_event_publisher         = rospy.Publisher('/PNPConditionEvent', String, queue_size=10)
    prolog_service_handle       = rospy.ServiceProxy('/prolog_interface/prolog_query', prologSrv)
    prolog_assert_handle        = rospy.ServiceProxy('/prolog_interface/prolog_assert', prologAssertSrv)
    prolog_retract_handle       = rospy.ServiceProxy('/prolog_interface/prolog_retract', prologRetractSrv)
    prolog_retractAll_handle    = rospy.ServiceProxy('/prolog_interface/prolog_retractall', prologRetractSrv)
    prolog_reloadAll_handle     = rospy.ServiceProxy('/prolog_interface/prolog_reload_all', prologReloadSrv)
    prolog_load_handle          = rospy.ServiceProxy('/prolog_interface/prolog_load', prologReloadSrv)
    rospy.init_node('robot_interface', anonymous=True)

    while True:
        command = raw_input("User: ")
        parse(command)