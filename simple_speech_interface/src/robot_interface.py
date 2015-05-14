#!/usr/bin/env python

import sys
import rospy
from prolog_interface.srv import *
from prolog_interface.msg import *

def parse(command):
    concepts_unknown    = []
    command             = str(command.split(" ")).replace("\"","").replace("\'","").replace(" ","")

    try:
        result = prolog_service_handle ('parse', [command, "Action", "Object"])

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
    if len(result.ris) == 0:
        print "Robot: I did not understand your command"
        return

    Action = result.ris[0].atoms[1]
    Object = result.ris[0].atoms[2]

    if "unknown_action_" in Action:
        concepts_unknown.append(Action.replace("unknown_action_","").replace("_"," "))

    if "unknown_object_" in Object:
        concepts_unknown.append(Action.replace("unknown_action_","").replace("_"," "))

    print "Robot: ", Action, " ", Object
    

if __name__ == "__main__":
    prolog_service_handle       = rospy.ServiceProxy('/prolog_interface/prolog_query', prologSrv)
    prolog_assert_handle        = rospy.ServiceProxy('/prolog_interface/prolog_assert', prologAssertSrv)
    prolog_retract_handle       = rospy.ServiceProxy('/prolog_interface/prolog_retract', prologRetractSrv)
    prolog_retractAll_handle    = rospy.ServiceProxy('/prolog_interface/prolog_retractall', prologRetractSrv)
    prolog_reloadAll_handle     = rospy.ServiceProxy('/prolog_interface/prolog_reload_all', prologReloadSrv)
    prolog_load_handle          = rospy.ServiceProxy('/prolog_interface/prolog_load', prologReloadSrv)

    while True:
        command = raw_input("User: ")
        parse(command)