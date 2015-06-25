#!/usr/bin/env python
import rospy
from gmm_spatial_model.msg import SpatialPredicate
import gmm_spatial_model.srv as sm_srv
import sys


def go(obj_id):
    pred = "near"
    args = [obj_id]

    try:
        rospy.loginfo('getting pose for: %s(%s)' % (pred, args))                        
        pose = get_pose(SpatialPredicate(pred, args)).pose
        rospy.loginfo("pose retrieved!")

    except rospy.ServiceException as exc:
        rospy.loginfo("Error calling service: " + str(exc))
        pose = None
        
    return pose

def give_feedback(obj_id, pose, feedback):
    pred = "near"
    args = [obj_id]
    predicate = SpatialPredicate(pred, args)

    try:
        rospy.loginfo("giving feedback '%s' for: %s(%s)..." % (feedback, pred, args))                        
        feedback_srv(predicate, pose, feedback)
        rospy.loginfo("feedback given!")
    except rospy.ServiceException as exc:
        rospy.loginfo("Error calling service: " + str(exc))


if __name__ == '__main__':
    rospy.init_node("spatial_model_client")
    
    rospy.loginfo("waiting for services '/spatial_model/get_feedback' and '/spatial_model/get_pose' to come up...")
    rospy.wait_for_service('/spatial_model/get_pose')
    rospy.wait_for_service('/spatial_model/get_feedback')

    get_pose        = rospy.ServiceProxy('/spatial_model/get_pose', sm_srv.GetPoseForPredicate)
    feedback_srv    = rospy.ServiceProxy('/spatial_model/get_feedback', sm_srv.AddPoseSampleForPredicate)

    rospy.loginfo("Commands: go <obj_id>, yes, no, quit")
    command = ""
    pose    = None
    obj_id  = None

    while (command != "quit"):
        command = raw_input("Command for the robot:")

        if command.startswith("go"):
            try:
                obj_id = command.split(" ")[1]
                pose = go(obj_id)
            except:
                rospy.loginfo("expecting command: go <obj_id>")

        elif command == "yes":
            if (pose != None):
                give_feedback(obj_id, pose, True)
            else:
                rospy.loginfo("before giving a feedback you need to send the robot somewhere!")

        elif command == "no":
            if (pose != None):
                give_feedback(obj_id, pose, False)
            else:
                rospy.loginfo("before giving a feedback you need to send the robot somewhere!")

        elif command == "quit":
            continue
        else:
            rospy.loginfo("I did not understand the command.\nPossible commands: go <obj_id>, yes, no, quit")

            
            



