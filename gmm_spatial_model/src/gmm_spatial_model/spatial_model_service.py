import rospy
import gmm_spatial_model.srv as sm_srv

class SpatialModelServer(object):

    def __init__(self, prefix='spatial_model'):
        super(SpatialModelServer, self).__init__()

        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service = getattr(self, attr)
                rospy.Service(prefix + "/" +attr[:-8], service.type, service)



    def get_pose_ros_srv(self, req):
        """
        Generates a pose that sastisfies the given spatial predicate
        """
        
        return None             
    get_pose_ros_srv.type=sm_srv.GetPoseForPredicate

