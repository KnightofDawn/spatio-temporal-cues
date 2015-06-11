import rospy
import gmm_spatial_model.srv as sm_srv
from geometry_msgs.msg import PoseStamped, Pose
from scipy.stats import multivariate_normal
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

def map_range(start, end, step):
    while start <= end:
        yield start
        start += step

def model_to_pc2(model, x_start, y_start, resolution, width, height):
    """
    Creates a PointCloud2 by sampling a regular grid of points from the given model.
    """
    pc = PointCloud2()
    pc.header.stamp = rospy.get_rostime()
    pc.header.frame_id = 'map'
 
    xy_points = []
    for x in map_range(x_start, x_start + width, resolution):
        for y in map_range(y_start, y_start + height, resolution):
            xy_points.append([x, y])
    
    probs = model.score_samples(xy_points)
    
    # and normalise to range to make the visualisation prettier
    normaliser = Normalize()
    normaliser.autoscale(probs)
    probs = normaliser(probs)

    colour_map = plt.get_cmap('jet')    
    colours = colour_map(probs, bytes=True)

    cloud = []
    for i in range(len(probs)):
        cloud.append([xy_points[i][0], xy_points[i][1], 2*probs[i], pack_rgb(colours[i][0], colours[i][1], colours[i][2])])
 
    return create_cloud_xyzrgb(pc.header, cloud)
  

def create_cloud_xyzrgb(header, points):
    """
    Create a L{sensor_msgs.msg.PointCloud2} message with 3 float32 fields (x, y, z).

    @param header: The point cloud header.
    @type  header: L{std_msgs.msg.Header}
    @param points: The point cloud points.
    @type  points: iterable
    @return: The point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgb', 12, PointField.UINT32, 1)]
    return pc2.create_cloud(header, fields, points)

def pack_rgb(r,g,b):
    return r << 16 | g << 8 | b


class Near(object):
    def __init__(self, pose, width):
        super(Near, self).__init__()        
        self.centre = multivariate_normal([pose.position.x, pose.position.y], [width, width], allow_singular=True)
        self.repulse = multivariate_normal([pose.position.x, pose.position.y], [width/2, width/2], allow_singular=True)

    def score_samples(self, points):
        centre_scores = self.centre.pdf(points)
        repulse_scores = self.repulse.pdf(points)

        weighted_scores = centre_scores - (0.5 * repulse_scores)
        below_zero_indices = weighted_scores < 0  # Where values are below 0
        weighted_scores[below_zero_indices] = 0
        return weighted_scores


class SpatialModelServer(object):

    def __init__(self, soma_map, soma_config, prefix='spatial_model'):
        super(SpatialModelServer, self).__init__()

        self.model_cloud = rospy.Publisher("/spatial_model", PointCloud2, queue_size=1, latch=True)
        self.geospatial_store = GeoSpatialStoreProxy('geospatial_store', 'soma')
        self.soma_map = soma_map
        self.soma_config = soma_config
        self.soma_roi_query = SOMAROIQuery(self.soma_map, self.soma_config)
        self.soma_proxy = MessageStoreProxy(collection="soma")

        # advertise ros services
        for attr in dir(self):
            if attr.endswith("_ros_srv"):
                service = getattr(self, attr)
                rospy.Service(prefix + "/" +attr[:-8], service.type, service)



    def get_soma_object(self, id):
        query = {'id': id, 'map': self.soma_map, 'config': self.soma_config}
        return self.soma_proxy.query(SOMAObject._type, message_query=query, single=True)[0]

    def get_pose_ros_srv(self, req):
        """
        Generates a pose that sastisfies the given spatial predicate
        """



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


        # subscribe to the service for generating nav goals
        rospy.loginfo('waiting for nav_goals')
        rospy.wait_for_service('nav_goals')
        rospy.loginfo('done')
        nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)
        
        points = 100
        inflation_radius = 0.5
        bounds = self.soma_roi_query.get_polygon(roi)

        resp = nav_goals(points, inflation_radius, bounds)

        # create the model centred at the object given
        soma_obj = self.get_soma_object(landmark)

        if soma_obj is None:
            raise rospy.ROSException('The object with id %s is not in the SoMa message store' % landmark)            

        model = Near(soma_obj.pose, 0.5)
        if rospy.get_param('~visualise_model', True):
            pcloud = model_to_pc2(model, -2, -2, 0.02, 4, 4)
            self.model_cloud.publish(pcloud)        


        return PoseStamped()             
    get_pose_ros_srv.type=sm_srv.GetPoseForPredicate

