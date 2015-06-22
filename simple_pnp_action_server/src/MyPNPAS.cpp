#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>
#include <gmm_spatial_model/SpatialPredicate.h>
#include <gmm_spatial_model/GetPoseForPredicate.h>

#include "MyActions.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define radians(a) ((a)/180.0*M_PI)
#define degrees(a) ((a)/M_PI*180.0)

class MyPNPActionServer : public PNPActionServer
{
private:
    ros::NodeHandle handle;
    ros::Publisher event_pub;
    ros::Subscriber laser_sub;
    ros::ServiceClient get_pose_for_predicate_client;
    
public:

    MyPNPActionServer() : PNPActionServer()
    { 
      // boost::thread t(boost::bind(&MyPNPActionServer::changeStatus, this));
    	event_pub = handle.advertise<std_msgs::String>("PNPConditionEvent", 10); 
    	laser_sub = handle.subscribe("scan", 10, &MyPNPActionServer::laser_callback, this);
      get_pose_for_predicate_client = handle.serviceClient<gmm_spatial_model::GetPoseForPredicate>("/spatial_model/get_pose");

    	// robotname external defined in MyActions.h/cpp
    	handle.param("robot_name",robotname,std::string(""));
    	ROS_INFO("ROBOTNAME: %s",robotname.c_str());
    	
    	register_action("init",&init);
    	register_action("gotopose",&gotopose);
    	register_action("home",&home);
    	register_action("wave",&wave);
    	register_action("sense1",&sense1);
      register_action("querydatabase",&MyPNPActionServer::querydatabase, this);	
      register_action("getbestpose",&MyPNPActionServer::getbestpose, this); 
    }
    
    
    virtual int evalCondition(string cond) {
      return PNPActionServer::evalCondition(cond);
    }
    
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      std::vector<float> scans;
      scans=std::vector<float>(msg->ranges);
      if (scans[scans.size()/2]<1.0) {
      	std_msgs::String cond;
      	cond.data = "obstacle";
      	event_pub.publish(cond);
      }
    }

    void querydatabase(string params, bool *run){
      cout << "executing querydatabase" << endl;
      std_msgs::String msg;
      std::stringstream ss;
      if (params == "nick")
        ss << "target_2";
      else
        ss << "target_7";
      msg.data = ss.str();
      event_pub.publish(msg);
      cout << "[MyPNPActionServer] Published: " << msg << endl;
    }

    void getbestpose(string params, bool *run){
      gmm_spatial_model::SpatialPredicate pred;
      double x;
      double y;
      double theta;

      int i               = params.find("_");
      pred.name           = params.substr(0,i);
      pred.arguments.push_back(params.substr(i+1));

      gmm_spatial_model::GetPoseForPredicate srv;
      srv.request.predicate = pred;

      if (get_pose_for_predicate_client.call(srv)) {
        x = srv.response.pose.pose.position.x;
        y = srv.response.pose.pose.position.y;
        theta = degrees(2*atan(srv.response.pose.pose.orientation.z/srv.response.pose.pose.orientation.w));
        //cout << srv.response.pose<< endl;
      }

      else {
        ROS_ERROR("Failed to call service /spatial_model/get_pose");
        return;
      }

      std_msgs::String msg;
      std::stringstream ss;
      ss << "target_" << x << "_" << y << "_" << theta;
      msg.data = ss.str();
      event_pub.publish(msg);
      cout << "[MyPNPActionServer] Published: " << msg << endl;
    }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mypnpas");

  MyPNPActionServer mypnpas;
  mypnpas.start();
  ros::spin();

  return 0;
}

