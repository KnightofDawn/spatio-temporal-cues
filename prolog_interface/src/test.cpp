#include <ros/ros.h>
#include <vector>
#include <string>

#include "prolog_interface/prologSrv.h"

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<prolog_interface::prologSrv>("prolog_query");
  
  //initializing predicate and arguments for the query
  string predicate = "vertex";
  
  vector<string> arguments;
  arguments.push_back("X");
  arguments.push_back("Y");
  
  //initializing service message
  prolog_interface::prologSrv srv;
  srv.request.predicate = predicate.c_str();
  
  for(int i=0; i < (int)arguments.size(); ++i) {
    srv.request.arg.push_back(arguments[i].c_str());
  }
  
  //executing the request
  if (client.call(srv)) { 
    ROS_INFO("[TEST] query executed");
    for(int j=0; j < (int)srv.response.ris.size(); ++j) {
      ROS_INFO("[TEST] Solution %d", j+1);
      for(int z=0; z < (int)arguments.size(); z++) {
	if(isupper(arguments[z][0]))
	  ROS_INFO("[TEST] %s= %s", arguments[z].c_str(), (srv.response.ris[j].atoms[z]).c_str());
      }   
    }

  }
  else {
    ROS_ERROR("[TEST] error!");
  } 

  ros::spinOnce();

  return 0;
}
