#include <string>
#include <vector>
#include "ros/ros.h"
#include "prolog_interface/prologSrv.h"
#include "prolog_interface/prologAssertSrv.h"
#include "prolog_interface/prologReloadSrv.h"
#include "prolog_interface/prologRetractSrv.h"
#include "prolog_interface/solution.h"
#include "SWI-cpp.h"
#include <vector>

#define DEBUG

using namespace std;
vector<string> prologFiles;
string prologBin;
string prologPath;
PlEngine *e;

void parseRequest(std::string str, std::string &predicate, std::vector<string> &arguments){
    
    int i       = str.find("(");
    predicate   = str.substr(0,i).c_str();
    str         = str.substr(i+1, str.size()-i-2).c_str();
    
    std::stringstream test(str);
    std::string segment;
    std::vector<string> targuments;

    while(std::getline(test, segment, ',')) {
      arguments.push_back(segment);
    }
    
    std::string tstr;
    bool inList = 0;
    
    for (unsigned int i = 0; i<arguments.size(); ++i) {
      if ((arguments[i][0] != '[' && arguments[i][arguments[i].length()-1] != ']' && !inList) || 
         (arguments[i][0] == '[' && arguments[i][arguments[i].length()-1] == ']')){
        targuments.push_back(arguments[i]);
      }
      
      else if (arguments[i][0] == '[') {
        inList = 1;
        tstr = arguments[i];
      }
      
      else if (arguments[i][arguments[i].length()-1] == ']'){
        inList = 0;
        tstr += ","+arguments[i];
        targuments.push_back(tstr);
      }
      
      else if (inList) {
        tstr += ","+arguments[i]; 
      }
    }
    
    arguments = targuments;
}

bool executeQuery(prolog_interface::prologSrv::Request &req, prolog_interface::prologSrv::Response &res ) {

  bool hasSolution = false;
  string query(req.predicate);
  int tmpInt;
  
  query.append("(");
  for(int j=0; j < (int)req.arg.size(); ++j) {
    if(j < (int)req.arg.size()-1)
      query.append(req.arg[j]).append(",");
    else query.append(req.arg[j]);
  }
  query.append(")");
#ifdef DEBUG  
  ROS_INFO("[PROLOG] request for the execution of the query %s received", query.c_str());
#endif
  PlTermv argomentiVar((int)req.arg.size());
  for(int i=0; i<(int)req.arg.size(); ++i){
    if (isdigit(req.arg[i].c_str()[0]))
    {
      tmpInt = (int)atof(req.arg[i].c_str());
      argomentiVar[i] = tmpInt;
    }
    else if (req.arg[i][0] == '[')
    {
      PlTail list(argomentiVar[i]);
      string ret;
      string substring = req.arg[i].substr(1,req.arg[i].length()-2);
      char* tokens = strtok((char *)substring.c_str(), ",");
      while(tokens != NULL) {
	list.append(tokens);
	tokens = strtok(NULL,",");
      }
      list.close();
    }
    else if(isupper(req.arg[i][0]) == 0) 
    {
      PlTerm term(req.arg[i].c_str());
      argomentiVar[i] = term;
    }
  }
  PlString PlS = PlString(req.predicate.c_str());
  PlQuery eF(PlS, argomentiVar);
  
  try {
    int solCounter = 1;
    while(eF.next_solution()) {
#ifdef DEBUG
      ROS_INFO("[PROLOG] Solution %d", solCounter);
#endif  
      hasSolution = true;
      prolog_interface::solution currentSolution;
      for(int j=0; j<argomentiVar.size; ++j) {
	currentSolution.atoms.push_back(string((char *)argomentiVar[j]));
	if(isupper(req.arg[j][0])) {
#ifdef DEBUG
	  ROS_INFO("%s= %s", req.arg[j].c_str(), currentSolution.atoms[j].c_str()); 
#endif
	}
      }
      res.ris.push_back(currentSolution);
      solCounter++;
    }
  } catch (PlException &ex) {
      ROS_ERROR("[PROLOG]: %s", (char *)ex);

      return false;
  }

  return true;
}

bool prologAssert(prolog_interface::prologAssertSrv::Request &req, prolog_interface::prologAssertSrv::Response &res) {
  try 
  {
    PlFrame fr;
    PlTermv av(1);
#ifdef DEBUG
      ROS_INFO("[PROLOG] asserting %s", req.req.c_str());
#endif    

    std::string predicate;
    std::vector<string> arguments;
    int tmpInt;
    
    parseRequest(req.req, predicate, arguments);
    
    PlTermv argsForAssert(arguments.size());
    
    for(int i = 0; i < (int) arguments.size(); ++i)
    {
	if (atoi(arguments[i].c_str()))
	{
	  tmpInt = atoi(arguments[i].c_str());
	  argsForAssert[i] = tmpInt;
	}
	else if (arguments[i][0] == '[')
	{
	  PlTail list(argsForAssert[i]);
	  string ret;
	  string substring = arguments[i].substr(1,arguments[i].length()-2);
	  char* tokens = strtok((char *)substring.c_str(), ",");
	  while(tokens != NULL) {
	    list.append(tokens);
	    tokens = strtok(NULL,",");
	  }
	  list.close();
	}
	else if(!isupper(arguments[i].c_str()[0]))
	{
	  PlTerm term(arguments[i].c_str()); 
	  argsForAssert[i] = arguments[i].c_str();
	}
    }
    
    av[0] = PlCompound(predicate.c_str(), argsForAssert);
    PlQuery q("assert", av);
    q.next_solution();
    
    res.resp = "ACK_UPDATE";
    return true;
  } 
  catch (PlException &ex) 
  {
    ROS_ERROR("[PROLOG]: %s", (char *)ex);
    return false;
  }
}

bool prologRetract(prolog_interface::prologRetractSrv::Request &req, prolog_interface::prologRetractSrv::Response &res) {
  try 
  {
    PlFrame fr;
    PlTermv av(1);
#ifdef DEBUG
      ROS_INFO("[PROLOG] Retracting %s", req.req.c_str());
#endif
    std::string predicate;
    std::vector<string> arguments;
    int tmpInt;
    
    parseRequest(req.req, predicate, arguments);
    
    PlTermv argsForRetract(arguments.size());
    
    for(int i = 0; i < (int) arguments.size(); ++i)
    {
	if (atoi(arguments[i].c_str()))
	{
	  tmpInt = atoi(arguments[i].c_str());
	  argsForRetract[i] = tmpInt;
	}
	else if (arguments[i][0] == '[')
	{
	  PlTail list(argsForRetract[i]);
	  string ret;
	  string substring = arguments[i].substr(1,arguments[i].length()-2);
	  char* tokens = strtok((char *)substring.c_str(), ",");
	  while(tokens != NULL) {
	    list.append(tokens);
	    tokens = strtok(NULL,",");
	  }
	  list.close();
	}
	else if(!isupper(arguments[i].c_str()[0]))
	{
	  PlTerm term(arguments[i].c_str()); 
	  argsForRetract[i] = arguments[i].c_str();
	}
    }

    av[0] = PlCompound(predicate.c_str(), argsForRetract);
    PlQuery q("retract", av);
    q.next_solution();
    res.resp = "Done";
    return true;
  } 
  catch (PlException &ex) 
  {
    ROS_ERROR("[PROLOG]: %s", (char *)ex);
    return false;
  }
}

bool prologRetractAll(prolog_interface::prologRetractSrv::Request &req, prolog_interface::prologRetractSrv::Response &res) {
  try 
  {
    PlFrame fr;
    PlTermv av(1);
#ifdef DEBUG
      ROS_INFO("[PROLOG] RetractingAll %s", req.req.c_str());
#endif
    std::string predicate;
    std::vector<string> arguments;
    int tmpInt;
    
    parseRequest(req.req, predicate, arguments);
    
    PlTermv argsForRetract(arguments.size());
    for(int i = 0; i < (int) arguments.size(); ++i)
    {
	
	if (atoi(arguments[i].c_str()))
	{
	  tmpInt = atoi(arguments[i].c_str());
	  argsForRetract[i] = tmpInt;
	}
	else if (arguments[i][0] == '[')
	{
	  PlTail list(argsForRetract[i]);
	  string ret;
	  string substring = arguments[i].substr(1,arguments[i].length()-2);
	  char* tokens = strtok((char *)substring.c_str(), ",");
	  while(tokens != NULL) {
	    list.append(tokens);
	    tokens = strtok(NULL,",");
	  }
	  list.close();
	}
	else if(!isupper(arguments[i].c_str()[0]))
	{
	  PlTerm term(arguments[i].c_str()); 
	  argsForRetract[i] = arguments[i].c_str();
	}
    }

    av[0] = PlCompound(predicate.c_str(), argsForRetract);
    
    PlQuery q("retractall", av);
    q.next_solution();
    res.resp = "Done";
    return true;
  } 
  catch (PlException &ex) 
  {
    ROS_ERROR("[PROLOG]: %s", (char *)ex);
    return false;
  }
}

bool prologLoad(prolog_interface::prologReloadSrv::Request &req, prolog_interface::prologReloadSrv::Response &res) {
  try 
  { 
    string prologFinal = "consult('"+prologPath+"/"+req.req+"')";
    
    if(PlCall(prologFinal.c_str())) {
      ROS_INFO("[PROLOG] Ready to receive queries for file %s.", req.req.c_str()); 
    }
    else {
      ROS_ERROR("[PROLOG] unable to load %s file",  req.req.c_str());
    } 
    
    prologFiles.push_back(prologPath+"/"+req.req);
    res.resp = "ACK_LOAD";
    return true;
  } 
  catch (PlException &ex) 
  {
    ROS_ERROR("[PROLOG]: %s", (char *)ex);
    return false;
  }
}

bool prologReload(prolog_interface::prologReloadSrv::Request &req, prolog_interface::prologReloadSrv::Response &res) {
  try 
  { 
    string prologFinal = "consult('"+prologPath+"/"+req.req+"')";
    
    if(PlCall(prologFinal.c_str())) {
      ROS_INFO("[PROLOG] Ready to receive queries for file %s.", req.req.c_str()); 
    }
    else {
      ROS_ERROR("[PROLOG] unable to load %s file",  req.req.c_str());
    } 
    
    res.resp = "ACK_RELOAD";
    return true;
  } 
  catch (PlException &ex) 
  {
    ROS_ERROR("[PROLOG]: %s", (char *)ex);
    return false;
  }
}

bool prologReloadAll(prolog_interface::prologReloadSrv::Request &req, prolog_interface::prologReloadSrv::Response &res) {
  try 
  {
    for(vector<string>::const_iterator i = prologFiles.begin(); i != prologFiles.end(); ++i)
    {
      string prologFinal = "consult('" + *i + "')";
      
      if(PlCall(prologFinal.c_str())) {
        ROS_INFO("[PROLOG] Ready to receive queries for file %s.", req.req.c_str()); 
      }
      else {
        ROS_ERROR("[PROLOG] unable to load %s file",  req.req.c_str());
      } 
    }
    
    res.resp = "ACK_RELOAD";
    return true;
  } 
  catch (PlException &ex) 
  {
    ROS_ERROR("[PROLOG]: %s", (char *)ex);
    return false;
  }
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "prologInterface");
  	ros::NodeHandle np("~");
	
	ROS_INFO("Prolog interface started");
	
	prologBin="";
        prologPath = "";
        
	string prologFile="";
	string prologAttrFileValuePre="prolog_file_";
	int fileIndex = 1;
	stringstream ss;
	ss << prologAttrFileValuePre << fileIndex;
	string prologAttrFileValue = ss.str();

	np.getParam("prolog_binary", prologBin);
	np.getParam("prolog_path", prologPath);
	ROS_INFO("Prolog binary path: %s", prologBin.c_str());
	ROS_INFO("Prolog directory path: %s", prologPath.c_str());
	
	e = new PlEngine((char *)prologBin.c_str());
	
	while(np.getParam(prologAttrFileValue,prologFile)) {
	  ROS_INFO("Prolog %d file path: %s", fileIndex, prologFile.c_str());
	  
	  //updating global array for the file reloading
	  string toPush = prologPath+"/"+prologFile;
	  prologFiles.push_back(toPush);
	  
	  string prologFinal = "consult('"+prologPath+"/"+prologFile+"')";
	  
	  if(PlCall(prologFinal.c_str())) {
	    ROS_INFO("[PROLOG] Ready to receive queries for file %s.", prologFile.c_str()); 
	  }
	  else {
	    ROS_ERROR("[PROLOG] unable to load %s file", prologFile.c_str());
	  } 
	  fileIndex++;
	  ss.str("");
	  ss.clear();
	  prologAttrFileValue.clear();
	  ss << prologAttrFileValuePre << fileIndex;
	  prologAttrFileValue = ss.str();
	}
	
	ros::ServiceServer service = np.advertiseService("prolog_query", executeQuery);
	ros::ServiceServer service_assert = np.advertiseService("prolog_assert", prologAssert);
	ros::ServiceServer service_retract = np.advertiseService("prolog_retract", prologRetract);
	ros::ServiceServer service_retract_all = np.advertiseService("prolog_retractall", prologRetractAll);
        ros::ServiceServer service_reload_all = np.advertiseService("prolog_reload_all", prologReloadAll);
        ros::ServiceServer service_reload = np.advertiseService("prolog_reload", prologReload);
        ros::ServiceServer service_load = np.advertiseService("prolog_load", prologLoad);

	ros::spin(); 
	return 0;
}

