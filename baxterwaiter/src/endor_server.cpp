//-------------------------------------------------------------//
//          Human Robot Cooperation with AND-OR graphs         //
//                                                             //
// Authors:     Clayton Leite, Aziza Zhanabatyrova             //
// Description: creates a server responsible for contacting    //
//              the endor library to set a certain node        //
//              as solved and receiving a suggestion of the    //
//              next node that should be solved.               //
//                                                             //
//-------------------------------------------------------------//

#include "ros/ros.h"
#include "baxterwaiter/EndorService.h"
#include "aograph.h"
#include "ros/package.h"

string name = "DEFAULT";
AOgraph oneGraph(name);

bool C = false;
bool G = false;
bool P = false;
bool S = false;

bool set_CG = true;
bool set_CP = true;
bool set_CS = true;
bool set_GP = true;
bool set_GS = true;
bool set_PS = true;

bool set_CGP = true;
bool set_CPS = true;
bool set_GPS = true;
bool set_CGS = true;

bool set_CGPS = true;


/** Set node as solved and returns the next node that should be solved.
 Input  args: node to be set as solved.
 Output args: next node that should be solved. */
bool solve(baxterwaiter::EndorService::Request  &req,
         baxterwaiter::EndorService::Response &res)
{
  // Set node as solved
  oneGraph.solveByName(req.set_node);
  
  if (req.set_node == "P")
	P = true;
 
  if (req.set_node == "C")
	C = true;

  if (req.set_node == "G")
	G = true;

  if (req.set_node == "S")
	S = true;

  // Automatically set node corresponding to cutlery and plate together
  // once the cutlery and the plate are placed on the table
  if (P && C && set_CP && oneGraph.findByName("CP") != NULL){
	oneGraph.solveByName("CP");
	set_CP = false;
  }

  if (P && G && set_GP && oneGraph.findByName("GP") != NULL){
	oneGraph.solveByName("GP");
	set_GP = false;
  }

  if (C && G && set_CG && oneGraph.findByName("CG") != NULL){
	oneGraph.solveByName("CG");
  	set_CG = false;
  }   

  if (S && C && set_CS && oneGraph.findByName("CS") != NULL){
	oneGraph.solveByName("CS");
	set_CS = false;
  }

  if (G && S && set_GS && oneGraph.findByName("GS") != NULL){
	oneGraph.solveByName("GS");
	set_GS = false;
  }

  if (P && S && set_PS && oneGraph.findByName("PS") != NULL){
	oneGraph.solveByName("PS");
  	set_PS = false;
  }   

  if (C && G && P && set_CGP && oneGraph.findByName("CGP") != NULL){
	oneGraph.solveByName("CGP");
  	set_CGP = false;
  } 

  if (C && P && S && set_CPS && oneGraph.findByName("CPS") != NULL){
	oneGraph.solveByName("CPS");
  	set_CPS = false;
  }  
 
  if (G && P && S && set_GPS && oneGraph.findByName("GPS") != NULL){
	oneGraph.solveByName("GPS");
  	set_GPS = false;
  }   

  if (C && G && S && set_CGS && oneGraph.findByName("CGS") != NULL){
	oneGraph.solveByName("CGS");
  	set_CGS = false;
  }     

  if (C && G && P && S && set_CGPS){
	oneGraph.solveByName("CGPS");
	set_CGPS = false;
  }

  // Receives the suggestion of the next node that should be solved
  res.next_node = oneGraph.suggestNext(1);
  return true;
}

/// Creates a server and loads the graph description for the endor library.
int main(int argc, char **argv)
{
  ros::init(argc, argv, "endor_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("endor_service", solve);
  
  std::string package_path = ros::package::getPath("baxterwaiter").c_str();
  std::string path = package_path + "/resources/table.txt";
 
  oneGraph.loadFromFile(path);


  ROS_INFO("Endor Service node ready.\n");
  ros::spin();

  return 0;
}
