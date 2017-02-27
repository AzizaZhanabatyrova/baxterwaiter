//-------------------------------------------------------------//
//          Human Robot Cooperation with AND-OR graphs         //
//                                                             //
// Authors:     Clayton Leite, Aziza Zhanabatyrova             //
// Description: creates a server responsible for receiving     //
//              the action that the robot should perform and   //
//              once the action is done sends as reply the     //
//              object that has been placed.                   //
//                                                             //
//-------------------------------------------------------------//

#include <stdlib.h>
#include "ros/ros.h"
#include "baxterwaiter/MoveService.h"
#include "ros/package.h"

std::string node_to_run = "rosrun baxter_examples joint_trajectory_file_playback.py -f ";
std::string resources = "/resources/";
std::string cutlery = "cutlery";
std::string spoon = "spoon";
std::string plate = "plate";
std::string glass = "glass";
std::string path = ros::package::getPath("baxterwaiter").c_str() + resources;

/// Places an object on the table.
/// Input args:  the object that should be placed on the table.
/// Output args: the object that was placed on the table.
bool move(baxterwaiter::MoveService::Request  &req,
         baxterwaiter::MoveService::Response &res)
{
	ROS_INFO("Starting moving object %s.\n", req.object.c_str());
	
	if (req.object == "C")
		system((node_to_run + path + cutlery).c_str());
	
	if (req.object == "S")
		system((node_to_run + path +  spoon).c_str());
	
	if (req.object == "P")
		system((node_to_run + path + plate).c_str());
	
	if (req.object == "G")
		system((node_to_run + path + glass).c_str());
	
	system("rosrun baxter_tools tuck_arms.py -u");
	ROS_INFO("Finished moving object %s.\n", req.object.c_str());
	res.done = req.object;

	return true;
}

/// Creates the service responsible for the playback of movement of the actions
int main(int argc, char **argv)
{
  ros::init(argc, argv, "baxter_move");
  ros::NodeHandle n;
  

  ros::ServiceServer service = n.advertiseService("baxter_move", move);

  
  ROS_INFO("Baxter Movement node ready.\n");
  ros::spin();

  return 0;
}
