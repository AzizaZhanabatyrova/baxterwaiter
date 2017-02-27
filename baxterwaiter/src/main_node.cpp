//----------------------------------------------------------------//
//          Human Robot Cooperation with AND-OR graphs            //
//                                                                //
// Authors:     Clayton Leite, Aziza Zhanabatyrova                //
// Description: creates a clients for both endor_service and      //
//              baxter_move as well as a subscriber to the        //
//              topic /human to which the human is suppose        //
//              to post what object he/she has moved.             //
//                                                                //
//              The human should use the following command:       //
//                                                                //
//              rostopic pub -1 /human std_msgs/String "object"   //
//----------------------------------------------------------------//

#include "ros/ros.h"
#include "baxterwaiter/EndorService.h"
#include "baxterwaiter/MoveService.h"
#include <string>
#include "std_msgs/String.h"

std::string setNode = " ";
bool humanReplied = false;
bool humansTurn = true;

/** Receives the message of the object that has been moved by the human.
* Input args: capital letter that refers to which object was placed on the table*/
void humanCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Human placed: %s", msg->data.c_str());
	setNode = msg->data;
	humanReplied = true;
}

/// Creates two clients and a subscriber that interact one with the other
int main(int argc, char **argv)
{
	std::string obj = " ";

	ros::init(argc, argv, "main_node");
	ros::NodeHandle n;

	// Subscriber to the "human" topic
	ros::Subscriber sub = n.subscribe("human", 1, humanCallback);

	//Client of the Endor service
	ros::ServiceClient client = n.serviceClient<baxterwaiter::EndorService>("endor_service");
	baxterwaiter::EndorService srv;
	
	//Client of the Baxter movement service
	ros::ServiceClient client2 = n.serviceClient<baxterwaiter::MoveService>("baxter_move");
	baxterwaiter::MoveService srv2;
	ROS_INFO("Human robot cooperation project");
	ROS_INFO("S - spoon; P - plate; C - cutlery; G - glass");

	while(ros::ok()) {
		
		// If it is the human's turn,
		// wait until there's something published in the human topic
		if (humansTurn){
			ROS_INFO("Baxter is waiting for human`s turn");
			while (!humanReplied){
				ros::spinOnce();
			}
                	humanReplied = false;
			humansTurn = !humansTurn;
		}

		// Else if it is the robot's turn
		else {

			srv2.request.object = obj;

			if (client2.call(srv2))
			{
			   setNode = srv2.response.done;
			   ROS_INFO("Baxter placed: %s", setNode.c_str());
			}
			else
			{
			   ROS_ERROR("Failed to call service baxter_move");
			   return 1;
			}
			humansTurn = !humansTurn;
		}


		// Call Endor service to set a node as solved and receive as reply the next node
                // that should be solved, i.e., the next object that should be placed on the table.
		srv.request.set_node = setNode;

		if (client.call(srv))
		{
		   obj = srv.response.next_node;
		   if (obj == "end")
		   {
			ROS_INFO("The end!");
			break;
		   }
		   ROS_INFO("Next node is: %s", obj.c_str());
		}
		else
		{
		   ROS_ERROR("Failed to call service endor_service");
		   return 1;
		}
	}	
	return 0;
}
