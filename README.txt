The baxterwaiter is a ROS software to be used with a Baxter robot with the goal of setting up a table with four objects. It makes use of the ENDOR library written by Barbara Bruno (https://github.com/EmaroLab/endor) 

Requirements:

- ROS Indigo or Kinetic
- Baxter SDK

How to use:

1. Copy the files to your workspace in ROS
2. Run catkin_make on your workspace
3. Run the baxter.sh in every terminal that is going to be used
4. Run rosrun baxter_interface joint_trajectory_action_server.py --mode velocity
5. Run rosrun baxterwaiter endor_service
6. Run rosrun baxterwaiter baxter_move
7. Run rosrun baxterwaiter main_node

After moving an object, the human has to report which object was moved by the means of the following bash command:

rostopic pub -1 /human std_msgs/String "P"

P - plate
S - spoon
C (cutlery) - fork and knife
G - glass

You can record your own playback files for the Baxter and replace with the ones available in the folder "resources". 

You can also change the costs inside the AND/OR graph by editing the file "table.txt" according to the standards defined by the ENDOR library. 