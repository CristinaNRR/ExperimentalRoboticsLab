#include <ros/ros.h>
#include <exp_assignment3/Num.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <sstream>

#include <unistd.h>


/*!
A moveBase action client is implemented and used to send the desired
goal positions to the default moveBase action server
*/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*!
The program enters here everytime a new goal position is received
from the state machine. Once received, the positions(expressed with respect to the map frame)
are converted and sent to the moveBase action server.
*/
void Callback (const exp_assignment3::Num::ConstPtr& msg) { 

  ROS_INFO("sono nella callback");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

 //send the target received to the actionlib server
  double x = 1.0 * msg->num[0];
  double y = 1.0 * msg->num[1];



  double pos_x = {x};
  double pos_y = {y};

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = pos_x;
  goal.target_pose.pose.position.y = pos_y;
  goal.target_pose.pose.orientation.w = 1.0;


  ROS_INFO("Sending goal");
  ac.sendGoal(goal);


}


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ROS_INFO("sono nel main");

  ros::NodeHandle nh;


  ros::Subscriber sub = nh.subscribe("targetPosition",4,Callback);


  ros::spin();
  return 0;
}
