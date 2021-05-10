#include <ros/ros.h>
#include <exp_assignment3/Num.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include <sstream>

#include <unistd.h>

ros::Publisher pub;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = pos_x;
  goal.target_pose.pose.position.y = pos_y;
  goal.target_pose.pose.orientation.w = 1.0;
  //goal.target_pose.pose.position.y = 0.1;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

//  ac.waitForResult();

//  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
 //   ROS_INFO("Hooray, the base moved 1 meter forward");
    //send a message to the state machine to alert that the target as been reached
  //  std_msgs::Int8 msg;
   // msg.data = 1;

 //   pub.publish(msg);
 //  }
//  else
 //   ROS_INFO("The base failed to move forward 1 meter for some reason");






}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ROS_INFO("sono nel main");

  ros::NodeHandle nh;


  ros::Subscriber sub = nh.subscribe("targetPosition",4,Callback);


  ros::spin();
  return 0;
}
