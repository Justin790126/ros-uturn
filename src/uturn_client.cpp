#include <ros/ros.h>                              // ROS Default Header File
#include <actionlib/client/simple_action_client.h>// action Library Header File
#include <actionlib/client/terminal_state.h>      // Action Goal Status Header File
#include <uturn/UturnAction.h> // UturnAction Action File Header
#include <geometry_msgs/Twist.h>

int main (int argc, char **argv)          // Node Main Function
{
  ros::init(argc, argv, "action_client"); // Node Name Initialization
  ros::NodeHandle n;
  ros::Rate r(10);
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  // Action Client Declaration (Action Name: fxnbot_uturn)
  actionlib::SimpleActionClient<uturn::UturnAction> ac("uturn_server", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //wait for the action server to start, will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  uturn::UturnGoal goal; // Declare Action Goal

 geometry_msgs::Twist vel;

 
  vel.linear.x = 0.1;
  vel.angular.z = 0.4;
  goal.uturn_vel = vel;
  goal.elapsed_time = 3.14159/0.4;


  ac.sendGoal(goal);  // Transmit Action Goal

  // Set action time limit (set to 30 seconds)

    // actionlib::SimpleClientGoalState state = ac.getState();
    // do {
    //   ROS_INFO("action is doing...");
      

    // } while (ac.getState() != actionlib::SimpleClientGoalState::ABORTED && ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    vel.linear.x = vel.angular.z = 0;
    cmd_pub.publish(vel);
    r.sleep();
  }
  
  return 0;
}
