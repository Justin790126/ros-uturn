#include <ros/ros.h>                              // ROS Default Header File
#include <actionlib/server/simple_action_server.h>// action Library Header File
#include <uturn/UturnAction.h> // UturnAction Action File Header

class UturnAction
{
protected:

  ros::NodeHandle nh_;  // Node handle declaration
  actionlib::SimpleActionServer<uturn::UturnAction> as_; // Action server declaration, NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_; // Use as action name
  // Create messages that are used to published feedback/result
  uturn::UturnFeedback feedback_;
  uturn::UturnResult result_;
  ros::Publisher cmd_pub;


public:
  // Initialize action server (Node handle, action name, action callback function)
  UturnAction(std::string name) :
    as_(nh_, name, boost::bind(&UturnAction::executeCB, this, _1), false),
    action_name_(name)
  {
    cmd_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    as_.start();
  }

  ~UturnAction(void)
  {
  }

  // A function that receives an action goal message and performs a specified
  // action (in this example, a Uturn calculation).
  void executeCB(const uturn::UturnGoalConstPtr &goal)
  {
    ros::Rate r(10);       // Loop Rate: 1Hz
    bool success = true;  // Used as a variable to store the success or failure of an action
    ROS_INFO("utrun processing");

    // add first (0) and second message (1) of feedback.
    feedback_.state = -1;


    // Notify the user of action name, goal, initial two values of Uturn state
    ROS_INFO("=====%s: Executing=====", action_name_.c_str());
    ROS_INFO("cmd_vel: vel_lin_x = %f, ang_z = %f", goal->uturn_vel.linear.x,
             goal->uturn_vel.angular.z);
    ROS_INFO("elapsed time: %f", goal->elapsed_time);
    ROS_INFO("start point = %f, %f ----> end point = %f, %f",
             goal->start_point.x, goal->start_point.y, goal->start_point.theta,
             goal->end_point.x, goal->end_point.y, goal->end_point.theta);
    ROS_INFO("=======================", action_name_.c_str());

    double last_time = ros::Time::now().toSec();
    // Action contents
    while (true) {

      if (as_.isPreemptRequested() || !ros::ok())
      {
        // Notify action cancellation
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // Action cancellation and consider action as failure and save to variable
        as_.setPreempted();
        success = false;
        break;
      }



      if (ros::Time::now().toSec() - last_time > goal->elapsed_time) {
        success = true;
        break;
      } else {
        cmd_pub.publish(goal->uturn_vel);
        r.sleep();

        feedback_.state = 0;
        // publish the feedback
        as_.publishFeedback(feedback_);
      }
    }


    // If the action target value is reached,
    if(success)
    {
      result_.state = 1;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};

int main(int argc, char** argv)                     // Node Main Function
{
  ros::init(argc, argv, "uturn_server");           // Initializes Node Name
  UturnAction Uturn("uturn_server"); // Uturn Declaration (Action Name: fxnbot_uturn)
  ros::spin();                                      // Wait to receive action goal
  return 0;
}
