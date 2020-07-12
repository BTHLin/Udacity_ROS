#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
// #include <std_msgs/Float64.h>

/*TODO: Include the ball_chaser "DriveToTarget" header file*/
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

/*
Create a handle_drive_request callback function that executes whenever
a drive_bot service is requested. This function should publish the requested
linear x and angular velocities to the robot wheel joints. After publishing
the requested velocities, a message feedback should be returned with the
requested wheel velocities
 */
 bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
   ball_chaser::DriveToTarget::Response& res){

     // Announce receiving requests
     req.msg_feedback = "Received DriveToTarget Request" +
                        "linear_x: %1.2f, angular_z: %1.2f", (float)req.linear_x, (float)req.angular_z;
     ROS_INFO_STREAM(req.msg_feedback);
     ROS_INFO("Received DriveToTarget Request linear_x: %1.2f, angular_z: %1.2f",
              (float)req.linear_x, (float)req.angular_z);

     // Create a motor_command object of type geometry_msgs::Twist
     geometry_msgs::Twist motor_command;

     // Get maximum linear and angular speed from parameters
     ros::NodeHandle nh2; // assign a new modehandle
     std::string node_name = ros::this_node::getName(); // get node name
     float max_lin_x, max_ang_z; // Declare parameters
     nh2.getParam(node_name+"/max_linear_x", max_lin_x);
     nh2.getParam(node_name+"/max_angular_z", max_ang_z);

     // check if requested linear velocity is out of bound
     if (fabs(req.linear_x) > max_lin_x) {
       ROS_WARN("Requested linear_x is out of bound, reset to max_linear_x: %1.2f", max_lin_x);
       req.linear_x = max_lin_x;
     }
     //  check if requested angular velocity is out of bound
     if (fabs(req.angular_z) > max_ang_z) {
       ROS_WARN("Requested angular_z is out of bound, reset to max_angular_z: %1.2f", max_ang_z);
       req.angular_z = max_ang_z;
     }
     // Set wheel velocities from requested commands
     motor_command.linear.x = req.linear_x;
     motor_command.angular.z = req.angular_z;

     // Publish angles to drive the robot
     motor_command_publisher.publish(motor_command);

     // Return a response and confirm message
     res.msg_feedback =
     "motor commanded - linear_x: "  + std::to_string(req.linear_x) +
     " - angular_z: " + std::to_string(req.angular_z);
     ROS_INFO_STREAM(res.msg_feedback);

     return true;
 }


int main(int argc, char** argv) {

  // Initialize a ROS node
  ros::init(argc, argv, "drive_bot");

  // Create a ROS node handle
  ros::NodeHandle nh;

  // Define Publisher (Inform ROS master that we will be publishing a message
  // of type geometry_msgs::Twist on the robot actuation topic with a
  // publishing queue size of 10)
  motor_command_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Define Server: a drive /ball_chaser/command_robot service with a
  // handle_drive_request callback function
  ros::ServiceServer server = nh.advertiseService("ball_chaser/command_robot", handle_drive_request);
  ROS_INFO("Ready to send drive commands");

  // Handle ROS communication events
  ros::spin();


  return 0;
}
