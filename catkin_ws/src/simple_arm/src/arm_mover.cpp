#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>

// Golable joints publisher variables
ros::Publisher joint1_pub, joint2_pub;

// This function checks and clamps the joint angles to a safe zone
std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2){
  // Define clamped joint angles and assign them to the requested ones
  float clamped_j1 = requested_j1;
  float clamped_j2 = requested_j2;

  // Get min and max joint patameters, then assigning them to their respective variables
  float min_j1, max_j1, min_j2, max_j2;

  // Assign a new node handle since we have no access to the main one
  ros::NodeHandle n2;

  // Get node name
  std::string node_name = ros::this_node::getName();

  // Get joints min and max parameters
  n2.getParam(node_name + "/min_joint_1_angle", min_j1);
  n2.getParam(node_name + "/max_joint_1_angle", max_j1);
  n2.getParam(node_name + "/min_joint_2_angle", min_j2);
  n2.getParam(node_name + "/max_joint_2_angle", max_j2);

  // Check if joint 1 is in the safe zone
  if (requested_j1 < min_j1 || requested_j1 > max_j1) {
    clamped_j1 = std::min(std::max(requested_j1, min_j1),max_j1);
    ROS_WARN("Joint 1 is out of bound, valid range (%1.2f, %1.2f) clamping to: %1.2f",
              min_j1, max_j1, clamped_j1);
  }

  // Check if joint 2 is in the safe zone
  if (requested_j2 < min_j2 || requested_j2 > max_j2) {
    clamped_j2 = std::min(std::max(requested_j2, min_j2),max_j2);
    ROS_WARN("Joint 2 is out of bound, valid range (%1.2f, %1.2f) clamping to: %1.2f",
              min_j2, max_j2, clamped_j2);
  }

  // output clamped joint angles in a clamped_data vector
  std::vector<float> clamped_data = {clamped_j1, clamped_j2};

  return clamped_data;
}



bool handle_safe_move_request(simple_arm::GoToPosition::Request& req,
  simple_arm::GoToPosition::Response& res){
    ROS_INFO("GoToPositionRequest received - j1%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);

    // check if the requested angles are in the safe zone, otherwise clamp them at the nearest angles
    std::vector<float> joints_angles = clamp_at_boundaries(req.joint_1, req.joint_2);

    // Publish clamped joint angles to the arm
    std_msgs::Float64 joint1_angle, joint2_angle; //declare the angles for publisher

    joint1_angle.data = joints_angles[0];
    joint2_angle.data = joints_angles[1];

    joint1_pub.publish(joint1_angle);
    joint2_pub.publish(joint2_angle);

    // wait 3 seconds for arm to response and settle down
    ros::Duration(3).sleep();

    // Return a response and confirm message
    res.msg_feedback = "joint angels set - joint_1: " + std::to_string(joints_angles[0])
     + " - joint_2: " + std::to_string(joints_angles[1]);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}


int main(int argc, char** argv) {
  // Initialize the arm_mover node and create a handle to it
  ros::init(argc, argv, "arm_mover");
  ros:: NodeHandle n;

  // Define two publishers to publish std_msgs::Float64 messages on joints respective topics
  joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
  joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

  // Define a safe_move service with a handle_safe_move_request callback function
  ros::ServiceServer service = n.advertiseService("/arm_mover/safe_move", handle_safe_move_request);
  ROS_INFO("Ready to send joint commands");

  // Handel ROS communication events
  ros::spin();

  return 0;
}
