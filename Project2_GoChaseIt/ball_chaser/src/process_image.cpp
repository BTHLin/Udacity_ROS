#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    // Request motor drive command
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // call the /ball_chaser/command_robot service and pass the requested motor commands
    if (!client.call(srv)){
      ROS_ERROR("Failed to call service ball_chaser/command_robot!");
    }
}

void get_current_velocity(const geometry_msgs::Twist speed) {
  // Please let me know if there is any way can pass "speed.angular.z" value into ball_searching function
  ROS_INFO_STREAM("ang_z: "  +std::to_string(speed.angular.z));
}

void ball_searching() {
  float vel_x = 0.; // stop the robot
  float rot_r = 0.5; // in the very rare scenario, the ball was at the center before being disapear.
                    // then, set to the Initialize point.

  // geometry_msgs::Twist speed;
  // ROS_INFO_STREAM(std::to_string(speed.angular.z));
  // if (speed.angular.z < 0.) {
  //   // it was yawing to right, continue the search on the right side
  //   rot_r = -1.;
  // } else if (speed.angular.z > 0.) {
  //   // it was yawing to left, continue the search on the left side
  //   rot_r = 1.;
  // }

  drive_robot(vel_x, rot_r);

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    // Because most of the walls are white, so change the ball colour to yellow.
    // Instead of stop the robot, I set the robot to self rotate to find the ball.

    // Initialization
    bool ball_in_sight = false;
    float vel_x, rot_r = .5; //1:left, -1:right

    // Get image size
    /* sensor image data = [[255,255,255],[255,255,255],.....[255,255,255]]
    but in C++ it is a 1-D array, so [255,255,255,255,255,255,...,255,255,255]
    So, the image size is img.height*img.step
    */
    int img_size = img.height * img.step;

    // loop over entire image, increment by 3, since it has RGB three channels
    for (int idx = 0; idx < img_size; idx+=3) {
      // check if any pixel is full yellow -->  RGB = 255, 255, 0
      if (img.data[idx]==255 && img.data[idx+1]==255 && img.data[idx+2]==0) {
        // found ball
        ROS_INFO_STREAM("Found a ball! Moving robot to the ball.");
        ball_in_sight = true;
        // set forward speed to constant: 0.5
        vel_x = .5;
        //dynamically change the robot rotation speed base on the location of the ball
        rot_r = (.5*img.step - idx%img.step)/img.step;

        // if found ball, stop scouting the image.
        break;
      }
    }

    // if not seeing ball, self rotate until find the ball
    if (ball_in_sight==false) {
      ROS_INFO_STREAM("Ball not found! Searching...");
      ball_searching();
    }else{
      // send drive_bot commands
      drive_robot(vel_x, rot_r);
    }
}


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    // Subscrube to /cmd_vel to read current angular velocity
    ros::Subscriber sub2 = n.subscribe("/cmd_vel", 10, get_current_velocity);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
