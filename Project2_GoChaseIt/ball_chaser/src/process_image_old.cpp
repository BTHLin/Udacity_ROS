#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Found a ball! Moving robot to the ball.");

    // Request motor drive command
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // call the /ball_chaser/command_robot service and pass the requested motor commands
    if (!client.call(srv)){
      ROS_ERROR("Failed to call service ball_chaser/command_robot!");
    }
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
        ball_in_sight = true;
        // set forward speed to constant: 0.5
        vel_x = .5;
        //dynamically change the robot rotation speed base on the location of the ball
        rot_r = (.5*img.step - idx%img.step)/img.step;
        // if found ball, stop searching.
        break;

        /* if true, check if the current index is in the left 1/3 (< 0.33*img.step)
         or is in the right 1/3 (> 0.33*img.step)
         or, else, in the middle */
        // if ((idx%img.step) < 0.33*img.step) {
        //   // Drive to Left
        //   vel_x = 0.5;
        //   rot_r = 0.5;
        // } else if ((idx%img.step) > 0.66*img.step) {
        //   // Drive to Right
        //   vel_x = 0.5;
        //   rot_r = -0.5;
        // } else {
        //   // Drive forward
        //   vel_x = 0.5;
        //   rot_r = 0.; // set to 1, so robot can still rotate
        // }
        // after identifide the ball location, stop searching.

      }
    }

    // if not seeing ball, self rotate to find the ball
    if (ball_in_sight==false) {
      ROS_INFO_STREAM("Searching for ball...");
      vel_x = 0.; // stop the robot
      if (rot_r == 0.)
        rot_r = 1.;
    }

    // send drive_bot commands
    drive_robot(vel_x, rot_r);
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

    // Handle ROS communication events
    ros::spin();

    return 0;
}
