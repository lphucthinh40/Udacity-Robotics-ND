#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

constexpr float SIDE_REGION_PERCENT = 0.3;   // percentage of left & right region (horizontally)
constexpr float BOTTOM_REGION_PERCENT = 0.3; // percentage of the bottom-centered region (vertically)
constexpr int WHITE = 255;
constexpr int X_TOLERANCE = 30; // tolerance for the center point of the image (30/3=10 pixels)

// steering only
constexpr float LINEAR_X_STEER = 0.0;
constexpr float ANGULAR_Z_STEER = 0.1;
// forward only
constexpr float LINEAR_X_FORWARD = 0.4;
constexpr float ANGULAR_Z_FORWARD = 0.0;

// Define robot states
enum RobotState { STOP=0, FORWARD=1, STEER_LEFT=2, STEER_RIGHT=3};
RobotState state=STOP;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");
}

bool isBallReached(const sensor_msgs::Image* img, int& left, int& right)
{   int row_base = img->height*(1-BOTTOM_REGION_PERCENT)*img->step;
    // simply check if the ball has crossed the upper line of the bottom-centered region
    for (int i=left; i<right; i+=3)
    {   
        if (img->data[row_base+i]==WHITE && img->data[row_base+i+1]==WHITE && img->data[row_base+i+2]==WHITE) {
            return true;
        }
    }
    return false;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int left_limit  = img.step * SIDE_REGION_PERCENT;
    int right_limit = img.step * (1-SIDE_REGION_PERCENT);
    int center = img.step * 0.5;
    int x;
    
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // stop if the ball is close enough
    if (isBallReached(&img, left_limit, right_limit)) {
        if (state!=STOP) {
            drive_robot(0, 0);
            state=STOP;
        }
        return;
    }
    // otherwise find out where it is
    for (int i = 0; i < img.height * img.step; i+=3) {
        // we only need to find the first pixel that is white
        // (this pixel is guaranteed to be horizontally aligned with the ball center)
        if (img.data[i] == WHITE && img.data[i+1] == WHITE && img.data[i+2] == WHITE) {
            x = i%img.step;
            // ROS_INFO_STREAM("x:"+std::to_string(x));
            if (x<left_limit && state!=STEER_LEFT) {
                drive_robot(LINEAR_X_STEER, ANGULAR_Z_STEER);  //  steer to the left
                state=STEER_LEFT;
            }
            else if (x>=right_limit && state!=STEER_RIGHT) {
                drive_robot(LINEAR_X_STEER, -ANGULAR_Z_STEER); // steer to the right
                state=STEER_RIGHT;
            }
            else if (state!=FORWARD && x>=center-X_TOLERANCE && x<center+X_TOLERANCE) {
                drive_robot(LINEAR_X_FORWARD, ANGULAR_Z_FORWARD); // forward
                state=FORWARD;
            }
            return;
        }
    }
    // if the ball disappears, stop the vehicle
    if (state!=STOP) {
        drive_robot(0, 0);
        state=STOP;
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

    // Handle ROS communication events
    ros::spin();

    return 0;
}