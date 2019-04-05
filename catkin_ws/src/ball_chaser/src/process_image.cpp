#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
}

// Name of OpenCV image
static const std::string OPENCV_WINDOW = "Image window";

void view_image(const cv::Mat img) {
    // This function takes in an OpenCV image in BGR color channels and outputs to a window
    cv::namedWindow(OPENCV_WINDOW);
    cv::imshow(OPENCV_WINDOW, img);
    waitKey(3);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    float lin_x;
    float ang_z;
    cv::Vec3b rgbpix;
    
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::Mat cv_img = cv_ptr->image;
    
    view_image(cv_img);
    
    int height = cv_img.rows;
    int width = cv_img.cols;
    int step = 0;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    // Convert image to gray scale
    cv::Mat gray;
    cv::cvtColor(cv_img, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    
    // Apply Hough Circle Transform to detect circles
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows/8, 200, 100, 0, 0);
    
    for (size_t i=0; i<circles.size(); i++) {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        circle(cv_img, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
        circle(cv_img, center, radius, Scalar(0,0,255), 3, 8, 0);
    }
    
    view_image(cv_img);
    
    /*
    // Go to each detected circle to determine if it is the white ball
    for( size_t i = 0; i < circles.size(); i++) {
        
        // Get position of center pixel of circle
        int x_pos = cvRound(circles[i][0]);
        int y_pos = cvRound(circles[i][1]);
        cv::Point center = cv::Point(x_pos, y_pos);
        
        // Check if center pixel is white
        cv::Scalar intensity = gray.at<uchar>(center);
        if (intensity[0] == 255) {
            // Find radius of circle
            int radius = cvRound(circles[i][2]);
            // If too big (ball is too close), stop driving
            if ((radius > width/3) || (radius > height/3)) {
                lin_x = 0;
                ang_z = 0;
            }
            else {
                // If left side, turn left
                if (x_pos <= width/3) {
                    lin_x = 0;
                    ang_z = 0.5;
                }
                // If middle, drive forward
                else if (x_pos <= 2*width/3) {
                    lin_x = 0.5;
                    ang_z = 0;
                }
                // If right, turn right
                else if (x_pos <= width) {
                    lin_x = 0;
                    ang_z = -0.5;
                }
            }
        }
        // If there are no white pixels, stop driving
        else {
            lin_x = 0;
            ang_z = 0;
        }
    }
    drive_robot(lin_x, ang_z);
}
*/

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
