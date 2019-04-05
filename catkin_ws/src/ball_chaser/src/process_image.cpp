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

    client.call(srv);
}

// Name of OpenCV image
static const std::string OPENCV_WINDOW = "Image window";

// This function takes in an OpenCV image in BGR color channels and outputs to a window
void view_image(const cv::Mat img) {
    cv::namedWindow(OPENCV_WINDOW);
    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(3);
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    float lin_x;
    float ang_z;
    cv::Vec3b rgbpix;
    
    // Convert ROS image message into OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::Mat cv_img = cv_ptr->image;
    
    // View image in new window
    view_image(cv_img);
    
    // Parameters of image
    int height = cv_img.rows;
    int width = cv_img.cols;
    int step = 0;
    
    ////////////////////////////////////////////////////
    // USE THRESHOLD AND BLOB TO DETECT WHITE BALL

    // Convert image to gray scale
    cv::Mat gray;
    cv::cvtColor(cv_img, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);

    // Set threshold and maxValue
    double thresh = 240;
    double maxValue = 255;

    // Binary Threshold
    cv::Mat binary;
    cv::threshold(gray, binary, thresh, maxValue, cv::THRESH_BINARY);

    // Find moments of the image
    cv::Moments m = cv::moments(binary, true);
    // Position of centroid
    cv::Point p(m.m10/m.m00, m.m01/m.m00);

    // Show image with point mark at center
    //cv::circle(cv_img, p, 5, cv::Scalar(128,0,0), -1);
    //view_image(cv_img);

    if (p.x <= width/3) {
        lin_x = 0.0;
        ang_z = 0.5;
    }
    else if (p.x <= 2*width/3) {
        lin_x = 0.5;
        ang_z = 0.0;
    }
    else if (p.x <= width) {
        lin_x = 0.0;
        ang_z = -0.5;
    }
    

/*
    //////////////////////////////////////////////////////////////////////
    // USE HOUGH CIRCLE TRANSFORM TO DETECT WHITE BALL
    
    // Convert image to gray scale
    cv::Mat gray;
    cv::cvtColor(cv_img, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    
    // Apply Hough Circle Transform to detect circles
    std::vector<cv::Vec3f> circles;
    //cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows/16, 200, 100, 0, width/3);
    
    // Go to each detected circle to determine if it is the white ball
    for( size_t i = 0; i < circles.size(); i++) {
        
        // Get position of centroid
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
                lin_x = 0.0;
                ang_z = 0.0;
            }
            else {
                // If left side, turn left
                if (x_pos <= width/3) {
                    lin_x = 0.0;
                    ang_z = 0.5;
                }
                // If middle, drive forward
                else if (x_pos <= 2*width/3) {
                    lin_x = 0.5;
                    ang_z = 0.0;
                }
                // If right, turn right
                else if (x_pos <= width) {
                    lin_x = 0.0;
                    ang_z = -0.5;
                }
            }
        }
        // If there are no white pixels, stop driving
        else {
            lin_x = 0.0;
            ang_z = 0.0;
        }
    }
*/

    // Pass velocity parameters to drive_robot
    drive_robot(lin_x, ang_z);
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
