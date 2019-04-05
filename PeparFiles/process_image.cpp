#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
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
      /*
	This function takes in an OpenCV image in BGR color channels and outputs to a window
      */
      
      cv::namedWindow(OPENCV_WINDOW);
      cv::imshow(OPENCV_WINDOW, img);
      cv::waitKey(3);
}



// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    /////////////////////////////////////////////////////////
    //I had to add a few declarations to get it to compile
    float lin_x;
    float ang_z;
    cv::Vec3b rgbpix;
    
    
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::Mat cv_img = cv_ptr->image;

    /////////////////////////////////////////////////////////
    //This function opens the OpenCV Window
    //You can pass in any cv::Mat object and it will display it in a window
    view_image(cv_img);
    /////////////////////////////////////////////////////////
    int height = 10; //placeholder
    int width = 10; //placeholder
    int step = 0; //placeholder
    ////////////////////////////////////////////////////////

    int white_pixel = 255;
    int white_pixel_count = 0;
    
    
    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    for (int i = 0; i < width; i++) {
        for (int j = 0; j < height; j++) {
            //I had to 
            rgbpix = cv_img.at<cv::Vec3b>(j, i);
            // Determine if the pixel is white
            //I updated this to access the pixel values from rgbpix
            if (rgbpix[0] == white_pixel & rgbpix[1] == white_pixel & rgbpix[2] == white_pixel) {
                white_pixel_count++;
                // if left side of image, turn left
                if (i < (step/3)) {
                    lin_x = 0.0;
                    ang_z = 0.5;
                }
                // if middle of image, drive forward
                else if (i < (2*step/3)) {
                    lin_x = 0.5;
                    ang_z = 0.0;
                }
                // if right side of image, turn right
                else if (i < step) {
                    lin_x = 0.0;
                    ang_z = -0.5;
                }
            }
        }

    }
    // if no white pixels, stop driving
    if (white_pixel_count == 0) {
        lin_x = 0.0;
        ang_z = 0.0;
    }

}



int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    //ImageConverter ic;
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    //ImageConverter ic;
    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
