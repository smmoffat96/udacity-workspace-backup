#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // Read the image file
    Mat image = imread("/home/udacity-workspace-backup/opencv_practice/Red_Poppy.jpeg");

    if (image.empty()) // Check for failure
    {
        cout << "Could not open or find the image" << endl;
        system_category("pause"); // Wait for any key press
        return -1;
    }

    String windowName = "My HelloWorld Window"; // Name of window

    namedWindow(windowName); // Create a window

    imshow(windowName, image); // Show our image inside the created window

    waitKey(0); // Wait for any keystroke in the window

    destroyWindow(windowName); // Destroy the created window

    return 0;
}