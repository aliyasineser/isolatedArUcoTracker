#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"

class c_camera
{
public:
    c_camera();
    ~c_camera();
    //Update function for fixed stereo
    bool startCamera();
    bool stopCamera();

    // Camera variables
    cv::VideoCapture capture;
    std::string deviceName;
    cv::Size frameSize;
private:

};

#endif // CAMERA_H
