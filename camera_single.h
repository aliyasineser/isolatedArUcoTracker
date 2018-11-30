#ifndef CAMERA_SINGLE
#define CAMERA_SINGLE

#include "camera.h"

class c_camera_single : public c_camera{

public:
    c_camera_single();
    c_camera_single(const char *calibrationFileName, const char *device_name, int width, int height);
    bool update(cv::Mat &img);
    
private:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
};

#endif
