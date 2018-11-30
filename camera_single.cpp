#include "camera_single.h"

using namespace std;

c_camera_single::c_camera_single(){

}

c_camera_single::c_camera_single(const char *calibrationFileName, const char *device_name, int width, int height)
{
    // Camera parameter allocations
    cameraMatrix = cv::Mat(3,3, CV_64F);
    distCoeffs = cv::Mat(5,1, CV_64F);
    // Frame size. Don't change the frame size if you didn't calibrate it for that spesific resolution.
    frameSize.width = width;
    frameSize.height = height;

    // Open the camera calibration file. File name is in the config file.
    cv::FileStorage file;
    file.open(calibrationFileName, cv::FileStorage::READ);
    if(!file.isOpened()){
        cout << "ERROR: File " << calibrationFileName << " could not be opened";
    }

    // Get the calibration parameters.
    file["Camera_Matrix"] >> cameraMatrix;
    file["Distortion_Coefficients"] >> distCoeffs;
    file.release();


    deviceName = std::string(device_name);
}

bool c_camera_single::update(cv::Mat &imgReturn){

     cv::Mat img;

    if(!capture.grab() ){
        cout << "Video camera is not working at the moment. Capture Error!";
        exit(1);
    }

    capture.retrieve(img);

    cv::Mat undistortedImage;
    cv::undistort(img, undistortedImage, cameraMatrix, distCoeffs);

    undistortedImage.copyTo(imgReturn);

    return true;
}
