#include "camera.h"

using namespace std;

c_camera::c_camera()
{

}

bool c_camera::startCamera(){

    // Camera variables initiations. If there is a problem with the naming, change from config.
    capture = cv::VideoCapture(deviceName, cv::CAP_V4L2);

    if(!capture.isOpened()){
        cout << "camera is offline. Error!";
        exit(1);
    }

    // Set the cameras resolutions. Change res values in the config file.
    capture.set(CV_CAP_PROP_FRAME_WIDTH, frameSize.width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, frameSize.height);

    return true;
}

bool c_camera::stopCamera(){
    if (capture.isOpened()){
        capture.release();
    }
    return false;

}

c_camera::~c_camera(){
    stopCamera();
}
