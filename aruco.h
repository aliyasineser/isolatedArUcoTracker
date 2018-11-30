#ifndef ARUCO_H
#define ARUCO_H

// OpenCV libraries
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/shape.hpp>
#include <opencv2/aruco.hpp>

#include <iostream>
#include <vector>
#include <list>

struct vision_offset
{
    int markerId;
    double x;
    double y;
    double z;
    double yaw;
};


class c_aruco
{
public:
    c_aruco(const char *calibrationFileName);
    c_aruco();
    ~c_aruco();

    public:
    /** Find Marker from the images. Normally left image is enough for that, signature will be updated.
     *  Returns vector of offsets which includes x,y,z and yaw from aruco centers to drone camera.
     */ 
    std::vector<vision_offset> visionFindMarker(const cv::Mat img);

    /** Uses offset to update and normalize values. 
     *  Returns normalized offsets for each marker, aruco centered.
     */
    std::vector<vision_offset> arucoUpdate(std::vector<vision_offset> offs); // Update function gets last aruco info and uses it for stabilization and storage

private:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    // Member variables
    std::vector<std::list<vision_offset>> offsetContainer; // Every elem is for one marker, second dimension for offsets frame by frame
    std::vector<vision_offset> normalizedCoords; // Smoothened coordinates. for each marker


    // TODO: PUSH TO CONFIG FILE -> marker side, landing constants
    #define NUMBEROFMARKERS 3

    // Side of the marker. Half of it not necessary but easier to read.
    static constexpr float sideOfTheMarker = 0.1;
    static constexpr double halfSideOfTheMarker = 0.05;

    // ArUco based landing constants
    int activeMarkers[NUMBEROFMARKERS] = {5, 6, 7}; // Markers those will be used for process. Others will be ignored.
    static constexpr int LIMITPOINTS = 5;  // Store limit for positions. If set to x, x offsets from the last x frames will be stored to calculate.

};

#endif // ARUCO_H
