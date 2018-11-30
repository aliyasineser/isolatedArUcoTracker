#include "aruco.h"

using namespace std;


c_aruco::c_aruco(const char *calibrationFileName){

    cameraMatrix = cv::Mat(3,3, CV_64F);
    distCoeffs = cv::Mat(5,1, CV_64F);

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


    // TODO: Get the info from the config file
    
    normalizedCoords = vector<vision_offset>(3);
    offsetContainer = vector<list<vision_offset>>();
    
    for(int i=0; i < NUMBEROFMARKERS; ++i)
        offsetContainer.push_back(list<vision_offset>());
    
}

c_aruco::c_aruco()
{
int test = 1;
}

c_aruco::~c_aruco(){

}

// Given rotation matrix ( calculate with Rodrigues from rvec ), calculates euler angles. Helper function
void getEulerAngles(cv::Mat &rotCamerMatrix, cv::Vec3d &eulerAngles){

    cv::Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                          _r[3],_r[4],_r[5],0,
                          _r[6],_r[7],_r[8],0};

    decomposeProjectionMatrix( cv::Mat(3,4,CV_64FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
}

// Get Mat from 3D vector.
cv::Mat DoubleMatFromVec3d(cv::Vec3d in)
{
    cv::Mat mat(3,1, CV_64FC1);
    mat.at <double>(0,0) = in [0];
    mat.at <double>(1,0) = in [1];
    mat.at <double>(2,0) = in [2];

    return mat;
};

// Create Homogeneous form for rvec and tvec. Helper
cv::Mat getRTMatrix ( const cv::Mat &R_,const cv::Mat &T_ ,int forceType ) {
   cv::Mat M;
   cv::Mat R,T;
   R_.copyTo ( R );
   T_.copyTo ( T );
   if ( R.type() ==CV_64F ) {
       assert ( T.type() ==CV_64F );
       cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_64FC1 );
       cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
       if ( R.total() ==3 ) {
           cv::Rodrigues ( R,R33 );
       } else if ( R.total() ==9 ) {
           cv::Mat R64;
           R.convertTo ( R64,CV_64F );
           R.copyTo ( R33 );
       }
       for ( int i=0; i<3; i++ )
           Matrix.at<double> ( i,3 ) =T.ptr<double> ( 0 ) [i];
       M=Matrix;
   } else if ( R.depth() ==CV_32F ) {
       cv::Mat Matrix=cv::Mat::eye ( 4,4,CV_32FC1 );
       cv::Mat R33=cv::Mat ( Matrix,cv::Rect ( 0,0,3,3 ) );
       if ( R.total() ==3 ) {
           cv::Rodrigues ( R,R33 );
       } else if ( R.total() ==9 ) {
           cv::Mat R32;
           R.convertTo ( R32,CV_32F );
           R.copyTo ( R33 );
       }
       for ( int i=0; i<3; i++ )
           Matrix.at<float> ( i,3 ) =T.ptr<float> ( 0 ) [i];
       M=Matrix;
   }
   if ( forceType==-1 ) return M;
   else {
       cv::Mat MTyped;
       M.convertTo ( MTyped,forceType );
       return MTyped;
   }
}


/*Find Marker
* Used to find all predefined aruco markers and return x,y,z and yaw values
* Return: offsets as vector
*/
vector<vision_offset> c_aruco::visionFindMarker(const cv::Mat img){
    
    vector<vision_offset> result; // The offset vector

    // Define the dictionary
    // TODO: Dictionary can change in needed. For that, it is better to move it to config file.
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_7X7_100));

    // Detection parameters and corner/id pairs.
    vector< vector< cv::Point2f > > corners;
    vector< int > ids;
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

    // Use ArUco detect Markers and define an Array for the order.
    cv::aruco::detectMarkers(img, dictionary, corners, ids, params);
    vector<vision_offset> markerOffsetArray = vector<vision_offset>();

    // Rotation and translation vectors for pose estimation. 
    // Rotation units are rodriguez angles, translation units are meters.
    vector< cv::Vec3d > rvecs, tvecs;

    cv::Mat invertedTvec;
    if(!ids.empty()){
        //aruco::drawDetectedMarkers(img, corners, noArray(), Scalar(0,0,250) ); // Debug
        cv::aruco::estimatePoseSingleMarkers(corners, sideOfTheMarker, cameraMatrix, distCoeffs, rvecs, tvecs); // Pose estimation

        for(int i = 0; i < ids.size(); ++i){
            // Invert to ArUco space
            cv::Mat homo = getRTMatrix(DoubleMatFromVec3d(rvecs[i]), DoubleMatFromVec3d(tvecs[i]), -1); // Get the homogeneous form
            cv::Mat invertedMat  = homo.inv(); // Get to ArUco space, thanks to Homogeneous matrix
            invertedTvec = invertedMat.col(3); // Get the Tvec column
            invertedTvec.pop_back(); // There reason for pop operation is to remove the 4th element, "1".

            // Get the angles 
            cv::Vec3d eulerAngles;
            cv::Mat rotMat = cv::Mat(3,3,CV_64F);
            cv::Mat tmp = invertedMat(cv::Rect(0,0,3,3)); // First 3x3 part is rotation matrix, always.
            tmp.copyTo(rotMat);
            getEulerAngles(rotMat, eulerAngles); 

            // It may be different.
            // TODO: CHECK IF THE ANGLES ARE IN WRONG PLACES.
            double pitch   = eulerAngles[0];
            double roll = eulerAngles[1];
            double yaw  = eulerAngles[2];

            // Debug
            /*
            cout << "For id: " << ids[i] << endl;
            cout << "yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << endl << endl;
            cout << "XYZ: " << invertedTvec << endl;
            */

            // Create an object for offset and push it.
            vision_offset offset;
            offset.markerId = ids[i];
            offset.x = invertedTvec.at<double>(0);
            offset.y = invertedTvec.at<double>(1);
            offset.z = invertedTvec.at<double>(2);
            offset.yaw = yaw;

            result.push_back(offset);
        }
    } 
    return result;
}


vector<vision_offset> c_aruco::arucoUpdate(vector<vision_offset> offs){

    if(offs.empty()) vector<vision_offset>(0); // TODO: Exception or correction

    // If the size exceeds limit, remove an element.
    for(int i = 0; i < offsetContainer.size(); ++i)
        if(offsetContainer.size() >= this->LIMITPOINTS)
            offsetContainer[i].pop_front();

    // Last frame drop, new frame in. 
    for(int i = 0; i < NUMBEROFMARKERS; ++i){
        bool isMarkerFound = false;
        for(vector<vision_offset>::iterator it = offs.begin(); it != offs.end(); ++it){
            if(it->markerId == this->activeMarkers[i]){
                offsetContainer[i].push_back(*it);
                isMarkerFound = true;
            }
        }

        if(!isMarkerFound && !offsetContainer[i].empty())
                offsetContainer[i].pop_front();
    }
    // Calculate the average for the markers
    for(int i = 0; i < NUMBEROFMARKERS; ++i){
        normalizedCoords[i].x = normalizedCoords[i].y = normalizedCoords[i].z = normalizedCoords[i].yaw = 0.0f;
        for(list<vision_offset>::iterator it = offsetContainer[i].begin(); it != offsetContainer[i].end(); ++it){
            normalizedCoords[i].x += it->x / offsetContainer[i].size();
            normalizedCoords[i].y += it->y / offsetContainer[i].size();
            normalizedCoords[i].z += it->z / offsetContainer[i].size();
            
            if(it->yaw >= 0)
                normalizedCoords[i].yaw += it->yaw / offsetContainer[i].size(); // already positif degree
            else
                normalizedCoords[i].yaw += ( it->yaw + 360.0) / offsetContainer[i].size(); // negative to positive space
            

        }
        normalizedCoords[i].markerId = this->activeMarkers[i];
    }

    return normalizedCoords;
}
