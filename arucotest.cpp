#include <iostream>
#include "camera_single.h"
#include "aruco.h"

using namespace std;

int main(int argc, char *argv[]){
	cout << "init" << endl;
	c_aruco module_aruco("cam.xml");
	c_camera_single module_camera("cam.xml","/dev/video0",640,480);
	cout << "start" << endl;
	module_camera.startCamera();
	cv::Mat img;
	cout << "loop start" << endl;
	while(true){
		module_camera.update(img);
		cout << "image taken" << endl;
		auto result = module_aruco.visionFindMarker(img);
		auto averaged = module_aruco.arucoUpdate(result);

		for(auto it = result.begin(); it != result.end(); ++it){
			cout << "X: " << it->x << " Y: " << it->y << " Z: " << it->z << endl;
		}

		cv::imshow("WindowName", img);
		char key = cv::waitKey(1);
		if(key == 'q') break;
	}

	cv::destroyAllWindows();
	module_camera.stopCamera();
	return 0;
}
