#include <iostream>
#include "Eigen/Dense"
#include <opencv2/opencv.hpp>
#include <queue>
#include <future>
#include <algorithm>
#include <random>
#include "detect.h"
#include "realsense.h"
#include "task.h"
//30 20 20 red
//60 20 20 green
//120 20 20 blue
//180 20 20  orange
 
void Quattest(){

    Eigen::Quaterniond q(0,0,0,0);
    // q=R;
    Eigen::Quaterniond q1(1,0,0,0);

    Eigen::Quaterniond m=q.inverse()*q1;

    Eigen::Matrix3d Rx=m.toRotationMatrix();

    std::cout<< Rx<<std::endl;
}
enum test{
	IMG,
	CAMERA,
	REALSENSE
};
int main(int argc, const char** argv){
	cv::Mat img,clone_img,depth_img;
	task detecter;
	{
		//my camera
		float cameraMatrix[3][3]={480.0456075942316, 0, 328.4887103828126, 0, 478.8971079559076, 249.130831872676, 0, 0, 1};
		float distCoeffs[5]={-0.3914272330750649, 0.136309583256524, -0.0008870578061134298, 0.0005048983403991557, 0};
		//car camera
		// float cameraMatrix[3][3]={607.2899719341872, 0, 332.0274000453248, 0, 603.0529620927523, 170.7091705813268, 0, 0, 1};
		// float distCoeffs[5]={0.3527004374515632, -0.4300252641780203, -0.01493240498382748, -0.002778756238195155, 0};

		cv::Mat CM=cv::Mat(3,3,CV_32FC1,cameraMatrix);
		cv::Mat D=cv::Mat(1,5,CV_32FC1,distCoeffs);
		detecter.SetConfig(CM,D);
	}
	test ustest;
	cv::VideoCapture cap;
	realsense realsense;
	if(cap.open(1))ustest=CAMERA;
	else if(realsense.init()){
		float cameraMatrix[3][3]={645, 0, 648, 0, 331, 231, 0, 0, 1};
		float distCoeffs[5]={-0.004489,0.002333,0.169973,-0.357225,0};
		cv::Mat CM=cv::Mat(3,3,CV_32FC1,cameraMatrix);
		cv::Mat D=cv::Mat(1,5,CV_32FC1,distCoeffs);
		detecter.SetConfig(CM,D);
		ustest=REALSENSE;
	}
	else ustest=IMG;

	if(ustest==IMG){
		img=cv::imread("../../cap5.jpg");
		if (!img.data || img.channels() != 3){
			printf("error img name \n");
			// return -1;
		}
		
		detecter.detect(img);

		detecter.update();
		detecter.draw(img);

		cv::imshow("img" ,img);
		cv::waitKey(0);
		return 0;
	}
	
	double t=0.;
	std::string FPS="FPS";

	while(1){
		t=(double)cv::getTickCount();
			
			if(ustest==REALSENSE)
				realsense.get(img,depth_img);
			else{
				cap>>clone_img;
				img=clone_img.clone(); 
				cv::undistort(clone_img, img,CM,D);
			}

		detecter.detect(img);

		detecter.update();
		detecter.draw(img);

		 t = (double)cv::getTickFrequency()/((double)cv::getTickCount() - t) ;

		 cv::putText(img, FPS+std::to_string(t),cv::Point(5,20), cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0,0,255) );

		cv::imshow("img" ,img);
		cv::waitKey(1);
	}

    return 0;
}