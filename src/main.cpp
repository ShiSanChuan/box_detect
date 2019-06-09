#include <iostream>
#include "Eigen/Dense"
#include <opencv2/opencv.hpp>
#include <queue>
#include <future>
#include <algorithm>
#include <random>
#include "detect.h"
#include "realsense.h"
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
	detect detecter;

	test ustest;
	cv::VideoCapture cap;
	realsense realsense;
	if(cap.open(1))ustest=CAMERA;
	else if(realsense.init())ustest=REALSENSE;
	else ustest=IMG;

	if(ustest==IMG){
		img=cv::imread("../../cap.jpg");
		if (!img.data || img.channels() != 3){
			printf("error img name \n");
			return -1;
		}
		cv::Mat img1=img.clone();
		cv::Mat img2=img.clone();
		detecter.Getaxisbyhsv(img1);
	 	detecter.Getaxis(img2);
		cv::imshow("img" ,img);
		cv::imshow("img1" ,img1);
		cv::imshow("img2" ,img2);
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
		// detecter.Getaxis(img);
		detecter.Getaxisbyhsv(img);
		 t = (double)cv::getTickFrequency()/((double)cv::getTickCount() - t) ;

		 cv::putText(img, FPS+std::to_string(t),cv::Point(5,20), cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0,0,255) );

		cv::imshow("img" ,img);
		cv::waitKey(1);
	}

    return 0;
}