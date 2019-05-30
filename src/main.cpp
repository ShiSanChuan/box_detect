#include <iostream>
#include "Eigen/Dense"
#include <opencv2/opencv.hpp>
#include <queue>
#include <future>
#include <algorithm>
#include <random>
#include <detect.h>
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

//h s v
cv::Scalar Red_min(134,59,80);
cv::Scalar Red_max(360,255,231);

cv::Scalar Green_min(22,37,22);
cv::Scalar Green_max(208,255,238);

int main(int argc, const char** argv){
	cv::Mat img,clone_img;
	detect detecter;

	// img=cv::imread("../../cap3.jpg");;
 // 	detecter.Getaxis(img);
	// cv::imshow("img" ,img);
	// cv::waitKey(0);
	// return 0;
	
	cv::VideoCapture cap;
	cap.open(1);
	double t=0.;
	std::string FPS="FPS";

	while(1){
		t=(double)cv::getTickCount();
			cap>>clone_img;
			
			img=clone_img.clone(); 
			cv::undistort(clone_img, img,CM,D);
		// 	
		
		detecter.Getaxis(img);


		 t = (double)cv::getTickFrequency()/((double)cv::getTickCount() - t) ;

		 cv::putText(img, FPS+std::to_string(t),cv::Point(5,20), cv::FONT_HERSHEY_SIMPLEX, 1,cv::Scalar(0,0,255) );

		cv::imshow("img" ,img);
		cv::waitKey(1);
	}

    return 0;
}