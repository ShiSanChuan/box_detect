#include<opencv2/opencv.hpp>

#include<iostream>
#include<string>
using namespace std;
using namespace cv;

float cameraMatrix[3][3]={480.0456075942316, 0, 328.4887103828126, 0, 478.8971079559076, 249.130831872676, 0, 0, 1};
float distCoeffs[5]={-0.3914272330750649, 0.136309583256524, -0.0008870578061134298, 0.0005048983403991557, 0};
cv::Mat CM=cv::Mat(3,3,CV_32FC1,cameraMatrix);
cv::Mat D=cv::Mat(1,5,CV_32FC1,distCoeffs);

int main(int argc, char const *argv[])
{
	cv::Mat frame,fiximage;
	cv::VideoCapture cap;
	cap.open(1);

	char key;
	int pic=0;
	while(1){
		cap>>frame;
		fiximage= frame.clone(); 
		cv::undistort(frame, fiximage,CM,D);
		cv::imshow("cap", fiximage);
		key=cv::waitKey(1);
		switch(key){
			case 'q':return 0;
			case 's':cv::imwrite("../../cap"+std::to_string(pic++)+".jpg",fiximage);break;
		}

	}
	return 0;
}