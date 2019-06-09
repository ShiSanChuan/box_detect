#ifndef _BOX_H
#define _BOX_H

#include <algorithm>
#include <opencv2/opencv.hpp>
#include "Eigen/Dense"
//消除重复检测方块
//
enum type{
	GREEN,
	RED,
	BLUE,
	ORANGE,
	None
};

class box
{
private:
	
	type boxtype; 
	cv::Scalar color;
	Eigen::AngleAxisf Erotation;
	Eigen::Translation3f Etranslation;
	//guess=(rotation*translation).matrix ()
	Eigen::Matrix4f  Eguess;
	cv::Mat rotation;
	cv::Mat translation;
	std::vector<cv::Point> point4;
	cv::Point axis2D;//该方形中心像素在原图的中心位置
	int distance;//距离通过尺度变换估计距离
public:
	box(type _type,
		cv::Mat _rotation,cv::Mat _translation,cv::Point _axis2D,
		std::vector<cv::Point> _point4)
		:boxtype(_type),rotation(_rotation),translation(_translation),
		axis2D(_axis2D),point4(_point4){
		distance=-1;
	}
	void SetDistance(int distance);
	cv::Point  &Getaxis2D();
	cv::Mat &Getrotation();
	cv::Mat &Gettranslation();  
	std::vector<cv::Point> &GetPoint4();
	bool operator ==( box & other);
	bool operator !=( box & other);
};


#endif