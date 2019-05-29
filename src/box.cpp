#include <algorithm>
#include <opencv2/opencv.hpp>
#include "Eigen/Dense"

enum Boxtype{
	GREEN,
	RED,
	BLUE,
	ORANGE,
	NONE
};

class box
{
private:
	
	Boxtype type; 
	cv::Scalar color;
	Eigen::AngleAxisf rotation;
	Eigen::Translation3f translation;
	//guess=(rotation*translation).matrix ()
	Eigen::Matrix4f  guess;
	cv::Point axis2D;//该方形中心像素在原图的中心位置
	int distance;//距离通过尺度变换估计距离

public:
	box(Boxtype _type,cv::Mat _rotation,cv::Mat _translation):type(_type) {

		
	}
	~box();
	
};