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

public:
	box(Boxtype _type,cv::Mat _rotation,cv::Mat _translation):type(_type) {

		
	}
	~box();
	
};