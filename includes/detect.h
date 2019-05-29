#ifndef _DETECT_H
#define _DETECT_H
#include <opencv2/opencv.hpp>
enum type{
	GREEN,
	RED,
	BLUE,
	None
};

class detect{
private:
	
private:
	void Denoising();
	type GetBoxType(cv::Scalar Color);
	void fixpoint(cv::Mat &img,std::vector<cv::Point> &point,int dd=15);
	double pointdistance(cv::Point &P1,cv::Point &P2);
	cv::Scalar GetColor(cv::Mat imgROI,int size=200);
	void drawboxContours(cv::Mat & img,std::vector<cv::Point> &polygon,type _type);
	void AdjustContrastAndLight(cv::Mat & src,double alpha,double beta);
	double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
public:
	void SetConfig(cv::Mat &_CM,cv::Mat &_D);
	int Getaxis(cv::Mat &img);
	cv::Mat rotation_vector;
	cv::Mat translation_vector;
};

#endif