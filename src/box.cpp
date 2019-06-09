
#include "box.h"
void box::SetDistance(int distance){
	box::distance=distance;
}

cv::Point &box::Getaxis2D(){
	return axis2D;
}

cv::Mat &box::Getrotation(){
	return rotation;
}

cv::Mat &box::Gettranslation(){
	return translation;
}

std::vector<cv::Point> &box::GetPoint4(){
	return point4;
}

bool box::operator ==( box & other){
	double x_dis=this->Getaxis2D().x-other.Getaxis2D().x;
	double y_dis=this->Getaxis2D().y-other.Getaxis2D().y;
	if(std::sqrt(x_dis*x_dis+y_dis*y_dis)<100)return true;
	return false;
}

bool box::operator !=( box & other){
	return !(*this==other);
}