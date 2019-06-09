#ifndef _TASK_H
#define _TASK_H
#include "detect.h"

class task  :public detect
{
public:
	void draw(cv::Mat &img);
	void update();
	void updateEnviroment(cv::Mat rotation,cv::Mat transilation);
};


#endif