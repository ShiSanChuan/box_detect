#include "task.h"
#include "detect.h"
#include <future>
void task::detect(cv::Mat &img){
	cv::Mat img1,img2;
	img1=img.clone();
	img2=img.clone();
	auto Get1=std::async(std::launch::async,[&](){
		Getaxisbyhsv(img1);
	});
	auto Get2=std::async(std::launch::async,[&](){
		Getaxis(img2);
	});
	Get1.get();
	Get2.get();
}

void task::draw(cv::Mat &img){
	for(int i=0;i<box_type;i++){
		if(boxs[i].size()==0)continue;
		float p1,p2=20.;
		float high= -20;
		cv::Scalar color;
		if(i==RED){
			p1=30.0;
			color=cv::Scalar(0,0,255);
		}
		else if(i==GREEN){
			p1=60.0;
			color=cv::Scalar(0,255,0);
		}
		else if(i==BLUE){
			p1=120.0;
			color=cv::Scalar(255,0,0);
		}
		for(auto box:boxs[i]){
			auto polygon=box.GetPoint4();
			for(int i=0;i<4;i++){
				cv::line(img,polygon[i], polygon[(i+1)%4], color, 2);
				cv::line(img,polygon[i+4], polygon[(i+1)%4+4], color, 2);
				cv::line(img,polygon[i], polygon[i+4], color, 2);
			}
		}

	}
}

void task::update(){
	for(int i=0;i<box_type;i++){
		if(boxs[i].size()<2)continue;
		std::sort(boxs[i].begin(), boxs[i].end(),[](box &a,box &b){
			return a.Getaxis2D().x>b.Getaxis2D().x;//#todo
		});
		boxs[i].erase(std::unique(boxs[i].begin(),boxs[i].end()),boxs[i].end());
	}
}
void task::updateEnviroment(cv::Mat rotation, cv::Mat transilation){

}

