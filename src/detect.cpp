#include <opencv2/opencv.hpp>
#include <iostream>
#include <future>
#include <random>
#include "detect.h"

float cameraMatrix[3][3]={480.0456075942316, 0, 328.4887103828126, 0, 478.8971079559076, 249.130831872676, 0, 0, 1};
float distCoeffs[5]={-0.3914272330750649, 0.136309583256524, -0.0008870578061134298, 0.0005048983403991557, 0};
cv::Mat CM=cv::Mat(3,3,CV_32FC1,cameraMatrix);
cv::Mat D=cv::Mat(1,5,CV_32FC1,distCoeffs);

cv::Scalar detect::GetColor(cv::Mat imgROI,int size){
	cv::Scalar color(0,0,0);
	int r=std::min(imgROI.cols,imgROI.rows);
	srand(time(NULL));
	for(int i=0;i<size;i++){
		int m=rand()%r-(r/2);
		for(int i=0;i<3;i++)
			color.val[i]+=imgROI.at<cv::Vec3b>(imgROI.cols/2+m,imgROI.rows+m)[i];
	}
	for(int i=0;i<3;i++)
		color.val[i]/=size;
	return color;
}

double detect::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0){
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / std::sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void detect::SetConfig(cv::Mat &_CM,cv::Mat &_D){
	CM=_CM.clone();
	D=_D.clone();
}

type detect::GetBoxType(cv::Scalar Color){
	int B= std::abs(Color.val[0]-255);
	int G=std::abs(Color.val[1]-255);
	int R=std::abs(Color.val[2]-255);
	long long disB=B*B+Color.val[1]*Color.val[1]+Color.val[2]*Color.val[2];
	long long disG=G*G+Color.val[0]*Color.val[0]+Color.val[2]*Color.val[2];
	long long disR=R*R+Color.val[0]*Color.val[0]+Color.val[1]*Color.val[1];
	long long min=std::min(disB,std::min(disG, disR));
	if(min>30000)return None;
	std::cout<<"CC"<<min<<std::endl;
	if(min==disR)return RED;
	if(min==disG)return GREEN;
	if(min==disB)return BLUE;
	return None;
}

double detect::pointdistance(cv::Point &P1,cv::Point &P2){
	double x_dis=P2.x-P1.x;
	double y_dis=P2.y-P1.y;
	return std::sqrt(x_dis*x_dis+y_dis*y_dis);
}

void detect::fixpoint(cv::Mat &img,std::vector<cv::Point> &point,int dd){
	int maxCorners=10;
	double qualityLevel = 0.1;
	double minDistance = 10;
	int blockSize = 6;
	bool useHarrisDetector = false;
    double k = 0.04;
	for(int i=0;i<4;i++){
		std::vector<cv::Point> corners;
		if(point[i].x<dd||point[i].y<dd||point[i].y+dd<img.rows||point[i].x+dd<img.cols)
			continue;
		cv::Mat imgROI(img(cv::Rect(point[i].x-dd,point[i].y-dd,dd*2,dd*2)));
		cv::cvtColor(imgROI, imgROI,cv::COLOR_RGB2GRAY);
		goodFeaturesToTrack(imgROI,corners,maxCorners,qualityLevel,minDistance,cv::Mat(),blockSize,useHarrisDetector,k);
		
		for (int j = 0; j < corners.size(); j++)
    	{
    		corners[j].x+=point[i].x-dd;
			corners[j].y+=point[i].y-dd;
			circle(img, corners[j], 2, cv::Scalar(255,0,0), -1, 8, 0);
		}
		std::cout<<"x "<<point[i].x<<" y"<<point[i].y<<"->"<<" x "<<corners[0].x<<" y"<<corners[0].y<<std::endl;
		if(pointdistance(point[i],corners[0])<8)
			point[i]=corners[0];
	}
}

void detect::AdjustContrastAndLight(cv::Mat & src,double alpha,double beta){
	float MaxCG=15;
	int k=7;
	cv::Mat meanlocal,varlocal,meanGlobal,varGlobal,gray;
	cv::cvtColor(src, gray, CV_BGR2GRAY);
	cv::blur(src, meanlocal, cv::Size(k,k));
	cv::Mat highfreq=src-meanlocal;
	varlocal=((cv::Mat)(highfreq.mul(highfreq))).clone();
	cv::blur(varlocal, varlocal, cv::Size(k,k));
	varlocal.convertTo(varlocal, CV_32FC3);
	for(int row=0;row<src.rows;row++){
		for(int col=0;col<src.cols;col++)
			if(src.channels()==3){
				for(int i=0;i<3;i++)
				varlocal.at<cv::Vec3f>(row,col)[i]=(float)std::sqrt(varlocal.at<cv::Vec3f>(row,col)[i]);
			}
	}
	cv::meanStdDev(gray, meanGlobal, varGlobal);
	cv::Mat gainArr=0.5* meanGlobal/varlocal;
	for(int row=0;row<src.rows;row++){
		for(int col=0;col<src.cols;col++)
			if(src.channels()==3){
				for(int i=0;i<3;i++)
					if(gainArr.at<cv::Vec3f>(row,col)[i]>MaxCG)
						gainArr.at<cv::Vec3f>(row,col)[i]=MaxCG;				
			}
	}
	gainArr.convertTo(gainArr, CV_8UC3);
	gainArr=((cv::Mat)gainArr.mul(highfreq)).clone();
	
	src=meanlocal+ gainArr ;

}
void detect::moveLightDiff(cv::Mat &img,int radius){
	cv::Mat img_tmp=img.clone();
	cv::Mat mask=cv::Mat::zeros(radius*2, radius*2, CV_8U);
	cv::erode(img_tmp, img_tmp, mask);
	cv::dilate(img_tmp, img_tmp, mask);
	img=img-img_tmp;
}

void detect::drawboxContours(cv::Mat & img,std::vector<cv::Point> &polygon,type _type){
	std::vector<cv::Point2f> image_points;
	std::vector<cv::Point3f> qurapoints;
	int size=polygon.size();
	if(polygon.size()!=4&&polygon.size()!=6)return;
	float high= -20;
	std::vector<cv::Point2f> _polygon;
	for(int i=0;i<size;i++)//bug
		_polygon.push_back(polygon[i]);

	cv::Scalar color;

	float p1,p2=20.;
	if(_type==RED){
		p1=30.0;
		color=cv::Scalar(0,0,255);
	}
	else if(_type==GREEN){
		p1=60.0;
		color=cv::Scalar(0,255,0);
	}
	else if(_type==BLUE){
		p1=120.0;
		color=cv::Scalar(255,0,0);
	}
	if(size==4){
		if(pointdistance(polygon[0],polygon[1])<pointdistance(polygon[0],polygon[size-1])){
			std::swap(p1, p2);
		}
		qurapoints.push_back(cv::Point3f(0.,0.,0.));
		qurapoints.push_back(cv::Point3f(p1,0.,0.));
		qurapoints.push_back(cv::Point3f(p1,p2,0.));
		qurapoints.push_back(cv::Point3f(0.,p2,0.));
		cv::solvePnP(qurapoints, _polygon, CM, D, rotation_vector, translation_vector,false,CV_ITERATIVE);
	}else if(size==6){
		if(pointdistance(polygon[0],polygon[1])<pointdistance(polygon[0],polygon[size-1])){
			std::swap(p1, p2);
			if(pointdistance(polygon[0],polygon[size-1])<pointdistance(polygon[1],polygon[2])){
				std::swap(p2, high);
			}
		}else{
			if(pointdistance(polygon[0],polygon[1])<pointdistance(polygon[1],polygon[2])){
				std::cout<<"??"<<std::endl;
				high=-high;
				std::swap(p1, high);
			}
		}

		qurapoints.push_back(cv::Point3f(0.,0.,0.));
		qurapoints.push_back(cv::Point3f(p1,0.,0.));
		qurapoints.push_back(cv::Point3f(p1,0.,high));
		qurapoints.push_back(cv::Point3f(p1,p2,high));
		qurapoints.push_back(cv::Point3f(0.,p2,high));
		qurapoints.push_back(cv::Point3f(0.,p2,0.));
		cv::solvePnP(qurapoints, _polygon, CM, D, rotation_vector, translation_vector,false,CV_EPNP);
	}
	std::cout<<"rotation:"<<std::endl<<rotation_vector<<std::endl;
	std::cout<<"translation:"<<std::endl<<translation_vector<<std::endl;
	std::vector<cv::Point3d> end_point3D;
	std::vector<cv::Point2d> end_point2D;
	if(size==4){
		end_point3D.push_back(cv::Point3d(0,0,high));
		end_point3D.push_back(cv::Point3d(p1,0,high));
		end_point3D.push_back(cv::Point3d(p1,p2,high));
		end_point3D.push_back(cv::Point3d(0,p2,high));
	}else if(size==6){
		end_point3D.push_back(cv::Point3d(0,0,high));
		end_point3D.push_back(cv::Point3d(p1,p2,0));
	}

 	projectPoints(end_point3D, rotation_vector, translation_vector, CM, D, end_point2D);
    
    if(size==4){	
		for(int i=0;i<size;i++){
			cv::line(img,polygon[i], end_point2D[i], color, 2);
			cv::line(img,end_point2D[i],end_point2D[(i+1)%size],color,2);
			cv::line(img,polygon[i] ,polygon[(i+1)%size], color,2);
		}
	}else if(size==6){
		for(int i=0;i<size;i++){
			// cv::putText(img, std::to_string(i),polygon[i], cv::FONT_HERSHEY_SIMPLEX, 1,color );
			cv::line(img,polygon[i] ,polygon[(i+1)%size], color,2);
		}
		for(int i=0;i<size;i+=2){
			cv::line(img,polygon[i%size] ,end_point2D[0], color,2);
			cv::line(img,polygon[(i+1)%size] ,end_point2D[1], color,2);
		}
	}
}
//h s v
const  cv::Scalar detect::Red_min(134,59./255,80./255);
const cv::Scalar detect::Red_max(360,255./255,231./255);

const cv::Scalar detect::Green_min(22,37./255,22./255);
const cv::Scalar detect::Green_max(208,255./255,238./255);

bool detect::three2four(std::vector<cv::Point> &polygon){


	return true;
}

int detect::Getaxisbyhsv(cv::Mat &img){
	cv::Mat hsv,bgr,bak_img;
	bak_img=img.clone();
	img.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
	cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

	auto func=[&](cv::Scalar min,cv::Scalar max,type _type){
		cv::Mat dst = cv::Mat::zeros(hsv.size(), CV_32FC3);
		cv::Mat mask;
		inRange(hsv, min, max, mask);
		for (int r = 0; r < bgr.rows; r++){
			for (int c = 0; c < bgr.cols; c++){
				if (mask.at<uchar>(r, c) == 255){
					dst.at<cv::Vec3f>(r, c) = bgr.at<cv::Vec3f>(r, c);
				}
			}
		}
		dst.convertTo(dst, CV_8UC3, 255.0, 0);
		medianBlur(dst,dst,3);
		// cv::cvtColor(dst, dst, CV_BGR2GRAY);
		cv::Canny(dst, dst, 10, 250,3);
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(dst, contours,hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE);
		
		cv::Mat BLACK_img(img.size(),CV_8UC1,cv::Scalar(0));

		cv::drawContours(BLACK_img, contours, -1, cv::Scalar(255),4);
		
		// cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
		// morphologyEx(BLACK_img,BLACK_img,cv::MORPH_CLOSE  ,kernel);

	 //    morphologyEx(BLACK_img,BLACK_img,cv::MORPH_DILATE  ,kernel);

	    contours.clear();
	    hierarchy.clear();
	    cv::findContours(BLACK_img, contours,hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE);
		// cv::drawContours(img, contours, -1, cv::Scalar(0,0,0),4);
		for(int i=0;i<static_cast<int>(contours.size());i++){
			int size=cv::contourArea(contours[i]);
			if(size<800)continue;
			std::vector<cv::Point> polygon;
			approxPolyDP(contours[i], polygon, arcLength(contours[i], 1)*0.01, 1);
			double area = fabs(contourArea(polygon));
			// if(area<1000||area>20000)continue;
			if(polygon.size()==6){
				drawboxContours(img,polygon,_type);
			}else if(polygon.size()==4){
				if(std::abs(angle(polygon[1] ,polygon[3], polygon[0])-
				angle(polygon[1] ,polygon[3], polygon[2]))>0.4)continue;
				drawboxContours(img,polygon,_type);
			}
		}

	};

	auto RedThread=std::async(std::launch::async,func,Red_min,Red_max,RED);
	auto GreenThread=std::async(std::launch::async,func,Green_min,Green_max,GREEN);

	RedThread.get();
	GreenThread.get();

	return 0;
}
int detect::Getaxis(cv::Mat &img){
	cv::Mat BLACK_img(img.size(),CV_8UC1,cv::Scalar(255));
	// cv::fastNlMeansDenoisingColored(img, img);
	cv::Mat bak_img=img.clone();
	cv::Mat gray;
	AdjustContrastAndLight(bak_img,0.8,-10);
	// medianBlur(bak_img,bak_img,3);
	cv::cvtColor(bak_img, gray, CV_BGR2GRAY);
	// cv::Mat grad_x,grad_y;
	// cv::Sobel(gray,grad_x,CV_16S,1, 0,3,1,0);
	// cv::Sobel(gray,grad_y,CV_16S,0, 1,3,1,0);
	// convertScaleAbs(grad_x,grad_x);
 //    convertScaleAbs(grad_y,grad_y);
 //    addWeighted(grad_x, 0.5, grad_y, 0.5, 0, gray);
    
	auto func=[&gray,&BLACK_img](int k){
		cv::Mat midimg;
		cv::GaussianBlur(gray, midimg, cv::Size(k,k),k,k);
		cv::Canny(midimg, midimg, 10, 250,3);
		// cv::Mat kernel;
		// kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  //   	morphologyEx(BLACK_img,BLACK_img,cv::MORPH_ERODE  ,kernel);
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(midimg, contours,hierarchy, CV_RETR_LIST , CV_CHAIN_APPROX_SIMPLE);
		cv::drawContours(BLACK_img, contours, -1, cv::Scalar(0),3);
	};

	auto GB1=std::async(std::launch::async,func,3);
	auto GB2=std::async(std::launch::async,func,5);
	auto GB3=std::async(std::launch::async,func,7);

	GB1.get();
	GB2.get();
	GB3.get();

	medianBlur(BLACK_img,BLACK_img,3);

	cv::Mat kernel;
	kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
	morphologyEx(BLACK_img,BLACK_img,cv::MORPH_CLOSE  ,kernel);

    morphologyEx(BLACK_img,BLACK_img,cv::MORPH_DILATE  ,kernel);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(BLACK_img, contours,hierarchy, CV_RETR_LIST , CV_CHAIN_APPROX_SIMPLE);

	// drawContours(img,contours,-1,cv::Scalar(0,255,0),1,8,hierarchy);
    
	for(int i=0;i<static_cast<int>(contours.size());i++){
		int size=cv::contourArea(contours[i]);
		if(size<800)continue;
		std::vector<cv::Point> polygon,hull;
		// convexHull(contours[i],hull);
		// approxPolyDP(cv::Mat(hull), polygon, arcLength(contours[i], 1)*0.02, 1);//
		approxPolyDP(contours[i], polygon, arcLength(contours[i], 1)*0.04, 1);//
		double area = fabs(contourArea(polygon));
		// if(polygon.size()==4)std::cout<<"poly "<<polygon.size()<<"size "<<size<<" area"<<area<<std::endl;
		if(polygon.size() == 4&&area>1000&&area<200000){
			
			if(std::abs(angle(polygon[1] ,polygon[3], polygon[0])-
				angle(polygon[1] ,polygon[3], polygon[2]))>0.4)continue;
			// std::cout<<"size:"<<size<<" area"<<area<<std::endl;

			cv::Rect rect=cv::boundingRect(contours[i]);
			cv::Scalar color=GetColor(bak_img(cv::Rect(rect.x+rect.width/4,rect.y+rect.height/4,
			rect.width/2,rect.height/2)));
			type boxtype=GetBoxType(color);
			if(BLUE==boxtype){
				std::cout<<"BLUE"<<std::endl;
			}else if(RED==boxtype){
				std::cout<<"RED"<<std::endl;
			}else if(GREEN==boxtype){
				std::cout<<"GREEN"<<std::endl;
			}else continue;
			fixpoint(bak_img,polygon);
			// for(int i=0;i<4;i++){
			// 	cv::line(img,polygon[i] , polygon[(i+1)%4],  cv::Scalar(0,0,0),2);
			// }
			drawboxContours(img,polygon,boxtype);

			auto mm=cv::moments(polygon,false);
			cv::circle(img, cv::Point(mm.m10/mm.m00,mm.m01/mm.m00), 6,  cv::Scalar(0,0,0));

		}
	}
	// cv::imshow("black_img", BLACK_img);
	// cv::waitKey(0);
	return 1;
}