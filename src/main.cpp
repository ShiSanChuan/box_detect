#include <iostream>
#include "Eigen/Dense"
#include <opencv2/opencv.hpp>
#include <queue>
#include <future>

//30 20 20 red
//60 20 20 green
//120 20 20 blue
//180 20 20  orange
 
struct Box
{	
	enum type{
		GREEN,
		RED,
		BLUE,
		None
	};
	Box(type _Btype,int _axis[3]){
		Btype=_Btype;
		if(_Btype==GREEN)color=cv::Scalar(0,255,0);
		if(_Btype==RED)color=cv::Scalar(0,0,255);
		if(_Btype==BLUE)color=cv::Scalar(255,0,0);
		axis[0]=_axis[0];
		axis[1]=_axis[1];
		axis[2]=_axis[2];
	}
	int axis[3];
	cv::Scalar color;
	type Btype;
};

void test1();

void Quattest(){

    Eigen::Quaterniond q(0,0,0,0);
    // q=R;
    Eigen::Quaterniond q1(1,0,0,0);

    Eigen::Quaterniond m=q.inverse()*q1;

    Eigen::Matrix3d Rx=m.toRotationMatrix();

    std::cout<< Rx<<std::endl;
}
std::vector<int> getVector(const cv::Mat &a){
	cv::Mat b;
	a.convertTo(b,CV_32F);
	return (std::vector<int>)b.reshape(1,1);
}
//h s v
cv::Scalar Red_min(134,59,80);
cv::Scalar Red_max(360,255,231);

cv::Scalar Green_min(22,37,22);
cv::Scalar Green_max(208,255,238);

double pointdistance(cv::Point2f &P1,cv::Point2f &P2){
	double x_dis=P2.x-P1.x;
	double y_dis=P2.y-P1.y;
	return std::sqrt(x_dis*x_dis+y_dis*y_dis);
}

void AdjustContrastAndLight(cv::Mat & src,double alpha,double beta){
	// cv::Mat src;
	// image.copyTo(src,CV_32F);
	for(int row=0;row<src.rows;row++){
		for(int col=0;col<src.cols;col++)
			if(src.channels()==3){
				float b=src.at<cv::Vec3b>(row,col)[0];
				float g=src.at<cv::Vec3b>(row,col)[1];
				float r=src.at<cv::Vec3b>(row,col)[2];

				src.at<cv::Vec3b>(row,col)[0]=cv::saturate_cast<unsigned char>(b*alpha + beta);
				src.at<cv::Vec3b>(row,col)[1]=cv::saturate_cast<unsigned char>(g*alpha + beta);
				src.at<cv::Vec3b>(row,col)[2]=cv::saturate_cast<unsigned char>(r*alpha + beta);
			}else if(src.channels()==1){
				src.at<unsigned>(row,col)=cv::saturate_cast<unsigned char>(src.at<unsigned char>(row,col)*alpha+beta);
			}
	}
	// src.copyTo(image,CV_8UC3);
}

float cameraMatrix[3][3]={480.0456075942316, 0, 328.4887103828126, 0, 478.8971079559076, 249.130831872676, 0, 0, 1};
float distCoeffs[5]={-0.3914272330750649, 0.136309583256524, -0.0008870578061134298, 0.0005048983403991557, 0};
cv::Mat CM=cv::Mat(3,3,CV_32FC1,cameraMatrix);
cv::Mat D=cv::Mat(1,5,CV_32FC1,distCoeffs);

void drawboxContours(cv::Mat & img,std::vector<cv::Point2f> &polygon,Box::type _type){
	std::vector<cv::Point2f> image_points;
	std::vector<cv::Point3f> qurapoints;

	float p1,p2=20.;
	if(_type==Box::RED)
		p1=30.0;
	else if(_type==Box::GREEN)
		p1=60.0;
	else if(_type==Box::BLUE)
		p1=120.0;
	else p1=1.0;

	if(pointdistance(polygon[0],polygon[1])>pointdistance(polygon[0],polygon[3])){
		qurapoints.push_back(cv::Point3f(0.,0.,0.));
		qurapoints.push_back(cv::Point3f(p1,0.,0.));
		qurapoints.push_back(cv::Point3f(p1,p2,0.));
		qurapoints.push_back(cv::Point3f(0.,p2,0.));
	}else{
		qurapoints.push_back(cv::Point3f(0.,0.,0.));
		qurapoints.push_back(cv::Point3f(p2,0.,0.));
		qurapoints.push_back(cv::Point3f(p2,p1,0.));
		qurapoints.push_back(cv::Point3f(0.,p1,0.));
	}
	cv::Mat rotation_vector;
	cv::Mat translation_vector;
	cv::solvePnP(qurapoints, polygon, CM, D, rotation_vector, translation_vector);
	std::vector<cv::Point3d> end_point3D;
	std::vector<cv::Point2d> end_point2D;
	// int midlenth=linedistance()
	double high= -20;
	// // if(high>20)high=20;
	if(pointdistance(polygon[0],polygon[1])>pointdistance(polygon[0],polygon[3])){
    	end_point3D.push_back(cv::Point3d(0,0,high));
    	end_point3D.push_back(cv::Point3d(p1,0,high));
    	end_point3D.push_back(cv::Point3d(p1,p2,high));
    	end_point3D.push_back(cv::Point3d(0,p2,high));
	}else{
		end_point3D.push_back(cv::Point3d(0,0,high));
    	end_point3D.push_back(cv::Point3d(p2,0,high));
    	end_point3D.push_back(cv::Point3d(p2,p1,high));
    	end_point3D.push_back(cv::Point3d(0,p1,high));
	}
 	projectPoints(end_point3D, rotation_vector, translation_vector, CM, D, end_point2D);
    
    cv::line(img,polygon[0], end_point2D[0], cv::Scalar(255,0,0), 2);
    cv::line(img,polygon[1], end_point2D[1], cv::Scalar(255,0,0), 2);
	cv::line(img,polygon[2], end_point2D[2], cv::Scalar(255,0,0), 2);
	cv::line(img,polygon[3], end_point2D[3], cv::Scalar(255,0,0), 2);

	std::vector<cv::Point2f> srcpoint,aimpoint;

	for(int i=0;i<4;i++){
		aimpoint.push_back(cv::Point2f((double)end_point2D[i].x,(double)end_point2D[i].y));
		cv::line(img,end_point2D[i],end_point2D[(i+1)%4],cv::Scalar(0,0,255),2);
	}
}
void drwaAixs(){

}
cv::Scalar GetColor(cv::Mat imgROI){
	std::vector<cv::Mat> bgr_planes;
	cv::split(imgROI, bgr_planes);

	int histSize=256;
	float range[]={0,256};
	const float * histRanges={range};

	cv::Mat b_hist,g_hist,r_hist;
	cv::calcHist(&bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRanges, true, false);
	cv::calcHist(&bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRanges, true, false);
	cv::calcHist(&bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRanges, true, false);
	
	std::vector<int> R=getVector(r_hist);
	std::vector<int> G=getVector(g_hist);
	std::vector<int> B=getVector(b_hist);

	int max_b=std::max_element(B.begin(), B.end())-B.begin();
	int max_r=std::max_element(R.begin(), R.end())-R.begin();
	int max_g=std::max_element(G.begin(), G.end())-G.begin();

	std::cout<<":::"<<max_g<<":"<<G[max_g] << " "<<max_r<<":"<<R[max_r]<<" "<<max_b<<":"<<B[max_g]<<std::endl;
	return cv::Scalar(max_b,max_g,max_r);
}
Box::type GetBoxType(cv::Scalar Color){
	int B= std::abs(Color.val[0]-255);
	int G=std::abs(Color.val[1]-255);
	int R=std::abs(Color.val[2]-255);
	long long disB=B*B+Color.val[1]*Color.val[1]+Color.val[2]*Color.val[2];
	long long disG=G*G+Color.val[0]*Color.val[0]+Color.val[2]*Color.val[2];
	long long disR=R*R+Color.val[0]*Color.val[0]+Color.val[1]*Color.val[1];
	long long min=std::min(disB,std::min(disG, disR));
	if(min==disR)return Box::RED;
	if(min==disG)return Box::GREEN;
	if(min==disB)return Box::BLUE;
	return Box::None;
}
cv::Scalar RGB2HSV(cv::Scalar RGB){
	int H,S,V;
	int B=RGB.val[0];
	int G=RGB.val[1];
	int R=RGB.val[2];
	int max=std::max(R,std::max(G,B));
	int min=std::min(R,std::min(G,B));
	V=std::max(R,std::max(G,B));
	S=(max-min)/max;
	if(R==max)H =(G-B)/(max-min)* 60;
	if(G==max)H = 120+(B-R)/(max-min)* 60;
	if(B==max)H = 240 +(R-G)/(max-min)* 60;
	if(H < 0)H = H+ 360;
	return cv::Scalar(H,S,V);
}

int main(int argc, const char** argv){
	cv::Mat img,clone_img; 
	img=cv::imread("../../cap.jpg");;
	// cv::VideoCapture cap;
	// cap.open(1);

	// while(1){
	// 	cap>>clone_img;
	// 	img=clone_img.clone(); 
	// 	cv::undistort(clone_img, img,CM,D);

	cv::Mat hsv,midImage,finImage;
	cv::Mat kernel;
	cv::Mat bak_img(img);
	cv::Mat BLACK_img(img.size(),CV_8UC3,cv::Scalar(255,255,255));
	cv::fastNlMeansDenoisingColored(img, img);
	cv::Canny(img, midImage, 10, 250, 3);
	{
	    std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(midImage,contours, hierarchy,CV_RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
		cv::drawContours(BLACK_img, contours,-1, cv::Scalar(0,0,0),3);
	}
   	// kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    // morphologyEx(BLACK_img,BLACK_img,cv::MORPH_OPEN  ,kernel);

    // kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    // morphologyEx(BLACK_img,BLACK_img,cv::MORPH_DILATE  ,kernel);

    cv::cvtColor(BLACK_img, BLACK_img, cv::COLOR_RGB2GRAY);
	{
	    std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(BLACK_img,contours, hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
		for(int i=0;i<static_cast<int>(contours.size());i++){
			int size=cv::contourArea(contours[i]);
			std::vector<cv::Point2f> polygon;
			approxPolyDP(contours[i], polygon, arcLength(contours[i], 1)*0.02, 1);//arcLength(contours[i], 1)*0.02
    		double area = fabs(contourArea(polygon));
			if(polygon.size() == 4&&area>2000){
				std::cout<<"size:"<<size<<" area"<<area<<std::endl;
				cv::Rect rect=cv::boundingRect(contours[i]);
				cv::Scalar color=GetColor(bak_img(rect));
				// cv::drawContours(img, contours,i, cv::Scalar(0,0,0),2);
				if(Box::BLUE==GetBoxType(color)){
					std::cout<<"BLUE"<<std::endl;
				}
				if(Box::RED==GetBoxType(color)){
					std::cout<<"RED"<<std::endl;
				}
				if(Box::GREEN==GetBoxType(color)){
					std::cout<<"GREEN"<<std::endl;
				}
				for(int i=0;i<4;i++){
					cv::line(bak_img,polygon[i] , polygon[(i+1)%4],  cv::Scalar(0,0,0),2);
				}
				drawboxContours(bak_img, polygon,Box::RED);
				// cv::imshow("bak_img", bak_img);
				// cv::waitKey(0);
			}
		}
	}

	cv::imshow("demo", img);
	cv::imshow("midimge", midImage);
	cv::imshow("black_img", BLACK_img);
    cv::waitKey(0);

	// }


    return 0;
}


void test1(){
	cv::Mat img,img_x;
	img=cv::imread("../../21XBP.jpg");	
	img.copyTo(img_x);
	for(int i=0;i<25;i++){
		cv::Mat dst,edged;
		cv::threshold(img,dst, i*10, 255, cv::THRESH_TOZERO_INV);
		cv::cvtColor(dst, edged, cv::COLOR_BGR2GRAY);
		std::vector<std::vector<cv::Point> >contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(edged,contours, hierarchy,CV_RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
	
		for(int i=0;i<static_cast<int>(contours.size());i++){
			int size=cv::contourArea(contours[i]);
			if(size>500&&size<50000){
				cv::Rect rect= cv::boundingRect(contours[i]);
				if(rect.width>100 && rect.height>100){
					std::vector<cv::Point2f> polygon;
			    	approxPolyDP(contours[i], polygon, arcLength(contours[i], 1)*0.02, 1);//arcLength(contours[i], 1)*0.02
    				//计算框中颜色计算画框颜色
    				if(polygon.size()<6){
    					cv::Rect rect=cv::boundingRect(contours[i]);
    					cv::Mat imgROI=img(rect);
    					std::vector<cv::Mat> bgr_planes;
    					cv::split(imgROI, bgr_planes);

    					int histSize=256;
    					float range[]={0,256};
    					const float * histRanges={range};

    					cv::Mat b_hist,g_hist,r_hist;
    					cv::calcHist(&bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRanges, true, false);
						cv::calcHist(&bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRanges, true, false);
						cv::calcHist(&bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRanges, true, false);
    					
    					std::vector<int> R=getVector(r_hist);
    					std::vector<int> G=getVector(g_hist);
    					std::vector<int> B=getVector(b_hist);

    					int max_b=std::max_element(B.begin(), B.end())-B.begin();
    					int max_r=std::max_element(R.begin(), R.end())-R.begin();
    					int max_g=std::max_element(G.begin(), G.end())-G.begin();

    					std::cout<<":::"<<max_g<<":"<<G[max_g] << " "<<max_r<<":"<<R[max_r]<<" "<<max_b<<":"<<B[max_g]<<std::endl;

    					if(G[max_g]<R[max_r]&&G[max_g]<B[max_b])
    					cv::drawContours(img, contours, i,cv::Scalar(max_b,0,max_r),3);
    					else if(R[max_r]<G[max_g]&&R[max_r]<B[max_b])
    					cv::drawContours(img, contours,i, cv::Scalar(max_b,max_g,0),3);
    					else if(B[max_b]<R[max_r]&&B[max_b]<G[max_g])
    					cv::drawContours(img, contours,i, cv::Scalar(0,max_g,max_r),3);
    						
					}
				}
			}
		}
	}
	cv::imshow("mm", img);
	cv::waitKey(0);
	cv::Mat hsv,res_yellow,gray_yellow,thresh_yellow;
	cv::cvtColor(img, hsv, cv::COLOR_RGB2HSV);
	//yellow
	cv::inRange(hsv, cv::Scalar(25,50,50),cv::Scalar(50,255,255),res_yellow );
	cv::bitwise_and(img, img, res_yellow);
	
	cv::threshold(res_yellow,thresh_yellow  ,  10, 255,cv::THRESH_BINARY);
    cv::cvtColor(thresh_yellow, gray_yellow,cv::COLOR_BGR2GRAY);

	std::vector<std::vector<cv::Point> >contours;
	std::vector<cv::Vec4i> hierarchy;
    cv::findContours(gray_yellow, contours,hierarchy,CV_RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	
    for(int i=0;i<contours.size();i++){
    	int size=cv::contourArea(contours[i]);
    	if(size>400){
    		std::cout<<"?"<<std::endl;
    		cv::drawContours(gray_yellow, contours, i, cv::Scalar(-1,255,-1));
    	}
    }
    cv::imshow("we", gray_yellow);
    contours.clear();
    hierarchy.clear();
    cv::findContours(gray_yellow, contours,hierarchy,cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for(int i=0;i<contours.size();i++){
    	int size=cv::contourArea(contours[i]);
    	if(size>500){
    		std::cout<<"."<<std::endl;
    		cv::drawContours(thresh_yellow, contours, i, cv::Scalar(-1,255,-1));
    		cv::Rect rect=cv::boundingRect(contours[i]);
    		cv::rectangle(img_x,cv::Point( rect.x,rect.y) ,cv::Point(rect.x+rect.width,rect.y+rect.height) ,cv::Scalar(0,255,0),3);
    	}
    }

   cv::imshow("img", img_x);
   cv::waitKey(0);


}