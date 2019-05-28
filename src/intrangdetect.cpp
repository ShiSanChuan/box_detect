#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
// using namespace cv;
cv::Mat bby;
int Contrast=186;
int Light=21;
int maxCosinelimte=8;
double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2) / std::sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
double GetTanline(cv::Vec4i &line){
	double x_dis=line[2]-line[0];
	if(x_dis==0.0)return 10e9;
	return (line[3]-line[1])/x_dis;
}
bool isParallel(cv::Vec4i &line1,cv::Vec4i &line2){
	double angle0=GetTanline(line1);
	double angle1=GetTanline(line2);
	return (fabs(angle0-angle1)<(double)(1.547/3)||fabs(angle0-angle1)>(double)(3.14+1.547/3))?1:0;
}
double linedistance(cv::Vec4i &line){
	double x_dis=line[2]-line[0];
	double y_dis=line[3]-line[1];
	return std::sqrt(x_dis*x_dis+y_dis*y_dis);
}
double Paralleldistance(cv::Vec4i &line1,cv::Vec4i &line2){
	cv::Point2f midpoint((double)(line2[2]-line2[0])/2.0,(double)(line2[3]-line2[1])/2.0);
	double a=GetTanline(line1);
	double b=(double)line1[1]-line1[0]*a;
	return fabs(a*midpoint.x-midpoint.y+b)/std::sqrt(a*a+1);
}
double pointdistance(cv::Point2f &P1,cv::Point2f &P2){
	double x_dis=P2.x-P1.x;
	double y_dis=P2.y-P1.y;
	return std::sqrt(x_dis*x_dis+y_dis*y_dis);
}
bool isConnect(cv::Vec4i &line1,cv::Vec4i &line2){
	std::vector<cv::Point2f> points;
	points.push_back(cv::Point2f(line1[0],line1[1]));
	points.push_back(cv::Point2f(line1[2],line1[3]));
	points.push_back(cv::Point2f(line2[0],line2[1]));
	points.push_back(cv::Point2f(line2[2],line2[3]));
	for(int i=0;i<4;i++)
		for(int j=i+1;j<4;j++){
			double x_dis=points[i].x-points[j].x;
			double y_dis=points[i].y-points[j].y;
			if(x_dis*x_dis+y_dis*y_dis<3) return 1;
		}
	return 0;
}
int hough_line(cv::Mat src)
{  
    cv::Mat srcImage = src;//imread("1.jpg");  //工程目录下应该有一张名为1.jpg的素材图
    cv::Mat midImage,dstImage;//临时变量和目标图的定义
 
    cv::Canny(srcImage, midImage, 50, 200, 3);//进行一此canny边缘检测
    cv::cvtColor(midImage,dstImage, CV_GRAY2BGR);//转化边缘检测后的图为灰度图
 
    std::vector<cv::Vec4i> lines;//定义一个矢量结构lines用于存放得到的线段矢量集合
    cv::HoughLinesP(midImage, lines, 1, CV_PI/180, 100, 100, 10 );
 
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        line( dstImage, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(186,88,255), 1, CV_AA);
    }

    cv::imshow("demo", dstImage);  
    return 0;  
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
void setCL(int ,void*){

}


int find_box(cv::Mat src,cv::Mat &CM,cv::Mat &D){
	cv::Mat srcImage=src,midImage=cv::Mat::zeros(src.size(),CV_8UC1),tempImage=cv::Mat::zeros(src.size(),CV_8UC3),aay;
	cv::Mat kernel;
	AdjustContrastAndLight(srcImage,Contrast/100,Light-100);
	//滤波模糊
	cv::GaussianBlur(srcImage, srcImage, cv::Size(3,3), 3,3);
	cv::cvtColor(srcImage, srcImage, CV_BGR2GRAY);
	//锐化
	cv::Mat grad_x,grad_y;
	cv::Sobel(srcImage,grad_x,CV_16S,1, 0,3,1,0);
	cv::Sobel(srcImage,grad_y,CV_16S,0, 1,3,1,0);
	convertScaleAbs(grad_x,grad_x);
    convertScaleAbs(grad_y,grad_y);
    addWeighted(grad_x, 0.5, grad_y, 0.5, 0, srcImage);

    kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    morphologyEx(srcImage,srcImage,cv::MORPH_CLOSE,kernel); //排除近似平行线

	cv::Canny(srcImage, midImage, 10, 250, 3);//50 200
	cv::cvtColor(midImage,aay, CV_GRAY2BGR);
	kernel= getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));
    morphologyEx(aay,aay,cv::MORPH_TOPHAT,kernel); //排除近似平行线

	std::vector<std::vector<cv::Point> >contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Canny(aay, tempImage, 50, 100, 3);  

	cv::cvtColor(midImage,aay, CV_GRAY2BGR);
	kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
    morphologyEx(aay,aay,cv::MORPH_TOPHAT,kernel); //排除近似平行线
    cv::cvtColor(aay,tempImage,CV_BGR2GRAY);

    cv::findContours(tempImage, contours,hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE);

    for(int i=0;i<contours.size();i++){
    	std::vector<cv::Point2f> polygon;
    	approxPolyDP(contours[i], polygon, arcLength(contours[i], 1)*0.02, 1);//arcLength(contours[i], 1)*0.02
    	double area = fabs(contourArea(polygon));
    	if(polygon.size() == 4){
    				drawContours(src,contours,i,cv::Scalar(0,255,0),1,8,hierarchy);
    	}
		if (isContourConvex(polygon) && 
			polygon.size() == 4  &&
			area>1800)
		{
			double maxCosine = 0;
			for (int j = 2; j < 5; j++)
			{
				double cosine = 
					fabs(angle(polygon[j % 4], polygon[j - 2], polygon[j - 1]));
				maxCosine = MAX(maxCosine, cosine);
			}
			if (maxCosine <maxCosinelimte/10.0)
			{
				// printf("%f\t%f\n", maxCosine,maxCosinelimte/10.0);
				std::vector<cv::Point2f> image_points;
				std::vector<cv::Point3f> qurapoints;
				
				for(int i=0;i<4;i++){
					cv::line(src,polygon[i] , polygon[(i+1)%4],  cv::Scalar(0,0,255),2);
				}
				if(pointdistance(polygon[0],polygon[1])>pointdistance(polygon[0],polygon[3])){
					qurapoints.push_back(cv::Point3f(0.,0.,0.));
					qurapoints.push_back(cv::Point3f(40.,0.,0.));
					qurapoints.push_back(cv::Point3f(40.,20.,0.));
					qurapoints.push_back(cv::Point3f(0.,20.,0.));
				}else{
					qurapoints.push_back(cv::Point3f(0.,0.,0.));
					qurapoints.push_back(cv::Point3f(20.,0.,0.));
					qurapoints.push_back(cv::Point3f(20.,40.,0.));
					qurapoints.push_back(cv::Point3f(0.,40.,0.));
				}
				cv::Mat rotation_vector;
				cv::Mat translation_vector;
				cv::solvePnP(qurapoints, polygon, CM, D, rotation_vector, translation_vector);
				std::vector<cv::Point3d> end_point3D;
		    	std::vector<cv::Point2d> end_point2D;
		    	// int midlenth=linedistance()
		    	double high= -20;
		    	// if(high>20)high=20;
		    	if(pointdistance(polygon[0],polygon[1])>pointdistance(polygon[0],polygon[3])){
			    	end_point3D.push_back(cv::Point3d(0,0,high));
			    	end_point3D.push_back(cv::Point3d(40,0,high));
			    	end_point3D.push_back(cv::Point3d(40,20,high));
			    	end_point3D.push_back(cv::Point3d(0,20,high));
		    	}else{
		    		end_point3D.push_back(cv::Point3d(0,0,high));
			    	end_point3D.push_back(cv::Point3d(20,0,high));
			    	end_point3D.push_back(cv::Point3d(20,40,high));
			    	end_point3D.push_back(cv::Point3d(0,40,high));
		    	}
			 	projectPoints(end_point3D, rotation_vector, translation_vector, CM, D, end_point2D);
			    
			    cv::line(src,polygon[0], end_point2D[0], cv::Scalar(255,0,0), 2);
			    cv::line(src,polygon[1], end_point2D[1], cv::Scalar(255,0,0), 2);
				cv::line(src,polygon[2], end_point2D[2], cv::Scalar(255,0,0), 2);
				cv::line(src,polygon[3], end_point2D[3], cv::Scalar(255,0,0), 2);

				std::vector<cv::Point2f> srcpoint,aimpoint;

				for(int i=0;i<4;i++){
					aimpoint.push_back(cv::Point2f((double)end_point2D[i].x,(double)end_point2D[i].y));
					
					cv::line(src,end_point2D[i],end_point2D[(i+1)%4],cv::Scalar(0,0,255),2);
				}

				//添加一张图片
				
				// srcpoint.push_back(cv::Point2f(0,0));
				// srcpoint.push_back(cv::Point2f(504.,0));
				// srcpoint.push_back(cv::Point2f(504.,504.));
				// srcpoint.push_back(cv::Point2f(0,504.));

				// cv::Mat picbby;

				// cv::Mat Trans=cv::getPerspectiveTransform(srcpoint, aimpoint);
				// cv::warpPerspective(bby, picbby,Trans,cv::Size(640,480));
				// cv::perspectiveTransform(bby, picbby, Trans);
				//cv::add(src,picbby ,src);
				// cv::imshow("image2",picbby);
			}
		}
    }
	cv::imshow("Image2", aay);
	cv::imshow("Image3", tempImage);
	cv::imshow("fix", srcImage);
	// cv::imshow("Image", midImage);
	cv::imshow("demo", src);
	//cv::imshow("image",bby);
	return 0;
}
void fiximage(cv::Mat &img){
	float cameraMatrix[3][3]={445.5745856062967, 0, 321.8550668645568,
 0, 446.3484431899639, 243.3262186174895,
 0, 0, 1};
 	float distCoeffs[5]={-0.4134480994096632, 0.1739702455953338, 0.004211590264998173, -0.00465287334325685, 0};
	cv::Mat CM=cv::Mat(3,3,CV_32FC1,cameraMatrix);
	cv::Mat D=cv::Mat(1,5,CV_32FC1,distCoeffs);
	cv::undistort(img, img,CM,D);
}
void UseChess(cv::Mat &img,cv::Mat &CM,cv::Mat &D){
	std::vector<cv::Point2f> image_points;
	std::vector<cv::Point2f> rectpoints;
	std::vector<cv::Point3f> qurapoints;
	bool ok=cv::findChessboardCorners(img,cv::Size(9,6), image_points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
	if(ok){
		qurapoints.push_back(cv::Point3f(0.,0.,0.));
		qurapoints.push_back(cv::Point3f(200.,0.,0.));
		qurapoints.push_back(cv::Point3f(200.,125.,0.));
		qurapoints.push_back(cv::Point3f(0.,125.,0.));
		
		rectpoints.push_back(image_points[0]);
		rectpoints.push_back(image_points[8]);
		rectpoints.push_back(image_points[53]);
		rectpoints.push_back(image_points[45]);
		for(int i=0;i<4;i++){
			cv::line(img,rectpoints[i] , rectpoints[(i+1)%4],  cv::Scalar(0,0,255),2);
		}
		cv::Mat rotation_vector;
		cv::Mat translation_vector;
	
		cv::solvePnP(qurapoints, rectpoints, CM, D, rotation_vector, translation_vector);
 	
		std::vector<cv::Point3d> end_point3D;
    	std::vector<cv::Point2d> end_point2D;
    	end_point3D.push_back(cv::Point3d(0,0,-200.0));
    	end_point3D.push_back(cv::Point3d(200,0,-200.0));
    	end_point3D.push_back(cv::Point3d(200,125,-200.0));
    	end_point3D.push_back(cv::Point3d(0,125,-200.0));

	 	projectPoints(end_point3D, rotation_vector, translation_vector, CM, D, end_point2D);
	    
	    cv::line(img,rectpoints[0], end_point2D[0], cv::Scalar(255,0,0), 2);
	    cv::line(img,rectpoints[1], end_point2D[1], cv::Scalar(255,0,0), 2);
		cv::line(img,rectpoints[2], end_point2D[2], cv::Scalar(255,0,0), 2);
		cv::line(img,rectpoints[3], end_point2D[3], cv::Scalar(255,0,0), 2);

		for(int i=0;i<4;i++){
			cv::line(img,end_point2D[i],end_point2D[(i+1)%4],cv::Scalar(0,0,255),2);
		}
	}

    cv::imshow("demo", img);
}


int main(int argc, char const *argv[])
{
	//读取图片
	// cv::Mat img=cv::imread("picture/20181020_8.jpg");
	//矫正畸变
	// fiximage(img);
	//resize　一下
	//
	float cameraMatrix[3][3]={445.5745856062967, 0, 321.8550668645568,
 0, 446.3484431899639, 243.3262186174895,
 0, 0, 1};
 	float distCoeffs[5]={-0.4134480994096632, 0.1739702455953338, 0.004211590264998173, -0.00465287334325685, 0};
	cv::Mat CM=cv::Mat(3,3,CV_32FC1,cameraMatrix);
	cv::Mat D=cv::Mat(1,5,CV_32FC1,distCoeffs);
	// std::cout<<img.cols<<"\t"<<img.rows<<std::endl;
	// cv::resize(img, img, cv::Size(img.cols/4,img.rows/4));
	// cv::pyrMeanShiftFiltering(img, img,25 , 10);//去除背景复杂线条
	//滤波
	// cv::GaussianBlur(img, img, cv::Size(3,3), 3,3);
	//霍夫检测
	// hough_line(img);
	// drawContours
	// find_box(img);
	cv::VideoCapture cap;
	cap.open(1);
	cv::Mat frame,fiximage;
	bby=cv::imread("huaji.jpeg");
	std::cout<<bby.cols<<"\t"<<bby.rows<<std::endl;
	// cv::resize(bby, bby, cv::Size(50,50));
	// frame=cv::imread("picture/20181020_8.jpg");
	namedWindow("demo",cv::WINDOW_AUTOSIZE);
	cv::createTrackbar("Contrast", "demo", &Contrast,200,setCL);
	cv::createTrackbar("Light", "demo", &Light, 200,setCL);
	cv::createTrackbar("cossinlimet", "demo", &maxCosinelimte, 10,setCL);
	while(char(cv::waitKey(1))!='q'){
		cap>>frame;
		if(frame.empty()){
			std::cout<<"image error!"<<std::endl;
			break;
		}
		fiximage= frame.clone(); 
		cv::undistort(frame, fiximage,CM,D);
		find_box(fiximage,CM,D);
		// hough_line(fiximage);
		// cv::GaussianBlur(frame, frame, cv::Size(3,3), 3,3);
		// UseChess(frame,CM,D);
		// 
	}
	cv::waitKey(0);
	cap.release();
	return 0;
}
