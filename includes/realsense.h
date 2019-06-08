#ifndef _REALSENSE_H
#define _REALSENSE_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class realsense
{
private:
     rs2::spatial_filter spat;
     rs2::decimation_filter dec;
     static rs2::pipeline pipe;
     static cv::Mat img;
     static cv::Mat depth_img;
     static pthread_mutex_t Mutex;//互斥锁
     static int height,width;
     pthread_t thread;
     bool flag; 
     // realsense * object;
public:
    realsense(int _height=640,int _width=480);
    ~realsense(){
        flag=false;
    }
    bool init(int _height=640,int _width=480);
    bool get(cv::Mat &img,cv::Mat &depth);
private:
    static void *read_func(void *param);
    int creatthreadread();
};

#endif