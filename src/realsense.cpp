#include "realsense.h"
// #include <librealsense2/rs.hpp>
// #include <opencv2/opencv.hpp>


rs2::pipeline realsense::pipe;
cv::Mat realsense::img;
cv::Mat realsense::depth_img;
pthread_mutex_t realsense::Mutex;//互斥锁
int realsense::height,realsense::width;
static rs2::align align_to_depth(RS2_STREAM_DEPTH);
bool realsense::get(cv::Mat &_img, cv::Mat &_depth){
    if(img.empty()||depth_img.empty())return false;
    pthread_mutex_lock(&Mutex);
    img.copyTo(_img);
    depth_img.copyTo(_depth);
    pthread_mutex_unlock(&Mutex);
    return true;
}

int realsense::creatthreadread(){
    flag=true;
    int ret=pthread_create(&thread,NULL,read_func,&flag);
    if(ret==-1){
        printf("error creat read realsense thread\n");
    }
    return ret;
}
void *realsense::read_func(void *param){
    bool *run=(bool*)param;
    while((*run)){

        try{
            rs2::frameset data=pipe.wait_for_frames();

            data=align_to_depth.process(data);

            rs2::frame color=data.get_color_frame();
            rs2::frame depth=data.get_depth_frame();

            cv::Mat img_tmp(cv::Size(width,height),CV_8UC3,(void*)color.get_data(),cv::Mat::AUTO_STEP);
            cv::Mat depth_tmp(cv::Size(width,height),CV_8UC1,(void*)depth.get_data(),cv::Mat::AUTO_STEP);

            pthread_mutex_lock(&Mutex);
            img_tmp.copyTo(img);
            depth_tmp.copyTo(depth_img);
            pthread_mutex_unlock(&Mutex);

        }
        catch (const rs2::error & e)
        {
            std::cerr << "RealSense color error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
    pthread_exit(0);
}
bool realsense::init(int _height,int _width){
    if(height!=0&&width!=0)return false;
    Mutex = PTHREAD_MUTEX_INITIALIZER;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width,height, RS2_FORMAT_BGR8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, width,height, RS2_FORMAT_Z16, 60);

    auto profile=pipe.start(cfg);
    auto sensor=profile.get_device().first<rs2::depth_sensor>();

    sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);

    spat.set_option(RS2_OPTION_HOLES_FILL, 5);
    
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);

    if(creatthreadread()==-1)return false;
    return true;
}
realsense::realsense(int _height,int _width){
    // if(height!=0&&width!=0)return *this;
    height=_height;
    width=_width;

    // object=this;
}