#include<iostream>
#include "manager.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>
#include <map>
#include "realsense_config.h"
#include "cv-helpers.hpp"
#include <cmath>
#include <time.h>
#define VIDEO_TYPE (1) //1:D455
cv::Mat Depthmate,color_mat;

using namespace cv;

int main() {
    // calculate every person's (id,(up_num,down_num,average_x,average_y))
    map<int, vector<int>> personstate;
    map<int, int> classidmap;
    bool is_first = true;
    char *yolo_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/yolov5s.engine";
    char *sort_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/deepsort.engine";
    float conf_thre = 0.6;
    cv::Mat frame;
    VideoCapture capture;

    if(VIDEO_TYPE==1){
        Realsense_config();
        //pipes.start();
    }

    if(VIDEO_TYPE==0){
        frame = capture.open(4);
        if (!capture.isOpened()) {
            std::cout << "can not open cam" << std::endl;
            return -1;
        }
        capture.read(frame);
    }

    Trtyolosort yosort(yolo_engine, sort_engine);
    std::vector<DetectBox> det;
    auto start_draw_time = std::chrono::system_clock::now();
    clock_t start_draw, end_draw;
    start_draw = clock();
    int i = 0;
    while (cv::waitKey(1) != 27) {

        if (VIDEO_TYPE == 0) {
            if (i % 3 == 0) {
                //std::cout<<"origin img size:"<<frame.cols<<" "<<frame.rows<<std::endl;
                auto start = std::chrono::system_clock::now();
                yosort.TrtDetect(frame, conf_thre, det);
                auto end = std::chrono::system_clock::now();
                int delay_infer = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                std::cout << "delay_infer:" << delay_infer << "ms" << std::endl;
            }
            i++;
        }
        if(VIDEO_TYPE==1){
            auto data = pipes.wait_for_frames();
            auto color_frame = data.get_color_frame();
            auto depth_frame = data.get_depth_frame();//.apply_filter(color_map)
            color_mat = frame_to_mat(color_frame);
            Mat depth_mat(Size(640,480),
                          CV_16U,(void*)depth_frame.get_data(),Mat::AUTO_STEP);
            Depthmate = depth_mat;
            if (i % 3 == 0) {
                //std::cout<<"origin img size:"<<frame.cols<<" "<<frame.rows<<std::endl;
                auto start = std::chrono::system_clock::now();
                yosort.TrtDetect(color_mat, conf_thre, det);
                auto end = std::chrono::system_clock::now();
                int delay_infer = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                std::cout << "delay_infer:" << delay_infer << "ms" << std::endl;
            }
            i++;
        }
    }
    capture.release();
    return 0;

}