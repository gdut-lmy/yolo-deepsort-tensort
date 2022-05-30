#include <iostream>
#include "manager.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <chrono>
#include "realsense_config.h"
#include "cv-helpers.hpp"
#include <ctime>
#include "unistd.h"
#include "thread.h"
#include <memory>

///define d455 depthMat and colorMat
cv::Mat depthMat, colorMat;
using namespace cv;

void yolo() {

    //char *yolo_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/float1.engine";
    char *yolo_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/yolov5s.engine";
    char *sort_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/deepsort.engine";

    ///set conf
    float conf = 0.4;


    ///realsense init
    Realsense_config();
    rs2_stream align_to = RS2_STREAM_COLOR;
    rs2::align align(align_to);
    rs2::colorizer c;

    Trtyolosort yoloSort(yolo_engine, sort_engine);
    std::vector<DetectBox> det;

    auto start_draw_time = std::chrono::system_clock::now();
    clock_t start_draw, end_draw;
    start_draw = clock();
    while (cv::waitKey(1) != 27) {

        ///get realsense frames
        auto data = pipes.wait_for_frames();
        auto processed = align.process(data);
        auto aligned_color_frame = processed.get_color_frame();
        auto aligned_depth_frame = processed.get_depth_frame();//.apply_filter(color_map)

        ///frame to mat
        colorMat = frame_to_mat(aligned_color_frame);
        Mat depth_mat(Size(640, 480), CV_16U, (void *) aligned_depth_frame.get_data(), Mat::AUTO_STEP);
        depthMat = depth_mat;

        ///detect and test delay
        auto start = std::chrono::system_clock::now();
        yoloSort.TrtDetect(colorMat, conf, det, aligned_depth_frame);
        //TODO deal the box
        //yoloSort.dealWithBox(det);
        auto end = std::chrono::system_clock::now();
        int delay_infer = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "delay_infer:" << delay_infer << "ms" << std::endl;

        usleep(1000);
    }
}


int main() {



    my::Thread::ptr th(std::make_shared<my::Thread>(&yolo,"yolo"));

    th->join();




    return 0;
}