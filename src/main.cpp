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
std::vector<DetectBox> det;

my::RWLock frameLock;
using namespace cv;


void getFrame(){

    ///realsense init
    Realsense_config();
    rs2_stream align_to = RS2_STREAM_COLOR;
    rs2::align align(align_to);

    while (cv::waitKey(1) != 27){

        my::WriteScopedLock frame(&frameLock);

        auto data = pipes.wait_for_frames();
        auto processed = align.process(data);
        auto aligned_color_frame = processed.get_color_frame();
        aligned_depth_frame = processed.get_depth_frame();//.apply_filter(color_map)


        ///frame to mat
        colorMat = frame_to_mat(aligned_color_frame);
        Mat depth_mat(Size(640, 480), CV_16U, (void *) aligned_depth_frame.get_data(), Mat::AUTO_STEP);
        depthMat = depth_mat;

        usleep(1000);
    }
    pipes.stop();

}

void yolo() {

    //char *yolo_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/float1.engine";
    char *yolo_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/yolov5s.engine";
    char *sort_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/deepsort.engine";

    ///set conf
    float conf = 0.4;

    Trtyolosort yoloSort(yolo_engine, sort_engine);
    auto start_draw_time = std::chrono::system_clock::now();
    while (cv::waitKey(1) != 27) {

        my::ReadScopedLock yolo(&frameLock);
        ///detect and test delay
        auto start = std::chrono::system_clock::now();
        yoloSort.TrtDetect(colorMat, conf, det, aligned_depth_frame);

        //TODO deal the box
        //yoloSort.dealWithBox(det);
        auto end = std::chrono::system_clock::now();
        int delay_infer = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "delay_infer:" << delay_infer << "ms" << std::endl;
        usleep(10000);
    }
}


void dealWithBox() {

    vector<DetectBox> validBox;

    while (cv::waitKey(1) != 27) {

        if (!det.empty()) {
            for (auto box: det) {
                if (!isnan(box.dis) || box.dis > 0.4 || box.dis < 10 || box.confidence > 0.6) {

                    validBox.push_back(box);
                    //cout << "valid box ID" << box.trackID << endl;
                }
            }
        }

        if (!validBox.empty()) {
            for (auto box: validBox) {
                trackBox(box);

                sleep(1);
            }
        }
        sleep(1);
    }
}


int main() {

    my::Thread::ptr th0(std::make_shared<my::Thread>(&getFrame, "getFrame"));
    my::Thread::ptr th1(std::make_shared<my::Thread>(&yolo, "yolo"));
    my::Thread::ptr th2(std::make_shared<my::Thread>(&dealWithBox, "box"));


    th0->join();
    th1->join();
    th2->join();


    return 0;
}