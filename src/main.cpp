#include <iostream>
#include "manager.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <chrono>
#include "unistd.h"
#include <threadPool.h>
#include <RealSenseD435.h>

///define d455 depthMat and colorMat
cv::Mat depthMat, colorMat;
rs2::frame aligned_depth_frame;
std::mutex main_mutex, main2_mutex;
std::condition_variable c_v;
std::vector<DetectBox> det;

using namespace cv;


void realsense() {

    RealSenseD435 rs;
    rs.colorInit(COLOR_BGR8_640x480_30Hz);
    rs.depthInit(DEPTH_Z16_640x480_30HZ);
    rs.start();
    cout << "******realsense init*****" << endl;
    while (cv::waitKey(1) != 27) {

        std::unique_lock<std::mutex> lk(main_mutex);
        rs.updateFrame();
        rs.updateColor();
        rs.updateDepth(DEPTH_ALIGN_TO_COLOR);
        rs.get_colorImage(colorMat);
        rs.get_depth_Mat_8UC1(depthMat);
        aligned_depth_frame = rs.depth_frames;
    }

}


void yolo() {

    //char *yolo_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/float1.engine";
    char *yolo_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/yolov5s.engine";
    char *sort_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/deepsort.engine";

    ///set conf
    float conf = 0.4;


    Trtyolosort yoloSort(yolo_engine, sort_engine);

    while (cv::waitKey(1) != 27) {


        std::unique_lock<std::mutex> lk(main_mutex);
        std::unique_lock<std::mutex> lk2(main2_mutex);
        yoloSort.TrtDetect(colorMat, conf, det, aligned_depth_frame);

    }
}


void dealWithBox() {

    vector<DetectBox> validBox;

    while (true) {


        if (!det.empty()) {
            cout << "------start-------" << endl;
            std::unique_lock<std::mutex> lk2(main2_mutex);
            for (auto box: det) {
                if (!isnan(box.dis) && box.dis > 0.4 && box.dis < 10 && box.confidence > 0.6) {

                    validBox.push_back(box);
                }
            }
            lk2.unlock();
        }

        if (!validBox.empty()) {
            sort(validBox.begin(), validBox.end(),
                 [](DetectBox box1, DetectBox box2) -> bool { return box1.dis < box2.dis; });
            for (auto &box: validBox) {
                trackBox(box);
            }
        }
        cout << "------end-------" << endl;
        usleep(100000);
    }

}


int main() {


    threadPool pool(4);
    pool.submit(realsense);
    pool.submit(yolo);
    pool.submit(dealWithBox);

}