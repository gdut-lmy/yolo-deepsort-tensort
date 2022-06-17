#include <iostream>
#include "manager.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <chrono>
#include <threadPool.h>
#include <RealSenseD435.h>
#include "shared_mutex"

///define d455 depthMat and colorMat
cv::Mat depthMat, colorMat;
rs2::frame aligned_depth_frame;
std::shared_mutex Rwlock_real_yolo, Rwlock_yolo_box;
std::vector<DetectBox> det;


using namespace cv;


void realsense() {

    RealSenseD435 rs;
    rs.colorInit(COLOR_BGR8_640x480_60Hz);
    rs.depthInit(DEPTH_Z16_640x480_60HZ);
    rs.start();
    cout << "******realsense init*****\n";
    while (true) {

        std::unique_lock<std::shared_mutex> lk(Rwlock_real_yolo);
        rs.updateFrame();
        rs.updateColor();
        rs.updateDepth(DEPTH_ALIGN_TO_COLOR);
        rs.get_colorImage(colorMat);
        rs.get_depth_Mat_8UC1(depthMat);
        aligned_depth_frame = rs.depth_frames;

    }

}


void yolo() {

    ///set the path of yolo and deepSort engine
    //const char *yolo_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/float1.engine";
    const char *yolo_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/yolov5s.engine";
    const char *sort_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/deepsort.engine";

    ///set conf
    float conf = 0.6;


    Trtyolosort yoloSort(const_cast<char *>(yolo_engine), const_cast<char *>(sort_engine));

    while (true) {

        std::shared_lock<std::shared_mutex> lk(Rwlock_real_yolo);
        std::unique_lock<std::shared_mutex> lk2(Rwlock_yolo_box);
        yoloSort.TrtDetect(colorMat, conf, det, aligned_depth_frame);

    }
}


void dealWithBox() {

    vector<DetectBox> validBox;

    while (true) {
        if (!det.empty()) {
            cout << "------start-------\n";
            std::shared_lock<std::shared_mutex> lk2(Rwlock_yolo_box);
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
                //TODO send box data
                trackBox(box);
            }
        }
        cout << "------end-------\n";
        this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

}


int main() {

    threadPool pool(4);
    pool.submit(realsense);
    pool.submit(yolo);
    pool.submit(dealWithBox);

}