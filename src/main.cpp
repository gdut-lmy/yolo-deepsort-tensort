#include <iostream>
#include "manager.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <chrono>
#include <threadPool.h>
#include <RealSenseD435.h>
#include <algorithm>
#include <queue>
#include "shared_mutex"
#include "sharedMemory.h"

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

struct cmp {
    bool operator()(DetectBox box1, DetectBox box2) {
        return box1.dis > box2.dis;
    }
};


void dealWithBox() {

    priority_queue<DetectBox, vector<DetectBox>, cmp> vB;

    Sem sem1(0, "nihao");
    Sem sem2(1, "nihao2");


    sharedMemory m_shm(1234, 1024);

    m_shm.sharedMemoryInit(nullptr, 0);


    while (true) {
        if (!det.empty()) {
            cout << "------start-------\n";
            std::shared_lock<std::shared_mutex> lk2(Rwlock_yolo_box);
            for (auto box: det) {
                if (!isnan(box.dis) && box.dis > 0.4 && box.dis < 10 && box.confidence > 0.6) {

                    vB.push(box);
                }
            }
            lk2.unlock();
        }

        if (!vB.empty()) {

            string s1;
            s1 = to_string(vB.top().dis) + ',' + to_string(vB.top().angle);
            cout << s1 << endl;
            sem2.wait();//-1
            m_shm.writeData(s1);
            sem1.post();
            s1.clear();
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