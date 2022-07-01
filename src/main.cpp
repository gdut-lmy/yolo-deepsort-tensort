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
std::vector<DetectBox> resultBox;


using namespace cv;


void realsense() {

    RealSenseD435 rs;
    rs.colorInit(COLOR_BGR8_640x480_30Hz);
    rs.depthInit(DEPTH_Z16_640x480_30HZ);
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
    const char *yolo_engine = "/home/haique/project/yolo-deepsort-tensort/resource/jetson_yolov5.engine";
    //const char *yolo_engine = "/home/haique/project/yolo-deepsort-tensort/resource/jetson_float.engine";
    const char *sort_engine = "/home/haique/project/yolo-deepsort-tensort/resource/jetson_sort.engine";

    ///set conf
    float conf = 0.6;


    Trtyolosort yoloSort(const_cast<char *>(yolo_engine), const_cast<char *>(sort_engine));

    while (true) {

        std::shared_lock<std::shared_mutex> lk(Rwlock_real_yolo);
        std::unique_lock<std::shared_mutex> lk2(Rwlock_yolo_box);
        yoloSort.TrtDetect(colorMat, conf, resultBox, aligned_depth_frame);

    }
}

struct cmp {
    bool operator()(DetectBox box1, DetectBox box2) {
        return box1.dis > box2.dis;
    }
};


void dealWithBox() {

    priority_queue<DetectBox, vector<DetectBox>, cmp> validBox;


    sharedMemory m_shm(0, "nihao", 1, "nihao2", 1234, 1024);

    m_shm.sharedMemoryInit(nullptr, 0);


    while (true) {

        if (!resultBox.empty()) {

            std::shared_lock<std::shared_mutex> lk2(Rwlock_yolo_box);
            for (auto box: resultBox) {
                if (!isnan(box.dis) && !isnan(box.angle) && box.dis > 0.4 && box.dis < 10 && box.confidence > 0.6) {
                    validBox.emplace(box);
                }
            }
            lk2.unlock();
        }
        if (!validBox.empty()) {

            string sendData;
            sendData = to_string(validBox.top().dis) + ',' + to_string(validBox.top().angle);

            while (!validBox.empty())
                validBox.pop();

            std::cout << sendData << std::endl;
            m_shm.writeData(sendData);

            sendData.clear();
        }

        this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

}

void videoTest() {

    char *yolo_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/floating.engine";
    char *sort_engine = "/home/lmy/project/yolov5-deepsort-tensorrt/resource/deepsort.engine";
    float conf_thre = 0.4;
    Trtyolosort yosort(yolo_engine, sort_engine);
    VideoCapture capture("/home/lmy/download/Video_Sequence_001_050/Sequence_49.avi");
    cv::Mat frame;
    if (!capture.isOpened()) {
        std::cout << "can not open" << std::endl;
    }

    std::vector<DetectBox> det;
    auto start_draw_time = std::chrono::system_clock::now();

    while (waitKey(1) != 27) {

        capture.read(frame);
        auto start = std::chrono::system_clock::now();
        yosort.TrtDetect(frame, conf_thre, det);
        auto end = std::chrono::system_clock::now();
        int delay_infer = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "delay_infer:" << delay_infer << "ms" << std::endl;
        usleep(20000);

    }
    capture.release();
}

int main() {

    threadPool pool(4);
    pool.submit(realsense);
    pool.submit(yolo);
    pool.submit(dealWithBox);
    //pool.sumbit(videoTest);


}
