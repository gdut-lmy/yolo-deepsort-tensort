//
// Created by lmy on 2022/5/19.
//
#include <thread>
#include "objectProcess.h"


//
void getObjectAngle(DetectBox &box) {

    if (isnan(box.dis) || box.dis < 0.4 || box.dis > 8) {
        getObjectAngleFromPixel(box);
    } else {
        getObjectAngleFrom3D(box);
    }

}


void getObjectAngleFromPixel(DetectBox &box) {
    if (box.pixel_x > 360) {
        box.angle = static_cast<float >(640 - box.pixel_x) / 320 * 40;
    } else if (box.pixel_x < 280) {
        box.angle = -static_cast<float >(320 - box.pixel_x) / 320 * 40;
    } else {
        box.angle = 0;
    }

}

void getObjectAngleFrom3D(DetectBox &box) {
    float tmp = atan(box.pdc[0] / box.pdc[2]);
    box.angle = tmp * 180 / pi;
    //cout << "The Object angle:" << box.angle << endl;
}

void dealWithDistance(float &dis, int x, int y, float angle) {
    if (x < 280 || x > 360) {
        dis = 0;//adjust angle dont move
    } else if (dis > 8) {
        dis = 4;
    } else if (isnan(dis) || dis < 0.4) {
        dis = 0;
    }
}


bool isValidBox(DetectBox box) {

    if (box.dis < 0.4)
        return false;
    else
        return true;

}

void trackBox(DetectBox box) {


    cout << "trackBox.dis ----------------------:" << box.dis << endl;
    cout << "trackBox ----------------------:" << box.trackID << endl;

    this_thread::sleep_for(std::chrono::milliseconds(1000));

}


float measure_distance(const cv::Mat &depth, const DetectBox &box, const cv::Size &range)//声明profile
{
    //定义图像中心点
    cv::Point center(box.pixel_x, box.pixel_y);
    //定义计算距离的范围
    cv::Rect RectRange(center.x - range.width / 2, center.y - range.height / 2, range.width, range.height);
    //std::cout<<box.x1<<" " <<box.y1<<" "<<box.x2 <<" "<<box.y2 <<std::endl;
    //画出范围
    float distance_sum = 0;
    int effective_pixel = 0;

    for (int y = RectRange.y; y < RectRange.y + RectRange.height; y++) {
        for (int x = RectRange.x; x < RectRange.x + RectRange.width; x++) {
            //不是0就有位置信息
            if (depth.at<uint16_t>(y, x))//出现位置信息
            {
                distance_sum += depth_scale * depth.at<uint16_t>(y, x);
                effective_pixel++;
            }
        }
    }
    //std::cout << "有效像素点：" << effective_pixel <<std::endl;//输出数据

    float effective_distance = distance_sum / effective_pixel;
    //std::cout << "目标距离：" << effective_distance << "m" << std::endl;

    return effective_distance;
}