//
// Created by lmy on 2022/5/19.
//
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
    cout << "The Object angle:" << box.angle << endl;
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


    cout << "trackBox ----------------------:" << box.trackID << endl;
}