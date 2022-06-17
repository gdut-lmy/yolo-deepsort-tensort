//
// Created by lmy on 2022/5/19.
//

#ifndef YOLOSORT_OBJECTANGLE_H
#define YOLOSORT_OBJECTANGLE_H

#include "sharedMemory.h"
#include <manager.hpp>

#define pi 3.14159
const float depth_scale = 0.001;

void dealWithDistance(float &dis, int x, int y, float angle);


void getObjectAngleFromPixel(DetectBox &box);

void getObjectAngle(DetectBox &box);

float measure_distance(const cv::Mat &depth, const DetectBox &box, const cv::Size &range);

bool isValidBox(DetectBox box);

void trackBox(DetectBox box);

/// object in right picture the angle is +
void getObjectAngleFrom3D(DetectBox &box);


#endif //YOLOSORT_OBJECTANGLE_H
