//
// Created by lmy on 2022/5/19.
//

#ifndef YOLOSORT_OBJECTANGLE_H
#define YOLOSORT_OBJECTANGLE_H

#include <realsense_config.h>
#include <manager.hpp>
#define pi 3.14159

void dealWithDistance(float &dis,int x,int y,float angle);


void getObjectAngleFromPixel(DetectBox &box);
void getObjectAngle(DetectBox &box);


/// object in right picture the angle is +
void getObjectAngleFrom3D(DetectBox &box);


#endif //YOLOSORT_OBJECTANGLE_H
