//
// Created by lmy on 2022/5/16.
//

#ifndef YOLOSORT_REALSENSE_CONFIG_H
#define YOLOSORT_REALSENSE_CONFIG_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>
#include <math.h>
#include <string>
#include "yolov5_lib.h"
#include "deepsort.h"
#include <cstdlib>
#include <Eigen/Dense>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <exception>
#include <opencv2/opencv.hpp>   // Include OpenCV API
int Realsense_config();
cv::Mat align_Depth2Color(cv::Mat depth, const cv::Mat &color, rs2::pipeline_profile profile);
float get_depth_scale(const rs2::device& dev);
float measure_distance(cv::Mat depth,DetectBox box,cv::Size range,rs2::pipeline_profile profile);
int Get_referance();
float getDistanceInMeters(DetectBox box,rs2::depth_frame aligned_depth_frame,rs2::video_frame aligned_color_frame);
extern rs2::pipeline pipes;
extern rs2::pipeline_profile profile;


#endif //YOLOSORT_REALSENSE_CONFIG_H
