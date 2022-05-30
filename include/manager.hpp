#ifndef _MANAGER_H
#define _MANAGER_H

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "deepsort.h"
#include "logging.h"
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <ctime>
#include "mutex.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "yolov5_lib.h"
#include "deepsort.h"
#include "realsense_config.h"
#include "objectProcess.h"
#define Stride 5 //稀疏化步长
extern rs2::pipeline_profile profile;
extern cv::Mat depthMat, colorMat;
using std::vector;
using namespace cv;
//static Logger gLogger;

class Trtyolosort{
public:
	// init 
	Trtyolosort(char *yolo_engine_path,char *sort_engine_path);
	// detect and show
    int TrtDetect(cv::Mat &frame,float &conf_thresh,std::vector<DetectBox> &det,const rs2::depth_frame& aligned_depth_frame);

    int TrtDetect(cv::Mat &frame,float &conf_thresh,std::vector<DetectBox> &det);

	void showDetection(cv::Mat& img, std::vector<DetectBox>& boxes,const rs2::depth_frame& aligned_depth_frame);

    static void showDetection(cv::Mat& img, std::vector<DetectBox>& boxes);

    static float GetBoxDepth2(DetectBox box,const rs2::depth_frame& alignedDepthFrame);
    void getBoxDepthAndAngle(DetectBox &boxes,const rs2::depth_frame& alignedDepthFrame);

    static float Get_Area_Depth(DetectBox box);
    static void dealWithBox(vector<DetectBox> boxes);

private:

    my::Mutex m_mutex;
	char* yolo_engine_path_ = NULL;
	char* sort_engine_path_ = NULL;
    void *trt_engine = NULL;
    // deepsort parms
    DeepSort* DS;
    std::vector<DetectBox> t;
    //std::vector<DetectBox> det;
};
#endif  // _MANAGER_H

