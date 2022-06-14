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
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "yolov5_lib.h"
#include "deepsort.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>
#include <cmath>
#include <string>
#include "yolov5_lib.h"
#include "deepsort.h"
#include <cstdlib>
#include <Eigen/Dense>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <exception>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include "objectProcess.h"

#define Stride 5 //稀疏化步长
extern cv::Mat depthMat, coloreMat;



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
    void setBoxAngleAndDis(cv::Mat& img, std::vector<DetectBox>& boxes,const rs2::depth_frame& aligned_depth_frame);

    static void showDetection(cv::Mat& img, std::vector<DetectBox>& boxes);

    static float GetBoxDepth2(DetectBox box,const rs2::depth_frame& alignedDepthFrame);
    static void getBoxDepthAndAngle(DetectBox &boxes,const rs2::depth_frame& alignedDepthFrame);

    static float Get_Area_Depth(DetectBox box);
    static void dealWithBox(vector<DetectBox> boxes);

private:

	char* yolo_engine_path_ = NULL;
	char* sort_engine_path_ = NULL;
    void *trt_engine = NULL;
    // deepsort parms
    DeepSort* DS;
    std::vector<DetectBox> t;
    std::mutex m_mutex;
    //std::vector<DetectBox> det;
};
#endif  // _MANAGER_H

