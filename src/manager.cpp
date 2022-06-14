#include "manager.hpp"

#include <utility>

using std::vector;
using namespace cv;
static Logger gLogger;

Trtyolosort::Trtyolosort(char *yolo_engine_path, char *sort_engine_path) {
    sort_engine_path_ = sort_engine_path;
    yolo_engine_path_ = yolo_engine_path;
    trt_engine = yolov5_trt_create(yolo_engine_path_);
    printf("create yolov5-trt , instance = %p\n", trt_engine);
    DS = new DeepSort(sort_engine_path_, 128, 256, 0, &gLogger);

}

void
Trtyolosort::showDetection(cv::Mat &img, std::vector<DetectBox> &boxes, const rs2::depth_frame &aligned_depth_frame) {

    cv::Mat temp = img.clone();
    //float *res;
    for (auto &box: boxes) {

        getBoxDepthAndAngle(box, aligned_depth_frame);
        cv::Point lt(box.x1, box.y1);
        cv::Point br(box.x2, box.y2);
        //cout << "Track ID" << box.trackID << endl;
        //float dis= getDistanceInMeters(box,aligned_depth_frame);
        //float dis= GetBoxDepth2(box,aligned_depth_frame);
        cv::rectangle(temp, lt, br, cv::Scalar(255, 0, 0), 1);
        //std::string lbl = cv::format("ID:%d_C:%d_CONF:%.2f", (int)box.trackID, (int)box.classID, box.confidence);
        //std::string lbl = cv::format("ID:%d_CONF:%.2fDis:%.3f", (int)box.trackID, box.confidence,dis);
        std::string lbl = cv::format("ID:%dA:%.2fD:%.2f", (int) box.trackID, box.angle, box.dis);
        //std::string lbl = cv::format("ID:%d_x:%f_y:%f",(int)box.trackID,(box.x1+box.x2)/2,(box.y1+box.y2)/2);
        cv::putText(temp, lbl, lt, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
    }

    cv::imshow("img", temp);
    cv::waitKey(1);

}


void Trtyolosort::showDetection(cv::Mat &img, std::vector<DetectBox> &boxes) {
    cv::Mat temp = img.clone();
    for (auto box: boxes) {
        cv::Point lt(box.x1, box.y1);
        cv::Point br(box.x2, box.y2);
        cout << "Track ID" << box.trackID << endl;
        cv::rectangle(temp, lt, br, cv::Scalar(255, 0, 0), 1);
        std::string lbl = cv::format("ID:%d_C:%d_CONF:%.2f", (int) box.trackID, (int) box.classID, box.confidence);
        //std::string lbl = cv::format("ID:%d_CONF:%.2fDis:%.3f", (int)box.trackID, box.confidence,dis);
        //std::string lbl = cv::format("ID:%d_x:%f_y:%f",(int)box.trackID,(box.x1+box.x2)/2,(box.y1+box.y2)/2);
        cv::putText(temp, lbl, lt, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));
    }
    cv::imshow("img", temp);
    cv::waitKey(1);
}


float Trtyolosort::Get_Area_Depth(DetectBox box) {

    cv::Rect Box = cv::Rect(box.x1, box.y1, box.x2 - box.x1, box.y2 - box.y1);
    std::array<int, Stride * Stride> Arr_Box{};
    int result;
    for (int i = Box.y; i < Box.y + Box.height; ++i)
        for (int j = Box.x; j < Box.x + Box.width; ++j) {
            if (depthMat.at<uint16_t>(i, j) > 6000 || depthMat.at<uint16_t>(i, j) < 200)//D435有效探测距离有限 0.2M-6M
                Arr_Box.at((i - Box.y) * Stride + (j - Box.x)) = 0;
            else
                Arr_Box.at((i - Box.y) * Stride + (j - Box.x)) = depthMat.at<uint16_t>(i, j);
        }
    sort(Arr_Box.begin(), Arr_Box.end());
    for (auto i: Arr_Box) {    //最小池化
        if (i > 200) {
            result = i;
            break;
        }
    }
    return result;
}


float Trtyolosort::GetBoxDepth2(DetectBox box, const rs2::depth_frame &alignedDepthFrame) {
    //Mat result = align_Depth2Color(depthMat,colorMat,profile);//调用对齐函数
    float dis = measure_distance(depthMat, box, cv::Size(5, 5));
    float x = (box.x1 + box.x2) / 2, y = (box.y1 + box.y2) / 2;
    float pd_uv[2];
    float angle;
    pd_uv[0] = x, pd_uv[1] = y;
    float Pdc3[3];
    const auto intrinDepth = alignedDepthFrame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    //float  pixel_distance_in_meters = alignedDepthFrame.get_distance((int)x,(int)y);
    rs2_deproject_pixel_to_point(Pdc3, &intrinDepth, pd_uv, dis);
    getObjectAngleFrom3D(box);
    cout << "X Y Z : " << Pdc3[0] << " " << Pdc3[1] << " " << Pdc3[2] << endl;
    //cout<<"O DIS : "<<sqrt(pow(Pdc3[0],2)+pow(Pdc3[1],2) +pow(Pdc3[2],2))<< endl;
    cout << "Dis : " << dis << endl;
    return dis;
}


int Trtyolosort::TrtDetect(cv::Mat &frame, float &conf_thresh, std::vector<DetectBox> &det,
                           const rs2::depth_frame &aligned_depth_frame) {


    // yolo detect
    std::unique_lock<std::mutex> lk(m_mutex);
    yolov5_trt_detect(trt_engine, frame, conf_thresh, det);
    DS->sort(frame, det);
    //setBoxAngleAndDis(frame,det,aligned_depth_frame);
    showDetection(frame, det, aligned_depth_frame);
    return 1;

}

//
int Trtyolosort::TrtDetect(cv::Mat &frame, float &conf_thresh, std::vector<DetectBox> &det) {
    // yolo detect
    auto ret = yolov5_trt_detect(trt_engine, frame, conf_thresh, det);
    DS->sort(frame, det);
    showDetection(frame, det);
    return 1;
}

void Trtyolosort::getBoxDepthAndAngle(DetectBox &box, const rs2::depth_frame &alignedDepthFrame) {

    float Pdc3[3];
    float pd_uv[2];

    box.dis = measure_distance(depthMat, box, cv::Size((int) (box.x2 - box.x1) / 3, (int) (box.y2 - box.y1) / 3));

    pd_uv[0] = box.pixel_x, pd_uv[1] = box.pixel_y;
    const auto intrinDepth = alignedDepthFrame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    //float  pixel_distance_in_meters = alignedDepthFrame.get_distance((int)x,(int)y);
    rs2_deproject_pixel_to_point(Pdc3, &intrinDepth, pd_uv, box.dis);

    box.pdc = Pdc3;
    getObjectAngle(box);


    // cout << "X Y Z : " << Pdc3[0] << " " << Pdc3[1] << " " << Pdc3[2] << endl;
    //cout<<"O DIS : "<<sqrt(pow(Pdc3[0],2)+pow(Pdc3[1],2) +pow(Pdc3[2],2))<< endl;
    // cout << "Dis : " << box.dis << endl;
}


void Trtyolosort::dealWithBox(vector<DetectBox> boxes) {
    if (boxes.empty()) {
        cout << "no object found" << endl;
    } else {
        for (auto box: boxes) {
            if (box.confidence > 0.6) {
                getObjectAngle(box);
            }

        }

    }

}

void
Trtyolosort::setBoxAngleAndDis(cv::Mat &img, vector<DetectBox> &boxes, const rs2::depth_frame &aligned_depth_frame) {

    for (auto box: boxes) {
        getBoxDepthAndAngle(box, aligned_depth_frame);
        /*  cv::Point lt(box.x1, box.y1);
          cv::Point br(box.x2, box.y2);
          //cout << "Track ID" << box.trackID << endl;
          //float dis= getDistanceInMeters(box,aligned_depth_frame);
          //float dis= GetBoxDepth2(box,aligned_depth_frame);
          cv::rectangle(img, lt, br, cv::Scalar(255, 0, 0), 1);
          //std::string lbl = cv::format("ID:%d_C:%d_CONF:%.2f", (int)box.trackID, (int)box.classID, box.confidence);
          //std::string lbl = cv::format("ID:%d_CONF:%.2fDis:%.3f", (int)box.trackID, box.confidence,dis);
          std::string lbl = cv::format("ID:%dA:%.2fD:%.2f", (int) box.trackID, box.angle, box.dis);
          //std::string lbl = cv::format("ID:%d_x:%f_y:%f",(int)box.trackID,(box.x1+box.x2)/2,(box.y1+box.y2)/2);
          cv::putText(img, lbl, lt, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0));*/
    }
}



