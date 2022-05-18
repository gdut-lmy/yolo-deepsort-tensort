#include "manager.hpp"
using std::vector;
using namespace cv;
static Logger gLogger;

Trtyolosort::Trtyolosort(char *yolo_engine_path,char *sort_engine_path){
	sort_engine_path_ = sort_engine_path;
	yolo_engine_path_ = yolo_engine_path;
	trt_engine = yolov5_trt_create(yolo_engine_path_);
	printf("create yolov5-trt , instance = %p\n", trt_engine);
	DS = new DeepSort(sort_engine_path_, 128, 256, 0, &gLogger);

}
void Trtyolosort::showDetection(cv::Mat& img, std::vector<DetectBox>& boxes,rs2::depth_frame aligned_depth_frame) {
    cv::Mat temp = img.clone();
    for (auto box : boxes) {
        cv::Point lt(box.x1, box.y1);
        cv::Point br(box.x2, box.y2);
        cout<<"Track ID"<<box.trackID<<endl;
        //float dis= getDistanceInMeters(box,aligned_depth_frame);

        float dis= GetBoxDepth(box,aligned_depth_frame);
        cv::rectangle(temp, lt, br, cv::Scalar(255, 0, 0), 1);
        //std::string lbl = cv::format("ID:%d_C:%d_CONF:%.2f", (int)box.trackID, (int)box.classID, box.confidence);
		std::string lbl = cv::format("ID:%d_CONF:%.2fDis:%.3f", (int)box.trackID, box.confidence,dis);
		//std::string lbl = cv::format("ID:%d_x:%f_y:%f",(int)box.trackID,(box.x1+box.x2)/2,(box.y1+box.y2)/2);
        cv::putText(temp, lbl, lt, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,255,0));
    }
    cv::imshow("img", temp);
    cv::waitKey(1);
}


float Trtyolosort::Get_Area_Depth(DetectBox box) {

    cv::Rect Box=cv::Rect(box.x1,box.y1,box.x2-box.x1,box.y2-box.y1);
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




float Trtyolosort::GetBoxDepth(DetectBox box,rs2::depth_frame alignedDepthFrame) {
    //Mat result = align_Depth2Color(depthMat,colorMat,profile);//调用对齐函数
    float dis= measure_distance(colorMat, depthMat, box, cv::Size(5,5 ), profile);
    float x=(box.x1+box.x2)/2,y=(box.y1+box.y2)/2;
    float pd_uv[2];
    pd_uv[0]=x,pd_uv[1]=y;
    float Pdc3[3];
    const auto intrinDepth = alignedDepthFrame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    //float  pixel_distance_in_meters = alignedDepthFrame.get_distance((int)x,(int)y);
    rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,dis);
    cout<<"X Y Z : "<<Pdc3[0]<<" "<<Pdc3[1]<<" "<<Pdc3[2]<<endl;
    //cout<<"O DIS : "<<sqrt(pow(Pdc3[0],2)+pow(Pdc3[1],2) +pow(Pdc3[2],2))<< endl;
    cout<<"Dis : "<<dis<<endl;
    return dis;
}


int Trtyolosort::TrtDetect(cv::Mat &frame,float &conf_thresh,std::vector<DetectBox> &det,rs2::depth_frame aligned_depth_frame){
	// yolo detect
	auto ret = yolov5_trt_detect(trt_engine, frame, conf_thresh,det);
	DS->sort(frame,det);
	showDetection(frame,det,aligned_depth_frame);
	return 1 ;
	
}
