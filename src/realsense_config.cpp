//
// Created by lmy on 2022/5/13.
//
#include "realsense_config.h"


rs2::pipeline pipes;///生成Realsense管道，用来封装实际的相机设备
rs2::stream_profile dprofile;
rs2::pipeline_profile profile;
rs2::stream_profile cprofile;
Eigen::Matrix<float,3,3> MTR;//相机坐标旋转矩阵
Eigen::Vector3f V_T;//平移向量T
Eigen::Matrix<float,3,3> Inner_Transformation_Depth,InnerTransformation_Color;// 相机内参

int Realsense_config(){
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    /// Create librealsense context for managing devices
    rs2::context ctx;
    auto devs = ctx.query_devices();                  ///获取设备列表
    int device_num = devs.size();
    std::cout<<"device num: "<<device_num<<std::endl;///设备数量

    ///我只连了一个设备，因此我查看第0个设备的信息
    /// 当无设备连接时此处抛出rs2::error异常
    rs2::device dev = devs[0];
    ///设备编号，每个设备都有一个不同的编号, 如果连接了多个设备，便可根据此编号找到你希望启动的设备
    char serial_number[100] = {0};
    strcpy(serial_number, dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    printf("serial_number: %s\n",serial_number);

    ///设置从设备管道获取的深度图和彩色图的配置对象
    rs2::config cfg;
    ///配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒
    ///默认配置任意一个设备，若要配置指定的设备可以根据设备在设备列表里的序列号进行制定:
    ///int indx = 0; ///表示第0个设备
    ///cfg.enable_stream(RS2_STREAM_COLOR,indx, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_COLOR,640, 480, RS2_FORMAT_BGR8, 30);
    ///配置深度图像流：分辨率640*480，图像格式：Z16， 帧率：30帧/秒
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    profile =pipes.start(cfg); ///根据给定的配置启动相机管道

    return EXIT_SUCCESS;
}


//获取深度像素对应长度单位转换
float get_depth_scale(const rs2::device& dev)
{
    //前往摄像头传感器
    for (rs2::sensor & sensor : dev.query_sensors())//使用与，两者发生一个既可
    {
        //检查是否有深度图像
        if(rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())//检查是否有深度图
        {
            return dpt.get_depth_scale();//在数组中返回数值
        }
    }
    throw std::runtime_error("Device Error!");//发生错误打印
}


//深度图对齐到彩色图像
cv::Mat align_Depth2Color(cv::Mat depth, const cv::Mat &color, rs2::pipeline_profile profile) {
    //定义数据流深度与图像//auto默认类型rs2::video_stream_profile
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();//使用auto可以直接定义之前的数据类型
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获得内部参数(使用const只能在内部使用，与静态变量相似)
    const auto intrinDepth = depth_stream.get_intrinsics();//只能在函数内使用
    const auto intrinColor = color_stream.get_intrinsics();

    //直接获取从深度相机坐标系到彩色摄像头坐标系的欧拉转换矩阵
    rs2_extrinsics extrinDepth2Color;//声明
    rs2_error *error;
    rs2_get_extrinsics(depth_stream,color_stream,&extrinDepth2Color,&error);

    //平面点定义
    float pd_uv[2],pc_uv[2];//定义数组
    //空间点定义
    float Pdc3[3],Pcc3[3];

    //获得深度像素与现实单位比例
    float depth_scale = get_depth_scale(profile.get_device());
    int y,x;
    //初始化结果
    cv::Mat result = cv::Mat(color.rows,color.cols,CV_16U,cv::Scalar(0));
    //对深度图像处理
    for(int row=0;row<depth.rows;row++)
    {
        for(int col=0;col<depth.cols;col++)
        {
            pd_uv[0] = col;
            pd_uv[1] = row;
            //得到当前的深度数值
            uint16_t depth_value = depth.at<uint16_t>(row,col);
            //换算单位
            float depth_m = depth_value*depth_scale;//换算成米
            //深度图像的像素点转换为坐标下三维点
            rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_m);
            //深度相机坐标系的三维点转化到彩色的坐标系下
            rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
            //彩色摄像头坐标系下深度三维点映射到二位平面上
            rs2_project_point_to_pixel(pc_uv,&intrinColor,Pcc3);

            //取得映射后的（u,v）
            x = (int )pc_uv[0];//处理后的数据
            y = (int )pc_uv[1];

            x = x < 0 ? 0 : x;
            x = x > depth.cols-1 ? depth.cols-1 : x;
            y = y < 0 ? 0 : y;
            y = y > depth.rows-1 ? depth.rows-1 : y;

            result.at<uint16_t>(y,x)=depth_value;
        }
    }
    return result;//返回与彩色图对齐的图像
}


float measure_distance(cv::Mat &color,cv::Mat depth,DetectBox box,cv::Size range,rs2::pipeline_profile profile)//声明profile
{
    //获得深度像素与现实单位比例
    float depth_scale = get_depth_scale(profile.get_device());
    //定义图像中心点
    cv::Point center(box.pixel_x,box.pixel_y);
    //定义计算距离的范围
    cv::Rect RectRange(center.x-range.width/2,center.y-range.height/2,range.width,range.height);
    //std::cout<<box.x1<<" " <<box.y1<<" "<<box.x2 <<" "<<box.y2 <<std::endl;
    //画出范围
    float distance_sum = 0;
    int  effective_pixel = 0;

    for(int y = RectRange.y;y < RectRange.y + RectRange.height;y++)
    {
        for(int x = RectRange.x;x < RectRange.x + RectRange.width;x++)
        {
            //不是0就有位置信息
            if(depth.at<uint16_t>(y,x))//出现位置信息
            {
                distance_sum += depth_scale*depth.at<uint16_t>(y,x);
                effective_pixel++;
            }
        }
    }
    //std::cout << "有效像素点：" << effective_pixel <<std::endl;//输出数据

    float effective_distance = distance_sum/effective_pixel;
    //std::cout << "目标距离：" << effective_distance << "m" << std::endl;

    return effective_distance;
}

float getDistanceInMeters(DetectBox box,rs2::depth_frame alignDepthFrame,rs2::video_frame aligned_color_frame){
    rs2_extrinsics  extrinsics;
    rs2_error *error;
    extrinsics=alignDepthFrame.get_profile().as<rs2::video_stream_profile>().get_extrinsics_to(aligned_color_frame.get_profile());

    float x=(box.x1+box.x2)/2,y=(box.y1+box.y2)/2;
    float pd_uv[2];
    pd_uv[0]=x,pd_uv[1]=y;
    float Pdc3[3];
    const auto intrinDepth = alignDepthFrame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    float  pixel_distance_in_meters = alignDepthFrame.get_distance((int)x,(int)y);
    rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,pixel_distance_in_meters);

    cout<<"X Y Z"<<Pdc3[0]<<" "<<Pdc3[1]<<" "<<Pdc3[2]<<endl;
    cout<<"O DIS"<<sqrt(pow(Pdc3[0],2)+pow(Pdc3[1],2) +pow(Pdc3[2],2))<< endl;
    cout<<pixel_distance_in_meters<<endl;
    return pixel_distance_in_meters;
}



int Get_referance() //try
{
    ///获取彩色相机内参
    rs2::video_stream_profile cvsprofile(cprofile);
    rs2_intrinsics color_intrin =  cvsprofile.get_intrinsics();
    std::cout<<"\ncolor intrinsics: "<<std::endl;
    InnerTransformation_Color<<color_intrin.fx,0,color_intrin.ppx,0,color_intrin.fy,color_intrin.ppy,0,0,1;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            std::cout<<InnerTransformation_Color(i,j)<<"\t";
        }
        std::cout<<std::endl;
    }
    std::cout<<"distortion model: "<<color_intrin.model<<std::endl;///畸变模型

    ///获取深度相机内参
    rs2::video_stream_profile dvsprofile(dprofile);
    rs2_intrinsics depth_intrin =  dvsprofile.get_intrinsics();
    std::cout<<"\ndepth intrinsics: "<<std::endl;
    Inner_Transformation_Depth<<depth_intrin.fx,0,depth_intrin.ppx,0,depth_intrin.fy,depth_intrin.ppy,0,0,1;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            std::cout<<Inner_Transformation_Depth(i,j)<<"\t";
        }
        std::cout<<std::endl;
    }
    std::cout<<std::endl;
    std::cout<<"distortion model: "<<depth_intrin.model<<std::endl;///畸变模型

    ///获取深度相机相对于彩色相机的外参，即变换矩阵: P_color = R * P_depth + T
    rs2_extrinsics extrin = dprofile.get_extrinsics_to(cprofile);
    std::cout<<"\nextrinsics of depth camera to color camera: \nrotaion: "<<std::endl;
    MTR<<extrin.rotation[0],extrin.rotation[1],extrin.rotation[2],extrin.rotation[3],extrin.rotation[4],extrin.rotation[5],extrin.rotation[6],extrin.rotation[7],extrin.rotation[8];
    for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 3; ++j){
            std::cout<<MTR(i,j)<<"\t";
        }
        std::cout<<std::endl;
    }
    std::cout<<std::endl;
    std::cout<<"translation: ";
    V_T<<extrin.translation[0],extrin.translation[1],extrin.translation[2];
    for(int i=0;i<3;i++)
        std::cout<<V_T(i)<<"\t";
    std::cout<<std::endl;
    pipes.stop();
    return EXIT_SUCCESS;
}

