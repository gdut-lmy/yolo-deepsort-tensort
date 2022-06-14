#include "RealSenseD435.h"


/***************************************************************************************************************************
****************************************************************************************************************************
**************************************                                             *****************************************
**************************************             该部分为公有函数                *****************************************
**************************************                                             *****************************************
****************************************************************************************************************************
****************************************************************************************************************************/
RealSenseD435::RealSenseD435() {

    // 初始化若干标志位

    // 该标志为为false表示相机以默认格式获取图像
    Flg_InitColor = false;
    Flg_InitDepth = false;

    Flg_depth_align_to_color_get_coordinate = false;


/************************这一段仅与改变相机的属性有关****************************/
    rs2::context ctx;
    // 设备列表 它的个数是电脑连接的摄像头个数
    rs2::device_list list_of_devices = ctx.query_devices();
    // 从列表中选择出来的设备
    rs2::device choosed_device = list_of_devices[0];
    // 获取传感器列表
    list_of_sensors = choosed_device.query_sensors();
/*******************************************************************************/
}


RealSenseD435::~RealSenseD435() {
    // Terminate the pipeline
    pipe.stop();
    printf("~~~Release RealSenseD435()!\n");
}

/************************************************************************
*函数名; property_Control
*功能  : 这个函数用来控制D435的一些属性例如曝光 亮度 白平衡等等
*参数 1; ControlD435 枚举类型参数  表示要控制哪些属性
*参数 2; 重新设置的值  注意区域范围
************************************************************************/
void RealSenseD435::property_Control(ControlD435 controlD435, int NewValue) {

    rs2::sensor choosed_sensor; // 选择的传感器
    rs2_option option_type;     // 选择的要控制的属性类型

    int type_value = static_cast<int>(controlD435);

    if (type_value < static_cast<int>(ControlD435::Stereo_Module))
        choosed_sensor = list_of_sensors[1]; // 1 : RGB Camera
    else {
        choosed_sensor = list_of_sensors[0]; // 0 : Stereo Module
        // 与 Stereo Module 有关的枚举是从 100开始的
        type_value = type_value - 100;
    }

    // 将自己定义的枚举转换成官方的配置枚举
    option_type = static_cast<rs2_option>(type_value);

    // 设备可能不支持这一选项(防止输入错误)
    if (!choosed_sensor.supports(option_type))
        cout << " This option is not supported by this sensor" << endl;

    // 获取这个属性的可设置范围
    rs2::option_range range = choosed_sensor.get_option_range(option_type);

    // 范围的最大最小值
    int maximum = range.max;
    int minimum = range.min;

    if (NewValue > maximum)NewValue = maximum;
    if (NewValue < minimum)NewValue = minimum;

    // 改变该属性的值
    choosed_sensor.set_option(option_type, NewValue);
}

/************************************************************************
*函数名; controlWindows_depth
*功能  : 对彩色摄像头进行设置
*参数 1: 最近距离
*参数 2: 最远距离
************************************************************************/
void RealSenseD435::controlWindows_depth(int &near, int &far) {
    namedWindow("depth_control");
    createTrackbar("near:", "depth_control", &near, 8000, NULL);
    createTrackbar("far :", "depth_control", &far, 8000, NULL);
}

/************************************************************************
*函数名; colorInit
*功能  : 对彩色摄像头进行设置
*参数  : RealSenseColorType 类型的枚举
************************************************************************/
void RealSenseD435::colorInit(RealSenseColorType ColorType) {

    static ColorCfg _ColorCfg; // 相机控制参数

    // 获取控制格式
    ProduceColorCfg(ColorType, _ColorCfg);

    // Color   width   height  压缩格式  fps
    cfg.enable_stream(RS2_STREAM_COLOR, _ColorCfg.width, _ColorCfg.height, _ColorCfg.format_type, _ColorCfg.fps);

    // 保存类型  分辨率
    ColorImage_type = _ColorCfg.MaType;

    Flg_InitColor = true;

}

/************************************************************************
*函数名; depthInit
*功能  : 对深度输出图进行设置
*参数  : RealSenseDepthType 类型的枚举
************************************************************************/
void RealSenseD435::depthInit(RealSenseDepthType DepthType) {

    static DepthCfg _DepthCfg;// 相机控制参数

    // 获取控制格式
    ProduceDepthCfg(DepthType, _DepthCfg);

    // Depth   width   height  压缩格式  fps
    cfg.enable_stream(RS2_STREAM_DEPTH, _DepthCfg.width, _DepthCfg.height, RS2_FORMAT_Z16, _DepthCfg.fps);

    // 确认定义深度相机
    Flg_InitDepth = true;
}

/************************************************************************
*函数名; start
*功能  : 开启d435
************************************************************************/
void RealSenseD435::start() {

    // 开启摄像头
    // 开始摄像头后如果仅设置了其中一个流 在程序中调用其它流会报错

    // 没有设置过深度和彩色摄像头
    if (Flg_InitColor == false && Flg_InitDepth == false)
        pipe.start();
    else {
        // 仅设置了深度
        if (Flg_InitColor == false && Flg_InitDepth == true)
            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);

        // 仅设置了彩色
        if (Flg_InitColor == true && Flg_InitDepth == false)
            cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 90);

        pipe.start(cfg);
    }


}

/************************************************************************
*函数名; updateFrame
*功能  : 等待下一帧
************************************************************************/
void RealSenseD435::updateFrame() {
    // Wait for next set of frames
    frames = pipe.wait_for_frames();

}

/************************************************************************
*函数名; updateFrame
*功能  : 等待下一帧
************************************************************************/
void RealSenseD435::updateColor(_align mode) {

    if (mode == NOUSE) // 不使用对其
        color_frames = frames.get_color_frame();

    else if (mode == COLOR_ALIGN_TO_DEPTH) {  // 颜色对齐到深度
        ;
    } else {
        cerr << "updateColor()  输入参数错误" << endl;
        while (true);
    }
    Flg_depth_align_to_color_get_coordinate = true;
}

/************************************************************************
*函数名; updateFrame
*功能  : 等待下一帧
************************************************************************/
void RealSenseD435::updateDepth(_align mode) {

    if (mode == NOUSE) // 不使用对齐
        depth_frames = frames.get_depth_frame();

    else if (mode == DEPTH_ALIGN_TO_COLOR) { // 深度对齐到颜色

        /*
        使用对齐会很消耗时间  使用的分辨率越高消耗时间越多
        */

        if (Flg_InitDepth == false) {
            cerr << "updateDepth()  未定义深度相机" << endl;
            while (true);
        }

        // 获得颜色和深度框架并将深度框架与颜色框架对齐
        static rs2::align align(rs2_stream::RS2_STREAM_COLOR);
        static rs2::frameset aligned_frames; // 对齐帧

        aligned_frames = align.process(frames);// 对齐

        depth_frames = aligned_frames.get_depth_frame();
    } else {
        cerr << "updateDepth()  输入参数错误" << endl;
        while (true);
    }

    Coordinate_align = mode;
}

/************************************************************************
*函数名; get_colorImage
*功能  : 获取彩色图像
*参数  : 输出的彩色图像
************************************************************************/
void RealSenseD435::get_colorImage(Mat &colorImage) {

    // 查询帧大小（宽度和高度）
    const int w = color_frames.as<video_frame>().get_width();
    const int h = color_frames.as<video_frame>().get_height();

    // 将帧的默认格式转换为Mat格式
    if (Flg_InitColor == true)
        colorImage = Mat(Size(w, h), ColorImage_type, (void *) color_frames.get_data());
    else
        colorImage = Mat(Size(w, h), CV_8UC3, (void *) color_frames.get_data());

}

void RealSenseD435::get_depth_Mat_8UC1(Mat &depthImage) {
    // 查询帧大小（宽度和高度）
    const int w = depth_frames.as<video_frame>().get_width();
    const int h = depth_frames.as<video_frame>().get_height();


    depthImage = Mat(Size(w, h), CV_16UC1, (void *) depth_frames.get_data());
}


/************************************************************************
*函数名; get_depth_color_map
*功能  : 获取八位单通道深度图
*参数 1: 输出深度图像
*参数 2: 显示的最近距离 默认    0
*参数 3: 显示的最远距离 默认 8000
************************************************************************/
void RealSenseD435::get_depth_Mat_8UC1(Mat &depthImage, int near, int far) {

    // 查询帧大小（宽度和高度）
    const int w = depth_frames.as<video_frame>().get_width();
    const int h = depth_frames.as<video_frame>().get_height();

    static Mat dm;
    dm = Mat(Size(w, h), CV_16UC1, (void *) depth_frames.get_data());

    static Mat depthMat(Size(w, h), CV_8UC1);// 可视化后的对齐深度图
    static unsigned short *dmData;
    static unsigned short dmDataSrc;
    static uchar *data;


    // 改变图像深度 使图像可视化
    for (i = 0; i < h; i++) {
        dmData = dm.ptr<unsigned short>(i);
        data = depthMat.ptr<uchar>(i);
        for (j = 0; j < w; j++) {
            dmDataSrc = dmData[j];
            if (dmDataSrc > far || dmDataSrc < near)data[j] = 0;
            else data[j] = (uchar) (dmDataSrc >> 5);
        }
    }

    // 提取图像
    depthImage = depthMat;

}

/************************************************************************
*函数名; get_depth_color_map
*功能  : 获取可视化的深度图像
*参数  : 输出深度图像
************************************************************************/
void RealSenseD435::get_depth_color_map(Mat &depthImage) {

    //为深度数据的可视化显示深度着色器
    static colorizer color_map;

    // 查询帧大小（宽度和高度）
    const int w = depth_frames.as<video_frame>().get_width();
    const int h = depth_frames.as<video_frame>().get_height();

    static frame color_depth_frames;
    //color_depth_frames = color_map(depth_frames);

    // 从着色的深度数据中创建OpenCV大小（w，h）的OpenCV矩阵
    depthImage = Mat(Size(w, h), CV_8UC3, (void *) color_depth_frames.get_data());

}

/************************************************************************
*函数名; get_Coordinate
*功能  : 获取三维坐标
************************************************************************/
void RealSenseD435::get_Coordinate() {

    // 声明pointcloud对象，用于计算pointcloud和纹理映射
    // 我们想让点对象保持持久，这样我们就可以在帧下降时显示最后的云
    static rs2::pointcloud pc;
    static rs2::points points;

    // 生成pointcloud和纹理映射
    points = pc.calculate(depth_frames);

    if (Coordinate_align == DEPTH_ALIGN_TO_COLOR) {
        if (Flg_depth_align_to_color_get_coordinate == false) {
            cout << "error: 使用get_Coordinate()前，没有获取彩色帧，终止程序检查获取帧顺序" << endl;
            while (1);
        }
        Flg_depth_align_to_color_get_coordinate = true;
        pc.map_to(color_frames);
    }


    // 获取实际的三维坐标
    tex_vertex = points.get_vertices();

    // 记录宽度 读取三维坐标点时会用到
    Dwidth = depth_frames.as<video_frame>().get_width();
}

/************************************************************************
*函数名; point2D_to_Point3D
*功能  : 获得某一点的三维坐标
*参数 1: 输出的对齐后的深度图像
*参数 2: 显示的最近距离 默认     0
************************************************************************/
void RealSenseD435::point2D_to_Point3D(cv::Point2i point2D, Point3D &point3D) {

    point3D.W = (short) (tex_vertex[point2D.y * Dwidth + point2D.x].x * 1000);
    point3D.H = (short) (tex_vertex[point2D.y * Dwidth + point2D.x].y * 1000);
    point3D.D = (unsigned short) (tex_vertex[point2D.y * Dwidth + point2D.x].z * 1000);
}










/***************************************************************************************************************************
****************************************************************************************************************************
**************************************                                             *****************************************
**************************************             该部分为私有函数                *****************************************
**************************************                                             *****************************************
****************************************************************************************************************************
****************************************************************************************************************************/
/************************************************************************
*函数名; ProduceColorCfg
*功能  : 产生 RealSenseColorType 枚举对应的 D435 彩色相机参数
*参数 1: RealSenseColorType 类型枚举
*参数 2: ColorCfg类型结构体 内含相机参数
************************************************************************/
void RealSenseD435::ProduceColorCfg(RealSenseColorType Colortype, ColorCfg &_ColorCfg) {
    switch (Colortype) {
        case COLOR_Y16_1920x1080_30Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 1080;
            _ColorCfg.width = 1920;
            break;
        case COLOR_BGR8_1920x1080_30Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 1080;
            _ColorCfg.width = 1920;
            break;
        case COLOR_Y16_1920x1080_15Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 1080;
            _ColorCfg.width = 1920;
            break;
        case COLOR_BGR8_1920x1080_15Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 1080;
            _ColorCfg.width = 1920;
            break;
        case COLOR_Y16_1920x1080_6Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 1080;
            _ColorCfg.width = 1920;
            break;
        case COLOR_BGR8_1920x1080_6Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 1080;
            _ColorCfg.width = 1920;
            break;
        case COLOR_Y16_1280x720_30Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 720;
            _ColorCfg.width = 1280;
            break;
        case COLOR_BGR8_1280x720_30Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 720;
            _ColorCfg.width = 1280;
            break;
        case COLOR_Y16_1280x720_15Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 720;
            _ColorCfg.width = 1280;
            break;
        case COLOR_BGR8_1280x720_15Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 720;
            _ColorCfg.width = 1280;
            break;
        case COLOR_Y16_1280x720_6Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 720;
            _ColorCfg.width = 1280;
            break;
        case COLOR_BGR8_1280x720_6Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 720;
            _ColorCfg.width = 1280;
            break;
        case COLOR_Y16_960x540_60Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 540;
            _ColorCfg.width = 960;
            break;
        case COLOR_BGR8_960x540_60Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 540;
            _ColorCfg.width = 960;
            break;
        case COLOR_Y16_960x540_30Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 540;
            _ColorCfg.width = 960;
            break;
        case COLOR_BGR8_960x540_30Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 540;
            _ColorCfg.width = 960;
            break;
        case COLOR_Y16_960x540_15Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 540;
            _ColorCfg.width = 960;
            break;
        case COLOR_BGR8_960x540_15Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 540;
            _ColorCfg.width = 960;
            break;
        case COLOR_Y16_960x540_6Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 540;
            _ColorCfg.width = 960;
            break;
        case COLOR_BGR8_960x540_6Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 540;
            _ColorCfg.width = 960;
            break;
        case COLOR_Y16_848x480_60Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 480;
            _ColorCfg.width = 848;
            break;
        case COLOR_BGR8_848x480_60Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 480;
            _ColorCfg.width = 848;
            break;
        case COLOR_Y16_848x480_30Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 480;
            _ColorCfg.width = 848;
            break;
        case COLOR_BGR8_848x480_30Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 480;
            _ColorCfg.width = 848;
            break;
        case COLOR_Y16_848x480_15Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 480;
            _ColorCfg.width = 848;
            break;
        case COLOR_BGR8_848x480_15Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 480;
            _ColorCfg.width = 848;
            break;
        case COLOR_Y16_848x480_6Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 480;
            _ColorCfg.width = 848;
            break;
        case COLOR_BGR8_848x480_6Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 480;
            _ColorCfg.width = 848;
            break;
        case COLOR_Y16_640x480_60Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 480;
            _ColorCfg.width = 640;
            break;
        case COLOR_BGR8_640x480_60Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 480;
            _ColorCfg.width = 640;
            break;
        case COLOR_Y16_640x480_30Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 480;
            _ColorCfg.width = 640;
            break;
        case COLOR_BGR8_640x480_30Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 480;
            _ColorCfg.width = 640;
            break;
        case COLOR_Y16_640x480_15Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 480;
            _ColorCfg.width = 640;
            break;
        case COLOR_BGR8_640x480_15Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 480;
            _ColorCfg.width = 640;
            break;
        case COLOR_Y16_640x480_6Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 480;
            _ColorCfg.width = 640;
            break;
        case COLOR_BGR8_640x480_6Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 480;
            _ColorCfg.width = 640;
            break;
        case COLOR_Y16_640x360_60Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 360;
            _ColorCfg.width = 640;
            break;
        case COLOR_BGR8_640x360_60Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 360;
            _ColorCfg.width = 640;
            break;
        case COLOR_Y16_640x360_30Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 360;
            _ColorCfg.width = 640;
            break;
        case COLOR_BGR8_640x360_30Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 360;
            _ColorCfg.width = 640;
            break;
        case COLOR_Y16_640x360_15Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 360;
            _ColorCfg.width = 640;
            break;
        case COLOR_BGR8_640x360_15Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 360;
            _ColorCfg.width = 640;
            break;
        case COLOR_Y16_640x360_6Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 360;
            _ColorCfg.width = 640;
            break;
        case COLOR_BGR8_640x360_6Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 360;
            _ColorCfg.width = 640;
            break;
        case COLOR_Y16_424x240_60Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 240;
            _ColorCfg.width = 424;
            break;
        case COLOR_BGR8_424x240_60Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 240;
            _ColorCfg.width = 424;
            break;
        case COLOR_Y16_424x240_30Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 240;
            _ColorCfg.width = 424;
            break;
        case COLOR_BGR8_424x240_30Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 240;
            _ColorCfg.width = 424;
            break;
        case COLOR_Y16_424x240_15Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 240;
            _ColorCfg.width = 424;
            break;
        case COLOR_BGR8_424x240_15Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 15;
            _ColorCfg.height = 240;
            _ColorCfg.width = 424;
            break;
        case COLOR_Y16_424x240_6Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 240;
            _ColorCfg.width = 424;
            break;
        case COLOR_BGR8_424x240_6Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 240;
            _ColorCfg.width = 424;
            break;
        case COLOR_Y16_320x240_60Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 240;
            _ColorCfg.width = 320;
            break;
        case COLOR_BGR8_320x240_60Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 240;
            _ColorCfg.width = 320;
            break;
        case COLOR_Y16_320x240_30Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 240;
            _ColorCfg.width = 320;
            break;
        case COLOR_BGR8_320x240_30Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 240;
            _ColorCfg.width = 320;
            break;
        case COLOR_Y16_320x240_6Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 240;
            _ColorCfg.width = 320;
            break;
        case COLOR_BGR8_320x240_6Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 240;
            _ColorCfg.width = 320;
            break;
        case COLOR_Y16_320x180_60Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 180;
            _ColorCfg.width = 320;
            break;
        case COLOR_BGR8_320x180_60Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 60;
            _ColorCfg.height = 180;
            _ColorCfg.width = 320;
            break;
        case COLOR_Y16_320x180_30Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 180;
            _ColorCfg.width = 320;
            break;
        case COLOR_BGR8_320x180_30Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 30;
            _ColorCfg.height = 180;
            _ColorCfg.width = 320;
            break;
        case COLOR_Y16_320x180_6Hz:
            _ColorCfg.MaType = CV_16UC1;
            _ColorCfg.format_type = RS2_FORMAT_Y16;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 180;
            _ColorCfg.width = 320;
            break;
        case COLOR_BGR8_320x180_6Hz:
            _ColorCfg.MaType = CV_8UC3;
            _ColorCfg.format_type = RS2_FORMAT_BGR8;
            _ColorCfg.fps = 6;
            _ColorCfg.height = 180;
            _ColorCfg.width = 320;
            break;
    }
}

/************************************************************************
*函数名; ProduceDepthCfg
*功能  : 产生 RealSenseDepthType 枚举对应的 D435 深度图参数
*参数 1: RealSenseDepthType 类型枚举
*参数 2: ColorCfg类型结构体 内含相机参数
************************************************************************/
void RealSenseD435::ProduceDepthCfg(RealSenseDepthType Colortype, DepthCfg &_DepthCfg) {
    switch (Colortype) {
        case DEPTH_Z16_1280x720_30HZ:
            _DepthCfg.fps = 30;
            _DepthCfg.height = 720;
            _DepthCfg.width = 1280;
            break;
        case DEPTH_Z16_1280x720_15HZ:
            _DepthCfg.fps = 15;
            _DepthCfg.height = 720;
            _DepthCfg.width = 1280;
            break;
        case DEPTH_Z16_1280x720_6HZ:
            _DepthCfg.fps = 6;
            _DepthCfg.height = 720;
            _DepthCfg.width = 1280;
            break;
        case DEPTH_Z16_848x480_90HZ:
            _DepthCfg.fps = 90;
            _DepthCfg.height = 480;
            _DepthCfg.width = 848;
            break;
        case DEPTH_Z16_848x480_60HZ:
            _DepthCfg.fps = 60;
            _DepthCfg.height = 480;
            _DepthCfg.width = 848;
            break;
        case DEPTH_Z16_848x480_30HZ:
            _DepthCfg.fps = 30;
            _DepthCfg.height = 480;
            _DepthCfg.width = 848;
            break;
        case DEPTH_Z16_848x480_15HZ:
            _DepthCfg.fps = 15;
            _DepthCfg.height = 480;
            _DepthCfg.width = 848;
            break;
        case DEPTH_Z16_848x480_6HZ:
            _DepthCfg.fps = 6;
            _DepthCfg.height = 480;
            _DepthCfg.width = 848;
            break;
        case DEPTH_Z16_640x480_90HZ:
            _DepthCfg.fps = 90;
            _DepthCfg.height = 480;
            _DepthCfg.width = 640;
            break;
        case DEPTH_Z16_640x480_60HZ:
            _DepthCfg.fps = 60;
            _DepthCfg.height = 480;
            _DepthCfg.width = 640;
            break;
        case DEPTH_Z16_640x480_30HZ:
            _DepthCfg.fps = 30;
            _DepthCfg.height = 480;
            _DepthCfg.width = 640;
            break;
        case DEPTH_Z16_640x480_15HZ:
            _DepthCfg.fps = 15;
            _DepthCfg.height = 480;
            _DepthCfg.width = 640;
            break;
        case DEPTH_Z16_640x480_6HZ:
            _DepthCfg.fps = 6;
            _DepthCfg.height = 480;
            _DepthCfg.width = 640;
            break;
        case DEPTH_Z16_640x360_90HZ:
            _DepthCfg.fps = 90;
            _DepthCfg.height = 360;
            _DepthCfg.width = 640;
            break;
        case DEPTH_Z16_640x360_60HZ:
            _DepthCfg.fps = 60;
            _DepthCfg.height = 360;
            _DepthCfg.width = 640;
            break;
        case DEPTH_Z16_640x360_30HZ:
            _DepthCfg.fps = 30;
            _DepthCfg.height = 360;
            _DepthCfg.width = 640;
            break;
        case DEPTH_Z16_640x360_15HZ:
            _DepthCfg.fps = 15;
            _DepthCfg.height = 360;
            _DepthCfg.width = 640;
            break;
        case DEPTH_Z16_640x360_6HZ:
            _DepthCfg.fps = 6;
            _DepthCfg.height = 360;
            _DepthCfg.width = 640;
            break;
        case DEPTH_Z16_480x270_90HZ:
            _DepthCfg.fps = 90;
            _DepthCfg.height = 270;
            _DepthCfg.width = 480;
            break;
        case DEPTH_Z16_480x270_60HZ:
            _DepthCfg.fps = 60;
            _DepthCfg.height = 270;
            _DepthCfg.width = 480;
            break;
        case DEPTH_Z16_480x270_30HZ:
            _DepthCfg.fps = 30;
            _DepthCfg.height = 270;
            _DepthCfg.width = 480;
            break;
        case DEPTH_Z16_480x270_15HZ:
            _DepthCfg.fps = 15;
            _DepthCfg.height = 270;
            _DepthCfg.width = 480;
            break;
        case DEPTH_Z16_480x270_6HZ:
            _DepthCfg.fps = 6;
            _DepthCfg.height = 270;
            _DepthCfg.width = 480;
            break;
        case DEPTH_Z16_424x240_90HZ:
            _DepthCfg.fps = 90;
            _DepthCfg.height = 240;
            _DepthCfg.width = 424;
            break;
        case DEPTH_Z16_424x240_60HZ:
            _DepthCfg.fps = 60;
            _DepthCfg.height = 240;
            _DepthCfg.width = 424;
            break;
        case DEPTH_Z16_424x240_30HZ:
            _DepthCfg.fps = 30;
            _DepthCfg.height = 240;
            _DepthCfg.width = 424;
            break;
        case DEPTH_Z16_424x240_15HZ:
            _DepthCfg.fps = 15;
            _DepthCfg.height = 240;
            _DepthCfg.width = 424;
            break;
        case DEPTH_Z16_424x240_6HZ:
            _DepthCfg.fps = 6;
            _DepthCfg.height = 240;
            _DepthCfg.width = 424;
            break;
    }
}
