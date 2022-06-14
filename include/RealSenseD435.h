#pragma once
#ifndef _REALSENSED435_H_
#define _REALSENSED435_H_
#include <librealsense2/rs.hpp> 
#include <librealsense2/rsutil.h> 
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;
using namespace rs2;

/*
这个枚举用来控制D435的一些属性
例如曝光 亮度 白平衡等等
顺序是根据设备列表写出的,这里的顺序和数值均不可更改
*/
typedef enum _ControlD435 {
	ENABLE = 0, // 启用
	DISABLE,    // 禁用

	COLOR_Backlight_Compensation = 0, // 启用/禁用 背光补偿   默认禁用
	COLOR_Brightness,			      // 彩色图像亮度         默认 0  		范围(-64,64)
	COLOR_Contrast,			          // 彩色图像对比度       默认 50 		范围(0,100) 
	COLOR_Exposure,                   // 控制彩色相机的曝光时间。设置任何值将禁用自动曝光   
									  //                      默认 156      范围(39,10000) 
	COLOR_Gain,                       // 彩色图像增益         默认 64       范围(0,128) 
	COLOR_Gamma,					  // 彩色图像伽马设置     默认 300      范围(100,500) 
	COLOR_Hue,						  // 彩色图像色调         默认 0        范围(-180,180) 
	COLOR_Saturation,				  // 彩色图像饱和度设置   默认 64       范围(0,100) 
	COLOR_Sharpness,				  // 彩色图像清晰度设置   默认 50       范围(0,100) 
	COLOR_White_Balance,              // 控制白色的彩色图像平衡。设置任何值将禁用自动白平衡  
									  //                      默认 4600     范围(2800,6500) 
	COLOR_Enable_Auto_Exposure,       // 启用/禁用 自动曝光   默认启用
	COLOR_Enable_Auto_White_Balance,  // 启用/禁用 自动白平衡 默认启用
	COLOR_Frames_Queue_Size = 19,     // 在给定的时间内，你可以持有最多的帧数。增加这个数字将减少帧数，但增加延迟，反之亦然
									  //                      默认 16
	COLOR_Power_Line_Frequency = 22,  // 电力线频率           默认 Disalbe   其它值 50HZ,60HZ,Auto
    COLOR_Auto_Exposure_Priority = 30,// 自动曝光时限制曝光时间以保持恒定的FPS速率
									  //                      默认 1

	// 从这里断开 后面立体视觉有关的控制组件从100开始  使它有别于彩色相机
	Stereo_Module = 99,

	STEREO_Exposure = 103,              // 立体成像仪图像曝光   默认 8500     范围(20,166000)  
	STEREO_Gain = 104,                  // 立体成像仪图像增益	默认 16       范围(16,248)
	STEREO_Enable_Auto_Exposure = 110,  // 启用/禁用 自动曝光   默认启用
	STEREO_Visual_Preset = 112,		    // 高级模式预置
	STEREO_Laser_Power,				    // 手动激光功率为兆瓦。仅当激光功率模式设置为手动时才适用
										//						默认 150      范围(0,360)
	STEREO_Emitter_Enabled = 118,	    // DS5投影仪的功率，0表示投影仪关闭，1指投影仪开启，2指自动投影仪模式
	STEREO_Frames_Queue_Size,		    // 在给定的时间内，你可以持有最多的帧数。增加这个数字将减少帧数，但增加延迟，反之亦然
	STEREO_Error_Polling_Enabled = 124, // 启用/禁用 摄像机内部错误轮询         默认启用
	STEREO_Output_Trigger_Enabled = 126,// 从相机到外部设备每帧生成一次触发器   默认禁用
	STEREO_Depth_Units = 128,           // 由单个深度单元表示的米数  
	STEREO_Stereo_Baseline = 140,		// 立体成像仪间毫米距离  

}ControlD435;


/*彩色图像用的枚举
Color  压缩格式  分辨率  帧率*/
typedef enum _RealSenseColorType {
	COLOR_Y16_1920x1080_30Hz,
	COLOR_BGR8_1920x1080_30Hz,
	COLOR_Y16_1920x1080_15Hz,
	COLOR_BGR8_1920x1080_15Hz,
	COLOR_Y16_1920x1080_6Hz,
	COLOR_BGR8_1920x1080_6Hz,
	COLOR_Y16_1280x720_30Hz,
	COLOR_BGR8_1280x720_30Hz,
	COLOR_Y16_1280x720_15Hz,
	COLOR_BGR8_1280x720_15Hz,
	COLOR_Y16_1280x720_6Hz,
	COLOR_BGR8_1280x720_6Hz,
	COLOR_Y16_960x540_60Hz,
	COLOR_BGR8_960x540_60Hz,
	COLOR_Y16_960x540_30Hz,
	COLOR_BGR8_960x540_30Hz,
	COLOR_Y16_960x540_15Hz,
	COLOR_BGR8_960x540_15Hz,
	COLOR_Y16_960x540_6Hz,
	COLOR_BGR8_960x540_6Hz,
	COLOR_Y16_848x480_60Hz,
	COLOR_BGR8_848x480_60Hz,
	COLOR_Y16_848x480_30Hz,
	COLOR_BGR8_848x480_30Hz,
	COLOR_Y16_848x480_15Hz,
	COLOR_BGR8_848x480_15Hz,
	COLOR_Y16_848x480_6Hz,
	COLOR_BGR8_848x480_6Hz,
	COLOR_Y16_640x480_60Hz,
	COLOR_BGR8_640x480_60Hz,
	COLOR_Y16_640x480_30Hz,
	COLOR_BGR8_640x480_30Hz,
	COLOR_Y16_640x480_15Hz,
	COLOR_BGR8_640x480_15Hz,
	COLOR_Y16_640x480_6Hz,
	COLOR_BGR8_640x480_6Hz,
	COLOR_Y16_640x360_60Hz,
	COLOR_BGR8_640x360_60Hz,
	COLOR_Y16_640x360_30Hz,
	COLOR_BGR8_640x360_30Hz,
	COLOR_Y16_640x360_15Hz,
	COLOR_BGR8_640x360_15Hz,
	COLOR_Y16_640x360_6Hz,
	COLOR_BGR8_640x360_6Hz,
	COLOR_Y16_424x240_60Hz,
	COLOR_BGR8_424x240_60Hz,
	COLOR_Y16_424x240_30Hz,
	COLOR_BGR8_424x240_30Hz,
	COLOR_Y16_424x240_15Hz,
	COLOR_BGR8_424x240_15Hz,
	COLOR_Y16_424x240_6Hz,
	COLOR_BGR8_424x240_6Hz,
	COLOR_Y16_320x240_60Hz,
	COLOR_BGR8_320x240_60Hz,
	COLOR_Y16_320x240_30Hz,
	COLOR_BGR8_320x240_30Hz,
	COLOR_Y16_320x240_6Hz,
	COLOR_BGR8_320x240_6Hz,
	COLOR_Y16_320x180_60Hz,
	COLOR_BGR8_320x180_60Hz,
	COLOR_Y16_320x180_30Hz,
	COLOR_BGR8_320x180_30Hz,
	COLOR_Y16_320x180_6Hz,
	COLOR_BGR8_320x180_6Hz,
} RealSenseColorType;


/*深度图像用的枚举
DEPTH  压缩格式  分辨率  帧率*/
typedef enum _RealSenseDepthType {
	DEPTH_Z16_1280x720_30HZ,
	DEPTH_Z16_1280x720_15HZ,
	DEPTH_Z16_1280x720_6HZ,
	DEPTH_Z16_848x480_90HZ,
	DEPTH_Z16_848x480_60HZ,
	DEPTH_Z16_848x480_30HZ,
	DEPTH_Z16_848x480_15HZ,
	DEPTH_Z16_848x480_6HZ,
	DEPTH_Z16_640x480_90HZ,
	DEPTH_Z16_640x480_60HZ,
	DEPTH_Z16_640x480_30HZ,
	DEPTH_Z16_640x480_15HZ,
	DEPTH_Z16_640x480_6HZ,
	DEPTH_Z16_640x360_90HZ,
	DEPTH_Z16_640x360_60HZ,
	DEPTH_Z16_640x360_30HZ,
	DEPTH_Z16_640x360_15HZ,
	DEPTH_Z16_640x360_6HZ,
	DEPTH_Z16_480x270_90HZ,
	DEPTH_Z16_480x270_60HZ,
	DEPTH_Z16_480x270_30HZ,
	DEPTH_Z16_480x270_15HZ,
	DEPTH_Z16_480x270_6HZ,
	DEPTH_Z16_424x240_90HZ,
	DEPTH_Z16_424x240_60HZ,
	DEPTH_Z16_424x240_30HZ,
	DEPTH_Z16_424x240_15HZ,
	DEPTH_Z16_424x240_6HZ
} RealSenseDepthType;

/*图像对其用到的枚举*/
typedef enum _align_ { 
	NOUSE,                 // 不使用对齐
	DEPTH_ALIGN_TO_COLOR,  // 深度对齐到颜色
	COLOR_ALIGN_TO_DEPTH   // 颜色对齐到深度
}_align;

/*某一点的三维坐标*/
typedef struct {
	short W;
	short H;
	unsigned short D;
}Point3D;


class RealSenseD435 {
public:
	RealSenseD435();
	~RealSenseD435();
	/*******************    相机属性控制D435    ********************/
	void property_Control(ControlD435 controlD435, int NewValue);

	/***********************    启动D435    ***********************/
	void start();

	/***********************    控制窗口    ***********************/
	void controlWindows_depth(int &near, int &far);


	/***********************    相机设置    ***********************/
	void colorInit(RealSenseColorType ColorType);//初始化彩色相机
	void depthInit(RealSenseDepthType DepthType);//初始化深度

  

	/***********************    数据更新    ***********************/
	void updateFrame();                   // 更新帧数据    
	void updateColor(_align mode = NOUSE);// 更新彩色流
	void updateDepth(_align mode = NOUSE);// 更新深度流


	/***********************    获取信息    ***********************/
	void get_colorImage(Mat &colorImage);					       // 获取彩色图
    void get_depth_Mat_8UC1(Mat&depthImage);
	void get_depth_Mat_8UC1(Mat &depthImage, int near, int far);   // 获取八位单通道深度图
	void get_depth_color_map(Mat &depthImage);				       // 获取可视化的深度图像
	void get_Coordinate();										   // 更新三维坐标
	void point2D_to_Point3D(cv::Point2i point2D, Point3D &point3D);// 获得某一点的三维坐标
	

	// 允许外界对该数据流进行处理
	rs2::frame depth_frames; // 获取深度图的 frame

private:


	/* 该结构体记录彩色摄像头图像 压缩格式 帧率 分辨率 以及用于显示的图像类型*/
	typedef struct {
		int             MaType;  // Mat 矩阵的类型
		rs2_format format_type;  // 摄像头图像压缩格式
		int				 width;  // 宽度
		int				height;  // 高度
		int                fps;  // 帧率
	}ColorCfg;

	/* 该结构体记录深度图像的 帧率 分辨率*/
	typedef struct {
		int	 width;  // 宽度
		int	height;  // 高度
		int    fps;  // 帧率
	}DepthCfg;
	
	/* 产生枚举对应的控制参数 */
	void  ProduceColorCfg(RealSenseColorType Colortype, ColorCfg &_ColorCfg);
	
	/* 产生枚举对应的控制参数 */
	void  ProduceDepthCfg(RealSenseDepthType Colortype, DepthCfg &_DepthCfg);

private:
	/************************     D435属性控制组件     ************************/
	// 传感器 D435 有两种 0 : Stereo Module 1 : RGB Camera
	std::vector<rs2::sensor> list_of_sensors;
	/**************************************************************************/
	rs2::pipeline		     pipe; // 设备管道
	rs2::config			      cfg; // 控制组建
	rs2::frameset          frames; 
	rs2::pipeline_profile profile;
	rs2::frame       color_frames; // 获取彩色图像的 frame
	

	int  i, j;            // 循环变量
	int  ColorImage_type; // 彩色图像的类型
	int  Dwidth;          // 深度图宽度
	_align Coordinate_align;
	// 存储三维坐标的结构体
	const vertex* tex_vertex;

	// 一系列标志位保护程序
	bool Flg_depth_align_to_color_get_coordinate;// 标志深度图映射到了彩色
	bool Flg_InitColor;							 // 定义彩色图标志
	bool Flg_InitDepth;							 // 定义深度图标志

};
#endif


