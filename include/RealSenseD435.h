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
���ö����������D435��һЩ����
�����ع� ���� ��ƽ��ȵ�
˳���Ǹ����豸�б�д����,�����˳�����ֵ�����ɸ���
*/
typedef enum _ControlD435 {
	ENABLE = 0, // ����
	DISABLE,    // ����

	COLOR_Backlight_Compensation = 0, // ����/���� ���ⲹ��   Ĭ�Ͻ���
	COLOR_Brightness,			      // ��ɫͼ������         Ĭ�� 0  		��Χ(-64,64)
	COLOR_Contrast,			          // ��ɫͼ��Աȶ�       Ĭ�� 50 		��Χ(0,100) 
	COLOR_Exposure,                   // ���Ʋ�ɫ������ع�ʱ�䡣�����κ�ֵ�������Զ��ع�   
									  //                      Ĭ�� 156      ��Χ(39,10000) 
	COLOR_Gain,                       // ��ɫͼ������         Ĭ�� 64       ��Χ(0,128) 
	COLOR_Gamma,					  // ��ɫͼ��٤������     Ĭ�� 300      ��Χ(100,500) 
	COLOR_Hue,						  // ��ɫͼ��ɫ��         Ĭ�� 0        ��Χ(-180,180) 
	COLOR_Saturation,				  // ��ɫͼ�񱥺Ͷ�����   Ĭ�� 64       ��Χ(0,100) 
	COLOR_Sharpness,				  // ��ɫͼ������������   Ĭ�� 50       ��Χ(0,100) 
	COLOR_White_Balance,              // ���ư�ɫ�Ĳ�ɫͼ��ƽ�⡣�����κ�ֵ�������Զ���ƽ��  
									  //                      Ĭ�� 4600     ��Χ(2800,6500) 
	COLOR_Enable_Auto_Exposure,       // ����/���� �Զ��ع�   Ĭ������
	COLOR_Enable_Auto_White_Balance,  // ����/���� �Զ���ƽ�� Ĭ������
	COLOR_Frames_Queue_Size = 19,     // �ڸ�����ʱ���ڣ�����Գ�������֡��������������ֽ�����֡�����������ӳ٣���֮��Ȼ
									  //                      Ĭ�� 16
	COLOR_Power_Line_Frequency = 22,  // ������Ƶ��           Ĭ�� Disalbe   ����ֵ 50HZ,60HZ,Auto
    COLOR_Auto_Exposure_Priority = 30,// �Զ��ع�ʱ�����ع�ʱ���Ա��ֺ㶨��FPS����
									  //                      Ĭ�� 1

	// ������Ͽ� ���������Ӿ��йصĿ��������100��ʼ  ʹ���б��ڲ�ɫ���
	Stereo_Module = 99,

	STEREO_Exposure = 103,              // ���������ͼ���ع�   Ĭ�� 8500     ��Χ(20,166000)  
	STEREO_Gain = 104,                  // ���������ͼ������	Ĭ�� 16       ��Χ(16,248)
	STEREO_Enable_Auto_Exposure = 110,  // ����/���� �Զ��ع�   Ĭ������
	STEREO_Visual_Preset = 112,		    // �߼�ģʽԤ��
	STEREO_Laser_Power,				    // �ֶ����⹦��Ϊ���ߡ��������⹦��ģʽ����Ϊ�ֶ�ʱ������
										//						Ĭ�� 150      ��Χ(0,360)
	STEREO_Emitter_Enabled = 118,	    // DS5ͶӰ�ǵĹ��ʣ�0��ʾͶӰ�ǹرգ�1ָͶӰ�ǿ�����2ָ�Զ�ͶӰ��ģʽ
	STEREO_Frames_Queue_Size,		    // �ڸ�����ʱ���ڣ�����Գ�������֡��������������ֽ�����֡�����������ӳ٣���֮��Ȼ
	STEREO_Error_Polling_Enabled = 124, // ����/���� ������ڲ�������ѯ         Ĭ������
	STEREO_Output_Trigger_Enabled = 126,// ��������ⲿ�豸ÿ֡����һ�δ�����   Ĭ�Ͻ���
	STEREO_Depth_Units = 128,           // �ɵ�����ȵ�Ԫ��ʾ������  
	STEREO_Stereo_Baseline = 140,		// ��������Ǽ���׾���  

}ControlD435;


/*��ɫͼ���õ�ö��
Color  ѹ����ʽ  �ֱ���  ֡��*/
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


/*���ͼ���õ�ö��
DEPTH  ѹ����ʽ  �ֱ���  ֡��*/
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

/*ͼ������õ���ö��*/
typedef enum _align_ { 
	NOUSE,                 // ��ʹ�ö���
	DEPTH_ALIGN_TO_COLOR,  // ��ȶ��뵽��ɫ
	COLOR_ALIGN_TO_DEPTH   // ��ɫ���뵽���
}_align;

/*ĳһ�����ά����*/
typedef struct {
	short W;
	short H;
	unsigned short D;
}Point3D;


class RealSenseD435 {
public:
	RealSenseD435();
	~RealSenseD435();
	/*******************    ������Կ���D435    ********************/
	void property_Control(ControlD435 controlD435, int NewValue);

	/***********************    ����D435    ***********************/
	void start();

	/***********************    ���ƴ���    ***********************/
	void controlWindows_depth(int &near, int &far);


	/***********************    �������    ***********************/
	void colorInit(RealSenseColorType ColorType);//��ʼ����ɫ���
	void depthInit(RealSenseDepthType DepthType);//��ʼ�����

  

	/***********************    ���ݸ���    ***********************/
	void updateFrame();                   // ����֡����    
	void updateColor(_align mode = NOUSE);// ���²�ɫ��
	void updateDepth(_align mode = NOUSE);// ���������


	/***********************    ��ȡ��Ϣ    ***********************/
	void get_colorImage(Mat &colorImage);					       // ��ȡ��ɫͼ
    void get_depth_Mat_8UC1(Mat&depthImage);
	void get_depth_Mat_8UC1(Mat &depthImage, int near, int far);   // ��ȡ��λ��ͨ�����ͼ
	void get_depth_color_map(Mat &depthImage);				       // ��ȡ���ӻ������ͼ��
	void get_Coordinate();										   // ������ά����
	void point2D_to_Point3D(cv::Point2i point2D, Point3D &point3D);// ���ĳһ�����ά����
	

	// �������Ը����������д���
	rs2::frame depth_frames; // ��ȡ���ͼ�� frame

private:


	/* �ýṹ���¼��ɫ����ͷͼ�� ѹ����ʽ ֡�� �ֱ��� �Լ�������ʾ��ͼ������*/
	typedef struct {
		int             MaType;  // Mat ���������
		rs2_format format_type;  // ����ͷͼ��ѹ����ʽ
		int				 width;  // ���
		int				height;  // �߶�
		int                fps;  // ֡��
	}ColorCfg;

	/* �ýṹ���¼���ͼ��� ֡�� �ֱ���*/
	typedef struct {
		int	 width;  // ���
		int	height;  // �߶�
		int    fps;  // ֡��
	}DepthCfg;
	
	/* ����ö�ٶ�Ӧ�Ŀ��Ʋ��� */
	void  ProduceColorCfg(RealSenseColorType Colortype, ColorCfg &_ColorCfg);
	
	/* ����ö�ٶ�Ӧ�Ŀ��Ʋ��� */
	void  ProduceDepthCfg(RealSenseDepthType Colortype, DepthCfg &_DepthCfg);

private:
	/************************     D435���Կ������     ************************/
	// ������ D435 ������ 0 : Stereo Module 1 : RGB Camera
	std::vector<rs2::sensor> list_of_sensors;
	/**************************************************************************/
	rs2::pipeline		     pipe; // �豸�ܵ�
	rs2::config			      cfg; // �����齨
	rs2::frameset          frames; 
	rs2::pipeline_profile profile;
	rs2::frame       color_frames; // ��ȡ��ɫͼ��� frame
	

	int  i, j;            // ѭ������
	int  ColorImage_type; // ��ɫͼ�������
	int  Dwidth;          // ���ͼ���
	_align Coordinate_align;
	// �洢��ά����Ľṹ��
	const vertex* tex_vertex;

	// һϵ�б�־λ��������
	bool Flg_depth_align_to_color_get_coordinate;// ��־���ͼӳ�䵽�˲�ɫ
	bool Flg_InitColor;							 // �����ɫͼ��־
	bool Flg_InitDepth;							 // �������ͼ��־

};
#endif


