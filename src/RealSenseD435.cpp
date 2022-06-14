#include "RealSenseD435.h"


/***************************************************************************************************************************
****************************************************************************************************************************
**************************************                                             *****************************************
**************************************             �ò���Ϊ���к���                *****************************************
**************************************                                             *****************************************
****************************************************************************************************************************
****************************************************************************************************************************/
RealSenseD435::RealSenseD435() {

    // ��ʼ�����ɱ�־λ

    // �ñ�־ΪΪfalse��ʾ�����Ĭ�ϸ�ʽ��ȡͼ��
    Flg_InitColor = false;
    Flg_InitDepth = false;

    Flg_depth_align_to_color_get_coordinate = false;


/************************��һ�ν���ı�����������й�****************************/
    rs2::context ctx;
    // �豸�б� ���ĸ����ǵ������ӵ�����ͷ����
    rs2::device_list list_of_devices = ctx.query_devices();
    // ���б���ѡ��������豸
    rs2::device choosed_device = list_of_devices[0];
    // ��ȡ�������б�
    list_of_sensors = choosed_device.query_sensors();
/*******************************************************************************/
}


RealSenseD435::~RealSenseD435() {
    // Terminate the pipeline
    pipe.stop();
    printf("~~~Release RealSenseD435()!\n");
}

/************************************************************************
*������; property_Control
*����  : ���������������D435��һЩ���������ع� ���� ��ƽ��ȵ�
*���� 1; ControlD435 ö�����Ͳ���  ��ʾҪ������Щ����
*���� 2; �������õ�ֵ  ע������Χ
************************************************************************/
void RealSenseD435::property_Control(ControlD435 controlD435, int NewValue) {

    rs2::sensor choosed_sensor; // ѡ��Ĵ�����
    rs2_option option_type;     // ѡ���Ҫ���Ƶ���������

    int type_value = static_cast<int>(controlD435);

    if (type_value < static_cast<int>(ControlD435::Stereo_Module))
        choosed_sensor = list_of_sensors[1]; // 1 : RGB Camera
    else {
        choosed_sensor = list_of_sensors[0]; // 0 : Stereo Module
        // �� Stereo Module �йص�ö���Ǵ� 100��ʼ��
        type_value = type_value - 100;
    }

    // ���Լ������ö��ת���ɹٷ�������ö��
    option_type = static_cast<rs2_option>(type_value);

    // �豸���ܲ�֧����һѡ��(��ֹ�������)
    if (!choosed_sensor.supports(option_type))
        cout << " This option is not supported by this sensor" << endl;

    // ��ȡ������ԵĿ����÷�Χ
    rs2::option_range range = choosed_sensor.get_option_range(option_type);

    // ��Χ�������Сֵ
    int maximum = range.max;
    int minimum = range.min;

    if (NewValue > maximum)NewValue = maximum;
    if (NewValue < minimum)NewValue = minimum;

    // �ı�����Ե�ֵ
    choosed_sensor.set_option(option_type, NewValue);
}

/************************************************************************
*������; controlWindows_depth
*����  : �Բ�ɫ����ͷ��������
*���� 1: �������
*���� 2: ��Զ����
************************************************************************/
void RealSenseD435::controlWindows_depth(int &near, int &far) {
    namedWindow("depth_control");
    createTrackbar("near:", "depth_control", &near, 8000, NULL);
    createTrackbar("far :", "depth_control", &far, 8000, NULL);
}

/************************************************************************
*������; colorInit
*����  : �Բ�ɫ����ͷ��������
*����  : RealSenseColorType ���͵�ö��
************************************************************************/
void RealSenseD435::colorInit(RealSenseColorType ColorType) {

    static ColorCfg _ColorCfg; // ������Ʋ���

    // ��ȡ���Ƹ�ʽ
    ProduceColorCfg(ColorType, _ColorCfg);

    // Color   width   height  ѹ����ʽ  fps
    cfg.enable_stream(RS2_STREAM_COLOR, _ColorCfg.width, _ColorCfg.height, _ColorCfg.format_type, _ColorCfg.fps);

    // ��������  �ֱ���
    ColorImage_type = _ColorCfg.MaType;

    Flg_InitColor = true;

}

/************************************************************************
*������; depthInit
*����  : ��������ͼ��������
*����  : RealSenseDepthType ���͵�ö��
************************************************************************/
void RealSenseD435::depthInit(RealSenseDepthType DepthType) {

    static DepthCfg _DepthCfg;// ������Ʋ���

    // ��ȡ���Ƹ�ʽ
    ProduceDepthCfg(DepthType, _DepthCfg);

    // Depth   width   height  ѹ����ʽ  fps
    cfg.enable_stream(RS2_STREAM_DEPTH, _DepthCfg.width, _DepthCfg.height, RS2_FORMAT_Z16, _DepthCfg.fps);

    // ȷ�϶���������
    Flg_InitDepth = true;
}

/************************************************************************
*������; start
*����  : ����d435
************************************************************************/
void RealSenseD435::start() {

    // ��������ͷ
    // ��ʼ����ͷ�����������������һ���� �ڳ����е����������ᱨ��

    // û�����ù���ȺͲ�ɫ����ͷ
    if (Flg_InitColor == false && Flg_InitDepth == false)
        pipe.start();
    else {
        // �����������
        if (Flg_InitColor == false && Flg_InitDepth == true)
            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);

        // �������˲�ɫ
        if (Flg_InitColor == true && Flg_InitDepth == false)
            cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 90);

        pipe.start(cfg);
    }


}

/************************************************************************
*������; updateFrame
*����  : �ȴ���һ֡
************************************************************************/
void RealSenseD435::updateFrame() {
    // Wait for next set of frames
    frames = pipe.wait_for_frames();

}

/************************************************************************
*������; updateFrame
*����  : �ȴ���һ֡
************************************************************************/
void RealSenseD435::updateColor(_align mode) {

    if (mode == NOUSE) // ��ʹ�ö���
        color_frames = frames.get_color_frame();

    else if (mode == COLOR_ALIGN_TO_DEPTH) {  // ��ɫ���뵽���
        ;
    } else {
        cerr << "updateColor()  �����������" << endl;
        while (true);
    }
    Flg_depth_align_to_color_get_coordinate = true;
}

/************************************************************************
*������; updateFrame
*����  : �ȴ���һ֡
************************************************************************/
void RealSenseD435::updateDepth(_align mode) {

    if (mode == NOUSE) // ��ʹ�ö���
        depth_frames = frames.get_depth_frame();

    else if (mode == DEPTH_ALIGN_TO_COLOR) { // ��ȶ��뵽��ɫ

        /*
        ʹ�ö���������ʱ��  ʹ�õķֱ���Խ������ʱ��Խ��
        */

        if (Flg_InitDepth == false) {
            cerr << "updateDepth()  δ����������" << endl;
            while (true);
        }

        // �����ɫ����ȿ�ܲ�����ȿ������ɫ��ܶ���
        static rs2::align align(rs2_stream::RS2_STREAM_COLOR);
        static rs2::frameset aligned_frames; // ����֡

        aligned_frames = align.process(frames);// ����

        depth_frames = aligned_frames.get_depth_frame();
    } else {
        cerr << "updateDepth()  �����������" << endl;
        while (true);
    }

    Coordinate_align = mode;
}

/************************************************************************
*������; get_colorImage
*����  : ��ȡ��ɫͼ��
*����  : ����Ĳ�ɫͼ��
************************************************************************/
void RealSenseD435::get_colorImage(Mat &colorImage) {

    // ��ѯ֡��С����Ⱥ͸߶ȣ�
    const int w = color_frames.as<video_frame>().get_width();
    const int h = color_frames.as<video_frame>().get_height();

    // ��֡��Ĭ�ϸ�ʽת��ΪMat��ʽ
    if (Flg_InitColor == true)
        colorImage = Mat(Size(w, h), ColorImage_type, (void *) color_frames.get_data());
    else
        colorImage = Mat(Size(w, h), CV_8UC3, (void *) color_frames.get_data());

}

void RealSenseD435::get_depth_Mat_8UC1(Mat &depthImage) {
    // ��ѯ֡��С����Ⱥ͸߶ȣ�
    const int w = depth_frames.as<video_frame>().get_width();
    const int h = depth_frames.as<video_frame>().get_height();


    depthImage = Mat(Size(w, h), CV_16UC1, (void *) depth_frames.get_data());
}


/************************************************************************
*������; get_depth_color_map
*����  : ��ȡ��λ��ͨ�����ͼ
*���� 1: ������ͼ��
*���� 2: ��ʾ��������� Ĭ��    0
*���� 3: ��ʾ����Զ���� Ĭ�� 8000
************************************************************************/
void RealSenseD435::get_depth_Mat_8UC1(Mat &depthImage, int near, int far) {

    // ��ѯ֡��С����Ⱥ͸߶ȣ�
    const int w = depth_frames.as<video_frame>().get_width();
    const int h = depth_frames.as<video_frame>().get_height();

    static Mat dm;
    dm = Mat(Size(w, h), CV_16UC1, (void *) depth_frames.get_data());

    static Mat depthMat(Size(w, h), CV_8UC1);// ���ӻ���Ķ������ͼ
    static unsigned short *dmData;
    static unsigned short dmDataSrc;
    static uchar *data;


    // �ı�ͼ����� ʹͼ����ӻ�
    for (i = 0; i < h; i++) {
        dmData = dm.ptr<unsigned short>(i);
        data = depthMat.ptr<uchar>(i);
        for (j = 0; j < w; j++) {
            dmDataSrc = dmData[j];
            if (dmDataSrc > far || dmDataSrc < near)data[j] = 0;
            else data[j] = (uchar) (dmDataSrc >> 5);
        }
    }

    // ��ȡͼ��
    depthImage = depthMat;

}

/************************************************************************
*������; get_depth_color_map
*����  : ��ȡ���ӻ������ͼ��
*����  : ������ͼ��
************************************************************************/
void RealSenseD435::get_depth_color_map(Mat &depthImage) {

    //Ϊ������ݵĿ��ӻ���ʾ�����ɫ��
    static colorizer color_map;

    // ��ѯ֡��С����Ⱥ͸߶ȣ�
    const int w = depth_frames.as<video_frame>().get_width();
    const int h = depth_frames.as<video_frame>().get_height();

    static frame color_depth_frames;
    //color_depth_frames = color_map(depth_frames);

    // ����ɫ����������д���OpenCV��С��w��h����OpenCV����
    depthImage = Mat(Size(w, h), CV_8UC3, (void *) color_depth_frames.get_data());

}

/************************************************************************
*������; get_Coordinate
*����  : ��ȡ��ά����
************************************************************************/
void RealSenseD435::get_Coordinate() {

    // ����pointcloud�������ڼ���pointcloud������ӳ��
    // �������õ���󱣳ֳ־ã��������ǾͿ�����֡�½�ʱ��ʾ������
    static rs2::pointcloud pc;
    static rs2::points points;

    // ����pointcloud������ӳ��
    points = pc.calculate(depth_frames);

    if (Coordinate_align == DEPTH_ALIGN_TO_COLOR) {
        if (Flg_depth_align_to_color_get_coordinate == false) {
            cout << "error: ʹ��get_Coordinate()ǰ��û�л�ȡ��ɫ֡����ֹ�������ȡ֡˳��" << endl;
            while (1);
        }
        Flg_depth_align_to_color_get_coordinate = true;
        pc.map_to(color_frames);
    }


    // ��ȡʵ�ʵ���ά����
    tex_vertex = points.get_vertices();

    // ��¼��� ��ȡ��ά�����ʱ���õ�
    Dwidth = depth_frames.as<video_frame>().get_width();
}

/************************************************************************
*������; point2D_to_Point3D
*����  : ���ĳһ�����ά����
*���� 1: ����Ķ��������ͼ��
*���� 2: ��ʾ��������� Ĭ��     0
************************************************************************/
void RealSenseD435::point2D_to_Point3D(cv::Point2i point2D, Point3D &point3D) {

    point3D.W = (short) (tex_vertex[point2D.y * Dwidth + point2D.x].x * 1000);
    point3D.H = (short) (tex_vertex[point2D.y * Dwidth + point2D.x].y * 1000);
    point3D.D = (unsigned short) (tex_vertex[point2D.y * Dwidth + point2D.x].z * 1000);
}










/***************************************************************************************************************************
****************************************************************************************************************************
**************************************                                             *****************************************
**************************************             �ò���Ϊ˽�к���                *****************************************
**************************************                                             *****************************************
****************************************************************************************************************************
****************************************************************************************************************************/
/************************************************************************
*������; ProduceColorCfg
*����  : ���� RealSenseColorType ö�ٶ�Ӧ�� D435 ��ɫ�������
*���� 1: RealSenseColorType ����ö��
*���� 2: ColorCfg���ͽṹ�� �ں��������
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
*������; ProduceDepthCfg
*����  : ���� RealSenseDepthType ö�ٶ�Ӧ�� D435 ���ͼ����
*���� 1: RealSenseDepthType ����ö��
*���� 2: ColorCfg���ͽṹ�� �ں��������
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
