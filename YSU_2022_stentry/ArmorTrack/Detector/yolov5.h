/**
 * @anchor 可莉不知道哦
 * @brief 模型结构：Yolox，参考沈航的模型，之后自己打算改一个face模型，
 * 		输入信息[1, 3, 416, 416]，代码内部自动resize，并且映射到原始图像，
 * 		输出信息[1, 3549, 21]，3549个锚框。
 * 		锚框分为三种：grid = {8, 16, 32}，分别对于框的大小，
 * 		每个框有21个参数：
 * 			0~7：装甲板四个点位置； 8：物体置信度； 9-11：颜色置信度； 12-20:9个类别；
 * 		类别分布：
 * 			idx            name
 * 			12：		   哨兵
 * 			13：           英雄
 * 			14：           工程
 * 			15：           3号步兵
 * 			16：           4号步兵
 * 			17：           5号步兵
 * 			18：           前哨站
 * 			19：           基地
 * 			20：
*/

#ifndef YOLOV5
#define YOLOV5


#include "Main/headfiles.h"

// #define DEBUG // 调试模式

using namespace std;
using namespace cv;
using namespace cv::dnn;
using namespace InferenceEngine;


// yolov5_detector

struct DetectRect{
    cv::Point min_point;
    cv::Point max_point;
    cv::Rect rect;
    std::vector<cv::Point2f> points;
    cv::Point cen_p;
    int class_id;
    float class_p;
    int color_id;
    float color_p;
};

class Yolov5{
    private:

    cv::Mat m_src_image;
    cv::Mat m_src_copy_image;
    cv::Mat blob_image;
    int m_input_height;                                          // 模型要求输入的高度，一般是[640, 640]咱们的模型为[416, 416]
    int m_input_width;                                           // 模型要求输入的宽度
    std::string image_info_name;                                 // 图片输入模型的id
    std::string m_xml_path;                                      // 模型xml文件的路径
    std::string m_bin_path;                                      // 模型bin文件的路径

    InferenceEngine::Core m_ie;
    InferenceEngine::InputsDataMap m_input_info;                 // 输入信息
    InferenceEngine::OutputsDataMap m_output_info;               // 输出信息
    InferenceEngine::ExecutableNetwork m_executable_network;     // 训练好了的模型

    float scale_x;                                               // x方向上resize的尺度
    float scale_y;                                               // y方向上resize的尺度
    float max_scale;                                             // 图像resize的尺度，用来映射到原来图像
    float threshold;                                             // 置信度阈值标准

    int class_num;                                               // 模型输出的类别数量
    int color_num;                                               // 模型输出的颜色种类数量

    std::vector<DetectRect> res_rects;                           // 结果Rect的存储容器

    std::string * class_idx_map;                                 // 类别名称对应的索引值
    public:

    Yolov5();                           //
    Yolov5(std::string xml_path, std::string bin_path, int input_weight, int input_height);

    void init_yolov5_detector();        // init detector class

    vector<DetectRect>& detect_yolov5(cv::Mat& src_);   // detect function

    void show_res();

    private:

    std::vector<DetectRect>& infer2res(cv::Mat& src_);      // Mat yuchuli

    void image_pre_processing(cv::Mat& src_);

    void read_network();                // read network to class

    bool is_allready();                 // judge if is allready

    float get_IOU(cv::Point);

    void draw_res();

    void clear_work();
};


#endif // YOLOV5_DETECTOR YOLOV5
