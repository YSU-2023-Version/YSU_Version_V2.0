/**
 * @brief 本文件armor_detector.cpp是目标装甲板识别的代码，截止到2023.5.2为之，添加了深度学习模块，具体为直接调用yolov5.h中的算法类实现对模型的调用。
 *          其次就是继承了原先代码的对识别后的四点进行后续处理的代码。
 * @attention ArmorDetector内部某些成员变量可能已经弃用没来得及清理
*/

#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H

#include "Main/headfiles.h"
#include "Kalman/kalman.h"
#include "SVM/svm.h"
#include "DeepLearning/yolov5.h"

/**
 * 一个存RotatedRect一个存vector<Point2d>,一个方便用于筛选装甲板，一个方便用于仿射变换
*/
struct Rect_VectorPoint {
    /* data */
    std::vector<cv::Point2f> points;
    cv::RotatedRect rect;
};



#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))
class ArmorDetector
{
public:
    friend cv::VideoWriter;
    ArmorDetector();
    vector<cv::Point2f>& DetectObjectArmor();
    void Yolov2Res();
    void InitArmor();
    void LoadImage(cv::Mat &frame);
    void ScreenArmor();
    void ClearAll();
    void Show();
    void Show(double y_err,double p_err);
    void baocun();
    void PerspectiveTransformation();
    cv::RotatedRect boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right);
    cv::VideoWriter writer;

private:

    Yolov5* yolov5_detector_;                                                     // yolox识别器
    std::vector<DetectRect> detect_res_armor_;                                    // yolox模型识别到的框，所有信息整合到了DetectRect结构体
    cv::Mat src_image_;                                                           // 输入图像
    cv::Mat warpPerspective_dst;                                                  // 预处理输出图像
    cv::Point2f Perspective_Transformation_src[4];                                // 
    cv::Point2f Perspective_Transformation_dst[4];                                // ????
    cv::Point2f lu, ld, ru, rd;                                                   // 结果的四个点坐标

    std::vector<DetectRect> match_armors_;                                        // 匹配到的装甲板，结构体中有rect和points，points是原始四点模型结果
    std::vector<cv::Point2f> target_armor_point_set;                              // 四点（具体用于?）

    int index1;                                                                   // ????
    int fps;                                                                      // 帧率

    std::vector<int> object_num;                                                  // 历史目标数量
    cv::Ptr<cv::ml::SVM> svm;                                                     // svm相关（弃用）
    int blue_color_threshold;                                                     // 蓝色通道二值图
    int red_color_threshold;                                                      // 红色通道二值图
    int max_g_dConArea;                                                           // 最大允许区域？
    int min_g_dConArea;                                                           // 最小允许区域？
    float max_angle_abs;
    float max_center_y;
    float get_data_fps;

    string imgname;
    string aviname;
    int f = 1;
    int enemy_color;


    float contour_area_min;
    float contour_area_max;
    float contour_width_height_ratio_max;
    float contour_width_height_ratio_min;
    float contour_angle_max;//-90~contour_angle_max
    float contour_div_rect;

    int hero_priority;
    static float hero_zjb_ratio_min;
    static float hero_zjb_ratio_max;
    static float score_of_hero;
    static float score_of_area;
    static float score_of_last;


    float two_light_strips_angle_sub;
    float two_light_strips_ratio_min;

    float objRect_angle_max;
    float height_width_ratio_min;
    float height_width_ratio_max;

    static int record_history_num;
    float wu_cha_yun_xu;
    static vector<DetectRect> record_history_arr;
    vector<int> record_history_arr_num;

    float Kalman_Q,Kalman_R;
    vector<unique_ptr<Kalman>> p_kal;

    unique_ptr<YSU_SVM> p_svm;
    cv::Mat src_image_copy; //????????????????
    cv::Mat matsvm;
    cv::Mat warpPerspective_mat=cv::Mat::zeros(3,3,CV_32FC1);//??????


};



#endif // ARMORDETECTOR_H


