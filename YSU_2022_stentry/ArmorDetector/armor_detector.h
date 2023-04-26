#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H

#include "Main/headfiles.h"//
#include "Kalman/kalman.h"
#include "SVM/svm.h"
#include "DeepLearning/yolov5.h"


#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))
class ArmorDetector
{
public:
    friend cv::VideoWriter;
    ArmorDetector();
    vector<cv::Point2d>& DetectObjectArmor();
    void Yolov2Res();
    void InitArmor();
    void LoadImage(cv::Mat &frame);
    void ScreenArmor();
    void ClearAll();
    void Show();
    void baocun();
    void PerspectiveTransformation();
    cv::RotatedRect boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right);
    cv::VideoWriter writer;

private:

    Yolov5* yolov5_detector_;                                       // yolov5 detector.

    std::vector<DetectRect> detect_res_armor_;                      // res rects

    cv::Mat src_image_;                                             // Source image.
    cv::Mat warpPerspective_dst;                                    // cnn????
    cv::Point2f Perspective_Transformation_src[4];                  // ????
    cv::Point2f Perspective_Transformation_dst[4];                  // ????
    cv::Point2f lu, ld, ru, rd;                                     // 四个角点

    std::vector<cv::RotatedRect> match_armors_;                     // Match to armor plate set.
    std::vector<cv::Point2d> target_armor_point_set;                // 

    int index1;                                                     //????
    int fps;                                                        // 帧率

    std::vector<int> object_num;                                    // 历史目标数量
    cv::Ptr<cv::ml::SVM> svm;
    int blue_color_threshold;
    int red_color_threshold;
    int max_g_dConArea;
    int min_g_dConArea;
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
    static vector<cv::RotatedRect> record_history_arr;
    vector<int> record_history_arr_num;

    float Kalman_Q,Kalman_R;
    vector<unique_ptr<Kalman_t>> p_kal;

    unique_ptr<YSU_SVM> p_svm;
    cv::Mat src_image_copy; //????????????????
    cv::Mat matsvm;


};



#endif // ARMORDETECTOR_H


