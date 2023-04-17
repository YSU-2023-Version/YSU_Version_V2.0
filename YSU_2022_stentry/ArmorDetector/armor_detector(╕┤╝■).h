#ifndef ARMORDETECTOR_H
#define ARMORDETECTOR_H

#include "Main/headfiles.h"//

#define POINT_DIST(p1,p2) std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y))
class ArmorDetector
{
public:
    friend VideoWriter;
    ArmorDetector();
    vector<Point2d>& DetectObjectArmor();
    void InitArmor();
    void LoadImage(Mat frame);
    void PretreatImage();
    void DetectLightBar();
    void DetectArmor();
    void ScreenArmor();
    void ClearAll();
    void Show();
    void baocun();
    void PerspectiveTransformation();
    cv::RotatedRect boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right);
    VideoWriter writer;
    //bool RotatedRectsort(const cv::RotatedRect &a1,const cv::RotatedRect &a2);

private:
    Mat src_image_; // Source image.
    Mat src_image_copy; //用于透视变换输出抗划线干扰的图像
    Mat thre_image_; // Preprocessed image.
    Mat warpPerspective_mat=Mat::zeros(3,3,CV_32FC1);//透视变换矩阵

    Mat warpPerspective_dst;//cnn代入图像
    Point2f Perspective_Transformation_src[4];//透视变换
    Point2f Perspective_Transformation_dst[4];//透视变换
    Point2f lu, ld, ru, rd;



    std::vector<cv::RotatedRect> light_bars_; // Light strip set.
    std::vector<cv::RotatedRect> match_armors_; // Match to armor plate set.
    std::vector<cv::Point2d> target_armor_point_set;

    int index1; //存图下标
    int fps;
\

    std::vector<int> object_num;//装甲板对应的数字
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

    static float hero_zjb_ratio;
    float two_light_strips_angle_sub;
    float two_light_strips_ratio_min;

    float objRect_angle_max;
    float height_width_ratio_min;
    float height_width_ratio_max;








    //float record_max_angle;

};



#endif // ARMORDETECTOR_H


