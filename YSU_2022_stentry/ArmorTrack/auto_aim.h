#include "armor_track.h"
#include "Main/headfiles.h"

#ifndef AUTOAIM_H
#define AUTOAIM_H
/**
 * @author 可莉不知道哦
 * @brief 参考沈航代码添加的模块，在TrackDemo分支中，用于开发跟踪，反小陀螺模块，很有可能需要世界坐标系
*/
class AutoAim{
    private:

    Yolov5 armor_detector;                  // 装甲板识别器
    std::vector<ArmorTracker> trackers;     // 跟踪器容器

    Mat src_image_;                         // 输入图像

    public:

    AutoAim(){

    }

    void init_auto_aim();

    void detect_image2res(cv::Mat& src_image_);

    std::vector<cv::Point2f>& detect_track_armor(cv::Mat src_image_);

    
};

#endif // !AUTOAIM_H
