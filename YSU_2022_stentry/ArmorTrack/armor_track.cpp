/**
 * @author FoPluto
 * @brief 测试预测，感觉预测可以做的更好，所以重新开了一个类用于研究，顺便把程序的解耦合程度提高一下
 * @date 2023/4/30
*/
#include "ArmorTrack/armor_track.h"

ArmorTracker::ArmorTracker(){
    max_len = 10;
    hit_score = 0;

}

ArmorTracker::ArmorTracker(int max_len){ // 先默认10
    this->max_len = max_len;
    hit_score = 0;
    is_initialized = false;
}


void ArmorTracker::add_to_tracker(DetectRect& src_rect_){
    if(this.history_detect_rects_.size() <= max_len){
        this->history_detect_rects_.push_back(src_rect_);
    } else {
        history_detect_rects_.pop_front();
        history_detect_rects_.push_back(src_rect_);
    }
    is_initialized = true;
    this->last_armor = this->prev_armor;
    this->prev_armor = src_rect_;
    calcTargetScore();
}


void ArmorTracker::calcTargetScore(){
    std::vector<cv::Point2f> points;
    float rotate_angle;
    RotatedRect rotated_rect = last_armor.r_rect;
    int area = last_armor.area;
    if (rotated_rect.size.width > rotated_rect.size.height)
        rotate_angle = rotated_rect.angle;
    else
        rotate_angle = 90 - rotated_rect.angle;
    //计算分数
    //使用log函数压缩角度权值范围
    hit_score = log(0.15 * (90 - rotate_angle) + 10) * (last_armor.area);
    #ifdef DEBUG
    std::cout << last_armor.class_name << "跟踪器分数:" << hit_score << std::endl;
    #endif // DEBUG
}


float ArmorTracker::get_res_score(){
    return hit_score;
}
