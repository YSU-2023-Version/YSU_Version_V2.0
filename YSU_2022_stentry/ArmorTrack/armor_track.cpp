/**
 * @author 可莉不知道哦
 * @brief 测试预测，感觉预测可以做的更好，所以重新开了一个类用于研究，顺便把程序的解耦合程度提高一下
 * @date 2023/4/30
*/
#include "ArmorTrack/armor_track.h"

ArmorTracker::ArmorTracker(){
    max_len = 10;
}

ArmorTracker::ArmorTracker(int max_len){ // 先默认10
    this->max_len = max_len;
}


void ArmorTracker::add_to_tracker(DetectRect src_rect_){
    this->history_detect_rects_.push_back(src_rect_);
}


void ArmorTracker::update(){
    float temp_score = 1000; // 初始分数
    float area = history_detect_rects_[0].area;
    temp_score += area;
    for(auto item_rects : this->history_detect_rects_){

    }
    this->score_ = temp_score;
}


cv::Point2f ArmorTracker::get_res_point(){

}



void ArmorTracker::clear_work(){

}


float ArmorTracker::get_res_score(){
    return score_;
}
