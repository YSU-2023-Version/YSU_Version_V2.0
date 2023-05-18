/**
 * @author FoPluto
 * @brief 测试预测，感觉预测可以做的更好，所以重新开了一个类用于研究
 * @date 2023/4/30
*/
#define DEBUG
/*        颜色宏定义        */
#define COLOR_BLUE 9   // 蓝
#define COLOR_RED 10   // 红
#define COLOR_DEAD 11  // 死亡

#ifndef ARMOR_TRACKER_H
#define ARMOR_TRACKER_H
#include "Main/headfiles.h"
#include "DeepLearning/yolov5.h"

class ArmorTracker{
public:
    bool is_initialized;
    DetectRect last_armor;
    DetectRect prev_armor;

private:
    // 最大长度
    unsigned int max_len;
    // 历史识别结果存放
    std::deque<DetectRect> history_detect_rects_;
    // 分数
    float hit_score;
    // 是否被初始化
    bool is_initialized;
public:
    // 计算分数，和面积、角度有关，如果面积太离谱就舍去
    ArmorTracker();
    ArmorTracker(int max_len);
    // 将识别结果添加到Tracker中
    void add_to_tracker(DetectRect& src_rect_);
    // 计算Tracker分数
    void calcTargetScore();
    // 获取当前跟踪器的分数
    float get_res_score();
};

#endif // !ARMOR_TRACKER_H
