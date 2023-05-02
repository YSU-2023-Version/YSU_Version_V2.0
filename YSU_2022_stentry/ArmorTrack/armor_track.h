/**
 * @author 可莉不知道哦
 * @brief 测试预测，感觉预测可以做的更好，所以重新开了一个类用于研究
 * @date 2023/4/30
*/

/*        颜色宏定义        */
#define COLOR_BLUE 9   // 蓝
#define COLOR_RED 10   // 红
#define COLOR_DEAD 11  // 死亡
/*        类别宏定义        */
#define ROB_HERO 13
#define ROB_INFANTRY_1 14
#define ROB_INFANTRY_2 15
#define ROB_INFANTRY_3 16
/*     看看之后需不需要     */

#ifndef ARMOR_TRACK
#define ARMOR_TRACK
#include "Main/headfiles.h"
#include "Detector/yolov5.h"
#include "Predictor/armor_predictor.h"
#include "Main/headfiles.h"

struct Armor
{
    int id;                              // 识别码
    int color;                           // 颜色标识符
    int area;
    float conf;
    float class_p;
    string key;
    Point2f apex2d[4];
    Rect rect;
    RotatedRect rrect;
    Point2f center2d;
    Eigen::Vector3d center3d_cam;
    Eigen::Vector3d center3d_world;
    Eigen::Vector3d euler;
    Eigen::Vector3d predict;

    TargetType type;
    // DetectRect => Armor
    Armor(DetectRect result){
        this->area = result.rect.area();
        for(int i = 0;i < 4;i++){
            this->apex2d[i] = result.points[i];
        }
        this->rrect = result.rect;
    }
};


class ArmorTracker{
public:
    Armor prev_armor;                       // 上一次装甲板
    Armor last_armor;                       // 本次装甲板
    bool is_initialized;                    // 是否完成初始化
    int last_selected_timestamp;            // 该Tracker上次被选为目标tracker时间戳
    int prev_timestamp;                     // 上次装甲板时间戳
    int last_timestamp;                     // 本次装甲板时间戳
    int history_type_sum;                   // 历史次数之和
    int selected_cnt;                       // 该Tracker被选为目标tracker次数和
    const int max_history_len = 4;          // 历史信息队列最大长度
    float hit_score;                        // 该tracker可能作为目标的分数,由装甲板旋转角度,距离,面积大小决定
    float velocity;
    float radius;
    string key;
    
public:
    std::deque<Armor> history_info;         // 目标队列
    // 创建跟踪器
    ArmorTracker(Armor src, int src_timestamp);
    // 更新跟踪器
    void update(Armor src, int src_timestamp);
    // 计算分数，和面积、角度有关，如果面积太离谱就舍去
}

#endif // !ARMOR_TRACK
