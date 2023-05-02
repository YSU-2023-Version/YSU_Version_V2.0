/**
 * @author 可莉不知道哦
 * @brief 测试预测，感觉预测可以做的更好，所以重新开了一个类用于研究，顺便把程序的解耦合程度提高一下
 * @date 2023/4/30
*/
#include "ArmorTrack/armor_track.h"

/**
 * @brief 构造一个ArmorTracker对象
 * 
 * @param src Armor对象   src_timestamp 时间戳
 */
ArmorTracker::ArmorTracker(Armor src, int src_timestamp)
{
    last_armor = src;
    last_timestamp = src_timestamp;
    key = src.key;
    is_initialized = false;
    hit_score = 0;
    history_info.push_back(src);
}

/**
 * @brief 更新ArmorTracker对象
 * 
 * @param src Armor对象   src_timestamp 时间戳
 */
void ArmorTracker::update(Armor new_armor, int new_timestamp) {
    if (history_info.size() <= max_history_len){ // 如果size是小于最大长度，直接push
        history_info.push_back(new_armor);
    } else { // 如果是大于最大允许长度就先退队，再插入
        history_info.pop_front();
        history_info.push_back(new_armor);
    }

    is_initialized = true;
    prev_armor = last_armor;
    prev_timestamp = last_timestamp;
    last_armor = new_armor;
    last_timestamp = new_timestamp;

    // calcTargetScore(); // 后续再用
}