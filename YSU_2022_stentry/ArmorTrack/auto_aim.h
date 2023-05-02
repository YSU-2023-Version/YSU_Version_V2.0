#include "armor_track.h"
#include "Main/headfiles.h"

#ifndef AUTOAIM_H
#define AUTOAIM_H
/**
 * @author 可莉不知道哦
 * @brief 参考沈航代码添加的模块，在TrackDemo分支中，用于开发跟踪，反小陀螺模块
*/
class AutoAim{
    private:

    std::vector<ArmorTracker> trackers;     // 跟踪器容器
    

    public:

    AutoAim(){

    }
}

#endif // !AUTOAIM_H
