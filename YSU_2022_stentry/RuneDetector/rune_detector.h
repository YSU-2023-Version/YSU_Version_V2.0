#ifndef RUNEDETECTOR_H
#define RUNEDETECTOR_H

#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>
#include<future>

using namespace std;
using namespace cv;



class RuneDetector
{public:
    RuneDetector();
    void getShootAim(const Mat &src,double time,std::promise<Point2f> &shoot);


private:
    Point2f out;
    void readFromXML();
    void preDelBuff();
    bool searchOfAim();
    void checkBuffMode();
    void getforecastAim();
    double Lagrange(const vector<double>& X, const vector<double>& Y, double x);

    Mat src_, dst;
    RotatedRect Aim, R;			//选定
    Point2f aim, r;				//"选定"的中点

    struct record {
    public:
        record();
        record(Point2f aim_, Point2f r_, double time_);

        double operator-(const record& a);//this.angle-a.angle。return：-180~180
        void print();
        Point2f aim, r;
        //Vec2f angle_vector;//第一位表示x方向，第二位表示y方向
        double distance;//半径
        double angle;//角度
        double time;
    };
    record history[30];



    int i;										//帧
    int R_SON_CONTOURS_NUM, AIM_SON_CONTOURS_NUM;//子轮廓数量阈值
    int NEIGHBORHOOD_SCALE_AIM;					//扇叶邻域检测边长
    float MIN_R_CENTER, MAX_R_CENTER;			//R所在图像中心范围限制
    float DELAY_TIME;							//表示预测需要预测多少秒以后的击打点（秒）。
    double D_VELOCITY_SAME_MAX;					//同速检测上限(像素/秒)。当V1-V2<=D_VELOCITY_SAME_MAX时，认为V1=V2。
    double AREA_LIMIT_MIN;						//面积小于该值的前景被认为是噪点。（单位：src.rows）

    enum {
        PRETREAT_HSV=0,
        PRETREAT_BGR=1,
    };
    int pretreatMode;//图像预处理模式
    enum {
        RUNE_DETECTOR_IS_RED = 0,
        RUNE_DETECTOR_IS_BLUE=1,
    };
    int color_aim;//能量机关颜色。只有图像预处理模式选为BGR时，该选项有效。
    enum {
        LARGE_BUFF_MODE=0,
        SMALL_BUFF_MODE=1,
        WAIT_TO_CHECK=2,
    };
    int buff_mode;//能量机关模式
};


#endif // RUNEDETECTOR_H
