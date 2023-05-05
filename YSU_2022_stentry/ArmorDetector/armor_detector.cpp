#include "Main/headfiles.h"
#include "ArmorDetector/armor_detector.h"
#include "Pose/angle_solver.h"
#include  <time.h>

#define DEBUG 1

//由于sort函数的第三参数不属于类内，因此需要使用全局变量，全局变量初始化区
float ArmorDetector:: hero_zjb_ratio_min=3.9;
float ArmorDetector::hero_zjb_ratio_max=4.1;
int  ArmorDetector:: record_history_num=5;
float ArmorDetector::score_of_hero=1;
float ArmorDetector::score_of_area=1;
float ArmorDetector::score_of_last=1;

vector<Rect_VectorPoint> ArmorDetector::record_history_arr;


float get_angle(const cv::Point2f &a,const cv::Point2f &b)
{
    float dx=a.x-b.x;
    if(fabs(dx)<0.1) return -90;
    float dy=a.y-b.y;
    return atanf(dy/dx)*180/CV_PI;
}

float get_dis ( const cv::Point2f &a,const cv::Point2f &b)
{
    return sqrtf(pow(a.x-b.x,2)+pow(a.y-b.y,2));
}

ArmorDetector::ArmorDetector()://  1
    p_svm(std::make_unique<YSU_SVM>()),
    blue_color_threshold(120),
    red_color_threshold(0),
    max_g_dConArea(10000),
    min_g_dConArea(200),
    max_angle_abs(20),
    max_center_y(40),
    get_data_fps(50)
{}


/**
 * @brief 初始化代码，包含：xml文件读取和深度学习模型初始化
*/
void ArmorDetector::InitArmor()
{
    //cout<<"装甲板检测初始化成功！"<<endl;
    std::cout << "armor_detector init begin" << std::endl;

    std::string file_path="../xml_path/armor_limited.xml";
    cv::FileStorage fr;
    fr.open(file_path,cv::FileStorage::READ);
    while(!fr.isOpened()){
        std::cout<<"armor_xml floading failed..." << std::endl;
        fr=cv::FileStorage(file_path,cv::FileStorage::READ);
        fr.open(file_path,cv::FileStorage::READ);
    }
    // 部分参数可能已经弃用
    fr["contour_area_min"]>>contour_area_min;
    fr["contour_area_max"]>>contour_area_max;

    fr["contour_width_height_ratio_max"]>>contour_width_height_ratio_max;
    fr["contour_width_height_ratio_min"]>>contour_width_height_ratio_min;

    fr["contour_angle_max"]>>contour_angle_max;
    fr["contour_div_rect"]>>contour_div_rect;

    fr["hero_priority"]>>hero_priority;
    fr["hero_zjb_ratio_max"]>>ArmorDetector:: hero_zjb_ratio_max;
    fr["hero_zjb_ratio_min"]>>ArmorDetector:: hero_zjb_ratio_min;
    fr["score_of_hero"]>>ArmorDetector::score_of_hero;
    fr["score_of_area"]>>ArmorDetector::score_of_area;
    fr["score_of_last"]>>ArmorDetector::score_of_last;
    fr["record_history_num"]>>record_history_num;

    fr["two_light_strips_angle_sub"]>>two_light_strips_angle_sub;
    fr["two_light_strips_ratio_min"]>>two_light_strips_ratio_min;

    fr["objRect_angle_max"]>>objRect_angle_max;
    fr["height_width_ratio_max"]>>height_width_ratio_max;
    fr["height_width_ratio_min"]>>height_width_ratio_min;
    fr["Kalman_Q"]>>Kalman_Q;
    fr["Kalman_R"]>>Kalman_R;

    fr["wu_cha_yun_xu"]>>wu_cha_yun_xu;
    fr.release();
    std::cout<<"armor_xml loading finished" << std::endl;

    p_svm->InitSVM();
    // 深度学习部分，稳定可用
    std::cout << "deep learning yolox model init" << std::endl;
    record_history_arr.clear();
    // 初始化深度学习算法类
    this->yolov5_detector_ = new Yolov5("../model/model/opt-0527-001.xml", "../model/model/opt-0527-001.bin", 416, 416); // 创建yolov5detector对象
    this->yolov5_detector_->init_yolov5_detector(); // init yolov5 detector 模型加载部分
    // 这个部分可能会消耗比较多的时间, 但是是正常现象
    std::cout << "armor_detector init finished" << std::endl;

}

void ArmorDetector::LoadImage(cv::Mat &frame)
{
    src_image_=frame;
    src_image_copy=frame.clone();
}

/**
 * @brief 原代码取最左边的矩形，改进代码是取面积最大的+预测
 *
 * @attention 在调用识别函数之后使用，默认是筛选最大的装甲板，也可以优先历史帧，具体模式需要修改xml文件中的hero_priority参数
 *
 * @param void
 * @return void
 */
void ArmorDetector::ScreenArmor(){
#ifdef DEBUG
    cout<<"match_armors_.size():"<<match_armors_.size()<<endl;
#endif

    record_history_arr_num.emplace_back(match_armors_.size());//tuxiangzhong zhenshi jiance daode zhuangjiaban shuliang
    while(record_history_arr_num.size()>record_history_num)record_history_arr_num.erase(record_history_arr_num.begin());

    //de-装甲板闪烁。如果当前帧比上record_history_num帧中的wu_cha_yun_xu的装甲板数量少，则将上一帧结果加入当前帧
    int i=0;//
    for(vector<int>::iterator it=record_history_arr_num.begin();it!=record_history_arr_num.end();it++)
    {if(match_armors_.size()<*it)i++;}
    if(i>wu_cha_yun_xu*record_history_num) match_armors_.emplace_back(*record_history_arr.end()); // 短暂的丢失目标仍然打击

    if(match_armors_.size()>0)//如果当前帧没有检测到目标
    {
        if(match_armors_.size()!=1)//如果当前帧检测到目标不唯一，则根据多目标优先算法进行排序。
        {
            if(hero_priority==0){//多因素混合打分
                sort(match_armors_.begin(),match_armors_.end(),[](const Rect_VectorPoint & rect1, const Rect_VectorPoint & rect2) {
                    float score1=1,score2=1;
                    //da zjb::score+=
                    float ratio1=(rect1.rect.size.width/rect1.rect.size.height);
                    float ratio2=(rect2.rect.size.width/rect2.rect.size.height);
                    if(ratio1<hero_zjb_ratio_max&&ratio1>hero_zjb_ratio_min)score1+=score_of_hero;
                    if(ratio2<hero_zjb_ratio_max&&ratio2>hero_zjb_ratio_min)score2+=score_of_hero;

                    rect1.rect.size.area()>rect2.rect.size.area()?score1+=score_of_area:score2+score_of_area;
                    (get_dis((record_history_arr.end()-1)->rect.center,rect1.rect.center)<get_dis((record_history_arr.end()-1)->rect.center,rect2.rect.center))?score1+=score_of_last:score2+=score_of_last;

                    return score1>score2;
                });
            }else if(hero_priority==1){//优先历史帧
                sort(match_armors_.begin(),match_armors_.end(),[](const Rect_VectorPoint & rect1, const Rect_VectorPoint & rect2)
                {
                    int score1=0,score2=0;
                    for(int tmp=record_history_arr.size();tmp>0;tmp--)
                    {(get_dis(record_history_arr[tmp-1].rect.center,rect1.rect.center))<(get_dis(record_history_arr[tmp-1].rect.center,rect2.rect.center))?score1++:score2++;}
                    return score1>score2;
                });
            }else if(hero_priority==2){//第一优先大装甲板，再二优先历史帧 
                sort(match_armors_.begin(),match_armors_.end(),[](const Rect_VectorPoint & rect1, const Rect_VectorPoint & rect2)
                {
                    float ratio1=rect1.rect.size.width/rect1.rect.size.height;
                    float ratio2=rect2.rect.size.width/rect2.rect.size.height;
                    if(ratio1>4.2)return false;
                    if(ratio2>4.2)return true;
                    if(ratio1>3.0&&ratio2<3.0)return true;
                    if(ratio2>3.0&&ratio1<3.0)return false;

                    int score1=0,score2=0;
                    for(int tmp=record_history_arr.size();tmp>0;tmp--)
                    {(get_dis(record_history_arr[tmp-1].rect.center,rect1.rect.center))<(get_dis(record_history_arr[tmp-1].rect.center,rect2.rect.center))?score1++:score2++;}
                    return score1>score2;
                });
            }else{//优先最大面积
                sort(match_armors_.begin(),match_armors_.end(),[](const Rect_VectorPoint & rect1, const Rect_VectorPoint & rect2) {
                    return rect1.rect.size.area() > rect2.rect.size.area();
                });
            }
        }

        int id=0;// 获取到最好的那个
        Point2f vertices[4];

        for(int i = 0;i < 4;i++) vertices[i] = match_armors_[id].points[i];

        while(1){
            sort(vertices, vertices + 4, [](const Point2f & p1, const Point2f & p2){return p1.x < p2.x; });
            if (vertices[0].y < vertices[1].y){
                lu = vertices[0];
                ld = vertices[1];
            }else{
                lu = vertices[1];
                ld = vertices[0];
            }

            if (vertices[2].y < vertices[3].y)	{
                ru = vertices[2];
                rd = vertices[3];
            }
            else {
                ru = vertices[3];
                rd = vertices[2];
            }

            PerspectiveTransformation();//透视变换


#ifdef DEBUG
            line(src_image_,lu,ld,Scalar(255,255,255),3);
            line(src_image_,ld,rd,Scalar(255,255,255),3);
            line(src_image_,rd,ru,Scalar(255,255,255),3);
            line(src_image_,ru,lu,Scalar(255,255,255),3);
            imshow("src",src_image_);
            waitKey(1);
#endif
            // float temp=p_svm->getNum(warpPerspective_dst)-2;
            // 短时间内调用两次会有内存报错，非常奇怪，怀疑是寄存器未清
            // 空，但是没找到解决方案。
            if(fabs(p_svm->getNum(warpPerspective_dst)-2)<0.1){//如果当前锁定装甲板为2（工程），则重新判断下一装甲板（如果存在多装甲板）
                if((1+id)>=match_armors_.size())break;
                else match_armors_[++id].rect.points(vertices);
            }else{
                break;
            }
        }
//        for(int i=0;i<4;i++)
//       {
//           vertices[i].x=p_kal[2*i]->KalmanFilter(vertices[i].x);
//           vertices[i].y=p_kal[2*i+1]->KalmanFilter(vertices[i].y);
//       }
        //记录输出结果，用于历史帧判断
        record_history_arr.emplace_back(match_armors_[id]);
        while(record_history_arr.size()>record_history_num)record_history_arr.erase(record_history_arr.begin());

        target_armor_point_set.clear();
        target_armor_point_set.push_back(lu);
        target_armor_point_set.push_back(ru);
        target_armor_point_set.push_back(rd);
        target_armor_point_set.push_back(ld);
    }
    else
    {
        target_armor_point_set.clear();
        target_armor_point_set.push_back(Point2d(0,0));
        target_armor_point_set.push_back(Point2d(0,0));
        target_armor_point_set.push_back(Point2d(0,0));
        target_armor_point_set.push_back(Point2d(0,0));
    }
}

/**
 * @brief 清空历史工作
*/
void ArmorDetector::ClearAll(){
    match_armors_.clear();
}

/**
 * @brief 展示结果
*/
void ArmorDetector::Show(){
#ifdef DEBUG
  // imshow("dst",thre_image_);
    imshow("src",src_image_);

//    imshow("warpPerspective",warpPerspective_dst);

    //if语句在imshow前src看不到绿色框(可以准确识别)，放在后面即使src显示绿色框，帧率调整好后录制效果也算满意(绿色框不会频繁显示）
    //cout<<src_image_.cols<<endl;
   //cout<<src_image_.rows<<endl;
    waitKey(1);
#endif
}

/**
 * @author 可莉不知道哦
 * @brief ArmorDetector::Yolov2Res
 * @details yolov5识别器，处理完成之后的结果将存到match_armors_的vector中
 */
void ArmorDetector::Yolov2Res(){
    this->detect_res_armor_ = this->yolov5_detector_->detect_yolov5(src_image_copy); // 模型推理到结果
    for(auto item : this->detect_res_armor_){ // 遍历识别到的结果
        Rect_VectorPoint temp_rect_points_;
        temp_rect_points_.points = item.points;
        temp_rect_points_.rect = cv::minAreaRect(item.points);
        this->match_armors_.push_back(temp_rect_points_); // 将rect转换成RotatedRect
    }
}


/**
 * @author 可莉不知道哦
 * @return vector<Point2d>& 
 * @param void
 * @brief 在初始化之后，每一次加载图片之后就执行一次识别过程，返回最终中心点坐标
*/
vector<Point2d>& ArmorDetector::DetectObjectArmor(){
    // old detector 老视觉识别已经弃用，现在用的深度学习识别
    Yolov2Res();                   // 调用yolo模型
    ScreenArmor();                 // 使用原来的装甲板筛选代码
    ClearAll();                    // 清除历史工作数据

    return target_armor_point_set; // 返回中心点的信息
}

/**
 * @brief 保存图片，在训练赛中可以使用这个函数记录图片用于标注数据集
*/
void ArmorDetector::baocun()
{
    imgname = "./debug_shortcut/"+to_string(f++) + ".jpg"; //输出文件名为 f.jpg, 保留在工程文件夹中
    imwrite(imgname, src_image_);
}

cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right){
    // 这个函数是用来将左右边的灯条拟合成一个目标旋转矩形，没有考虑角度
    const Point & pl = left.center, & pr = right.center;
    Point2f center = (pl + pr) / 2.0;
    //    cv::Size2f wh_l = left.size;
    //    cv::Size2f wh_r = right.size;
    // 这里的目标矩形的height是之前灯柱的width
    double width_l = MIN(left.size.width, left.size.height);
    double width_r = MIN(right.size.width, right.size.height);
    double height_l = MAX(left.size.width, left.size.height);
    double height_r = MAX(right.size.width, right.size.height);
    float width = POINT_DIST(pl, pr) - (width_l + width_r) / 2.0;
    float height = std::max(height_l, height_r);
    //float height = (wh_l.height + wh_r.height) / 2.0;
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
}

void ArmorDetector::PerspectiveTransformation(){
    Perspective_Transformation_src[0].x=lu.x+0.1*(ru.x-lu.x);
    Perspective_Transformation_src[0].y=lu.y-0.45*(ld.y-lu.y);
    Perspective_Transformation_src[1].x=ru.x-0.1*(ru.x-lu.x);
    Perspective_Transformation_src[1].y=ru.y-0.45*(rd.y-ru.y);
    Perspective_Transformation_src[2].x=rd.x-0.1*(rd.x-ld.x);
    Perspective_Transformation_src[2].y=rd.y+0.45*(rd.y-ru.y);
    Perspective_Transformation_src[3].x=ld.x+0.1*(rd.x-ld.x);
    Perspective_Transformation_src[3].y=ld.y+0.45*(ld.y-lu.y);

    Perspective_Transformation_dst[0]=Point2d(0,0);
    Perspective_Transformation_dst[1]=Point2d(64,0);
    Perspective_Transformation_dst[2]=Point2d(64,64);
    Perspective_Transformation_dst[3]=Point2d(0,64);
    warpPerspective_dst = Mat::zeros(64, 64, src_image_copy.type());
  //  warpPerspective_dst = Mat::zeros(100, 100, src_image_copy.type());
    warpPerspective_mat = getPerspectiveTransform(Perspective_Transformation_src, Perspective_Transformation_dst);
#ifdef DEBUG
    warpPerspective(src_image_copy, warpPerspective_dst, warpPerspective_mat, warpPerspective_dst.size());
//    circle(src_image_, Perspective_Transformation_src[0], 3, Scalar(0, 255, 0), -1);  // 画半径为1的圆(画点）
//    circle(src_image_, Perspective_Transformation_src[1], 3, Scalar(0, 255, 0), -1);  // 画半径为1的圆(画点）
//    circle(src_image_, Perspective_Transformation_src[2], 3, Scalar(0, 255, 0), -1);  // 画半径为1的圆(画点）
//    circle(src_image_, Perspective_Transformation_src[3], 3, Scalar(0, 255, 0), -1);  // 画半径为1的圆(画点）

    warpPerspective(src_image_, warpPerspective_dst, warpPerspective_mat, warpPerspective_dst.size());
#endif
}


