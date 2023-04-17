#include "Main/headfiles.h"
#include "ArmorDetector/armor_detector.h"
#include "Pose/angle_solver.h"
#include  <time.h>

// #define USE_OLD_DETECTOR // 使用老的detector（传统视觉）

#define DEBUG 1

//由于sort函数的第三参数不属于类内，因此需要使用全局变量，全局变量初始化区
float ArmorDetector:: hero_zjb_ratio_min=3.9;
float ArmorDetector::hero_zjb_ratio_max=4.1;
int  ArmorDetector:: record_history_num=5;
float ArmorDetector::score_of_hero=1;
float ArmorDetector::score_of_area=1;
float ArmorDetector::score_of_last=1;

vector<cv::RotatedRect> ArmorDetector::record_history_arr;


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



void ArmorDetector::InitArmor()
{
    //cout<<"装甲板检测初始化成功！"<<endl;
    cout<<"armor_detector init begin"<<endl;

         string file_path="../xml_path/armor_limited.xml";
         cv::FileStorage fr;
         fr.open(file_path,cv::FileStorage::READ);
         while(!fr.isOpened()){
               cout<<"armor_xml floading failed..."<<endl;
              fr=cv::FileStorage(file_path,cv::FileStorage::READ);
              fr.open(file_path,cv::FileStorage::READ);
         }

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
cout<<"armor_xml loading finished"<<endl;

         for(int i=0;i<8;i++){
             p_kal.emplace_back(std::make_unique<Kalman_t>());
             p_kal[i]->KalmanInit(Kalman_Q,Kalman_R);
         }

         p_svm->InitSVM();

        record_history_arr.clear();
#ifndef USE_OLD_DETECTOR
        this->yolov5_detector_ = new Yolov5("../model/model/opt-0527-001.xml", "../model/model/opt-0527-001.bin", 416, 416); // 创建yolov5detector对象
        this->yolov5_detector_->init_yolov5_detector(); // init yolov5 detector 模型加载部分
#endif  // 这个部分可能会消耗比较多的时间, 但是是正常现象
        cout<<"armor_detector init finished"<<endl;
}

void ArmorDetector::LoadImage(cv::Mat &frame)
{
    src_image_=frame;
    src_image_copy=frame.clone();
}

void ArmorDetector::PretreatImage(){
/*
经过尝试，相机曝光度选择人肉眼勉强可以看清楚数字的状态，使用以下参数进行预处理，可以有效规避误识别。
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5));
    cv::Mat element2 = getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3));

    if(ENEMY_COLOR_IS_RED)
    {//Red
        cv::Mat thre_whole;
        cv::Mat diff[3];//分离后的单通道图像,填函数的输出数组或者输出的vector容器
        cv::split(src_image_,diff); //分离通道 BGR
        cvtColor(src_image_,thre_whole,CV_BGR2GRAY);
        threshold(thre_whole,thre_whole,80,255,cv::THRESH_BINARY);
        addWeighted(diff[2],1,diff[0],-1,0,thre_image_);
        threshold(thre_image_,thre_image_,100,255,cv::THRESH_BINARY);
        dilate(thre_image_,thre_image_,element);
        thre_image_ = thre_whole & thre_image_;
        dilate(thre_image_,thre_image_,element2);
    }
    else
    {//Blue
        cv::Mat thre_whole;
        cv::Mat diff[3];//分离后的单通道图像
        cv::split(src_image_,diff); //分离通道 BGR
        cvtColor(src_image_,thre_whole,CV_BGR2GRAY);
        threshold(thre_whole,thre_whole,170,255,cv::THRESH_BINARY);
        addWeighted(diff[0],1,diff[2],-0.70,0,thre_image_);
        cv::threshold(thre_image_,thre_image_,130,255,cv::THRESH_BINARY);
        dilate(thre_image_,thre_image_,element);
        thre_image_ = thre_whole & thre_image_;
        dilate(thre_image_,thre_image_,element2);
    }
*/
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5));
    cv::Mat element2 = getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3,3));

    if(ENEMY_COLOR_IS_RED)
    {//Red
        cv::Mat thre_whole;
        cv::Mat diff[3];//分离后的单通道图像,填函数的输出数组或者输出的vector容器
        cv::split(src_image_,diff); //分离通道 BGR
        cvtColor(src_image_,thre_whole,CV_BGR2GRAY);
        threshold(thre_whole,thre_whole,70,255,cv::THRESH_BINARY);


//        cv::imshow("thre_whole",thre_whole);


         addWeighted(diff[2],1,diff[0],-1,0,thre_image_);
        //subtract(diff[2],diff[0],thre_image_); //  2

        threshold(thre_image_,thre_image_,90,255,cv::THRESH_BINARY);
//        cv::imshow("thre_image_",thre_image_);

        dilate(thre_image_,thre_image_,element);
        thre_image_ = thre_whole & thre_image_;
        dilate(thre_image_,thre_image_,element2);
    }//  3
    else
    {//Blue
        cv::Mat thre_whole;
        cv::Mat diff[3];//分离后的单通道图像
        cv::split(src_image_,diff); //分离通道 BGR
        cvtColor(src_image_,thre_whole,CV_BGR2GRAY);
        threshold(thre_whole,thre_whole,170,255,cv::THRESH_BINARY);



        addWeighted(diff[0],1,diff[2],-0.70,0,thre_image_);
//        subtract(diff[0],diff[2],thre_image_);

        cv::threshold(thre_image_,thre_image_,130,255,cv::THRESH_BINARY);
        dilate(thre_image_,thre_image_,element);
        thre_image_ = thre_whole & thre_image_;
        dilate(thre_image_,thre_image_,element2);

    }
//     cv::waitKey(1);
}

//筛选灯条
void ArmorDetector::DetectLightBar(){
    std::vector<std::vector<cv::Point> > contours;  //轮廓集
    std::vector<cv::Vec4i> hierarchy;//
    cv::findContours(thre_image_,contours,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    if(contours.size()>1)
    {
        std::vector <std::vector<cv::Point> >::iterator iter; //轮廓类型迭代器
        for ( iter = contours.begin(); iter != contours.end() ; iter++ )
        {
            double g_dConArea = contourArea(*iter);
//            putText(src_image_,to_string(g_dConArea),Point2f(10,50),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));
      //面积限制
       if (g_dConArea>contour_area_max || g_dConArea<contour_area_min){continue;}

            cv::RotatedRect rect = cv::minAreaRect(*iter); //用最小矩形包起来每个轮廓便于后续处理
            float max_radio=cv::max(rect.size.height/rect.size.width,rect.size.width/rect.size.height);

//        putText(src_image_,"area_rotia:"+to_string(g_dConArea/rect.size.area()),Point2f(10,50),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));
      //轮廓面积/外接矩形面积，可用于限制ROI区域规则程度，装甲板正对的时候结果约为0.85
      if((g_dConArea/rect.size.area())<contour_div_rect)continue;

//           putText(src_image_,"max_radio"+to_string(max_radio),Point2f(10,50),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));
       //装甲板长宽比限制，下限限制了左右摇头程度，上限限制了俯仰角程度
       if(max_radio>contour_width_height_ratio_max){continue;}
       if(max_radio<contour_width_height_ratio_min){continue;}

        //旋转角限制，该值给的低可以筛掉车辆45度方向的虚假“装甲板”，但是也容易导致侧倾装甲板识别不到。
       if(rect.angle>(-90 + contour_angle_max)&&rect.angle<(0 - contour_angle_max)){continue;}

//            putText(src_image_,"width"+to_string(rect.size.width),Point2f(10,50),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));
//            putText(src_image_,"heigh"+to_string(rect.size.height),Point2f(10,100),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));
//            putText(src_image_,"angle"+to_string(rect.angle),Point2f(10,150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));
        //去除横着的灯条（竖着的“装甲板”）
       if((rect.angle<(-90+contour_angle_max)&&(rect.size.width<rect.size.height))
         ||((rect.angle>(0-contour_angle_max)&&(rect.size.width>rect.size.height)))){continue;}



//            Point2f pt[4];
//            rect.points(pt);
//            float dx_0to2=fabs(pt[2].x-pt[0].x);
//            float dy_0to2=fabs(pt[2].y-pt[0].y);

            light_bars_.push_back(rect);
        }
    }
}

//tong guo deng tiao pi pei zhuang jia ban
void ArmorDetector::DetectArmor(){

    if(light_bars_.size() > 1)
    {//按照从左到右排序，只检测相邻的灯条
        sort(light_bars_.begin(),light_bars_.end(),[](const RotatedRect & rect1, const RotatedRect & rect2) { return rect1.center.x < rect2.center.x; });
        for(size_t i=0;i<light_bars_.size()-1;i++)
        {
                //如果这里可以当作可以用的装甲板，那么把两个灯条定义为装甲板然后放进另一个数组里
                //rotatedRects[i]和rotatedRects[j]
                //首先通过平行度和center_y进行筛选，找到候选的装甲板
                //但是一个灯条只能构成一个装甲板，怎么衡量那个装甲板最需要这个灯条，但是如果判断错误了，那么可能会把中间图案的装甲板加进来了。
                //所以如果有可能的话，暂时先保存着这两个装甲板

                float leftLength = std::max(light_bars_[i].size.width,light_bars_[i].size.height);
                float rightLength = std::max(light_bars_[i+1].size.width,light_bars_[i+1].size.height);
//                putText(src_image_,"ratio:"+std::to_string(cv::max(leftLength,rightLength) / cv::min(leftLength,rightLength)),Point2f(10,50),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));
            //两灯条长度比例限制，可以筛掉一些不在同一平面的灯条。
            if((cv::max(leftLength,rightLength) / cv::min(leftLength,rightLength) )> two_light_strips_ratio_min){continue;}

                Point2f lp[4],rp[4];
                light_bars_[i].points(lp);
                light_bars_[i+1].points(rp);
                float leftAngle,rightAngle,angle_sub;
                if(leftLength==light_bars_[i].size.width){leftAngle=get_angle(lp[0],lp[3]);}
                else {leftAngle=get_angle(lp[0],lp[1]);}
                if(rightLength==light_bars_[i+1].size.width){rightAngle=get_angle(rp[0],rp[3]);}
                else {rightAngle=get_angle(rp[0],rp[1]);}
                if(leftAngle<(-90+two_light_strips_angle_sub)){leftAngle=fabs(leftAngle);}
                if(rightAngle<(-90+two_light_strips_angle_sub)){rightAngle=fabs(rightAngle);}
                angle_sub=fabs(leftAngle-rightAngle);
//                cout<<"angle_sub"<<angle_sub<<endl;
           //两灯条角度差值限制，该条件可用于筛掉车辆45度方向的虚假装甲板，建议7~10度
           if(angle_sub > two_light_strips_angle_sub){continue;}//jiao du cha zhi


                RotatedRect obj_rect = boundingRRect(light_bars_[i],light_bars_[i+1]);

//长宽比限制，旋转角度限制
          if((obj_rect.size.width/obj_rect.size.height)> height_width_ratio_max) {continue;}
          if((obj_rect.size.width/obj_rect.size.height)<height_width_ratio_min){continue;}
          if(obj_rect.angle>objRect_angle_max||obj_rect.angle<0-objRect_angle_max){continue;}
          match_armors_.push_back(obj_rect);
 //putText(src_image_,to_string(obj_rect.size.width/obj_rect.size.height),obj_rect.center,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255));

#ifdef DEBUG
              Point2f p[4];
              obj_rect.points(p);
//                cv::putText(src_image_,"left:"+to_string(leftAngle),p[0],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255));
//                    putText(src_image_,"right:"+to_string(rightAngle),p[2],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255));
//              putText(src_image_,"raito:"+to_string(obj_rect.size.width/obj_rect.size.height),p[0],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));
//              putText(src_image_,"angle_sub:"+to_string(angle_sub),obj_rect.center,cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255));
//            putText(src_image_,"p0:",p[0],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));
//            putText(src_image_,"p1:",p[1],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));
//            putText(src_image_,"p2:",p[2],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));
//            putText(src_image_,"p3:",p[3],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));

//            putText(src_image_,"obj_rect.angle"+to_string(obj_rect.angle),p[3],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));
//            putText(src_image_,"obj_rect.width"+to_string(obj_rect.size.width),Point2f(10,150),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));
//            putText(src_image_,"obj_rect.heigth"+to_string(obj_rect.size.height),Point2f(10,200),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));



//            putText(src_image_,"obj.area():"+std::to_string(obj_rect.size.area()),Point2d(10,60),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,0,255));

//            //debug
//            obj_rect.points(p);
//            std::cout << "angle_sub: " << angle_sub << std::endl;
//            std::cout << "two_light_strips_ratio: " << cv::min(leftLength,rightLength) / cv::max(leftLength,rightLength)<< std::endl;
//            std::cout << "height_width_ratio: " << dis / averageLength<< std::endl;
//            line(src_image_, p[0], p[1], cv::Scalar(0, 255,0), 1);
//            line(src_image_, p[1], p[2], cv::Scalar(0, 255,0), 1);
//            line(src_image_, p[2], p[3], cv::Scalar(0, 255,0), 1);
//            line(src_image_, p[3], p[0], cv::Scalar(0, 255,0), 1);

            //putText(src_image_,"angle_sub:"+std::to_string(angle_sub),Point2d(10,10),cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(0,0,255));

//            //line(src_image_, leftCenter, rightCenter, cv::Scalar(0, 255, 100), 2);
#endif
        }
    }
//     putText(src_image_,"record_max_angle:"+std::to_string(record_max_angle),Point2d(10,40),cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(0,0,255));
}

/**
 * @brief 原代码取最左边的矩形，改进代码是取面积最大的+预测
 *
 * @attention [调用函数需要注意的地方]
 *
 * @param [参数1] [参数说明]
 * @param [参数2] [参数说明]
 * @return [返回值] [返回值说明]
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
    if(i>wu_cha_yun_xu*record_history_num)match_armors_.emplace_back(*record_history_arr.end());//duanzan de diushi mu biao rengran jida

    if(match_armors_.size()>0)//如果当前帧没有检测到目标
    {
        if(match_armors_.size()!=1)//如果当前帧检测到目标不唯一，则根据多目标优先算法进行排序。
        {
           if(hero_priority==0){//多因素混合打分
                sort(match_armors_.begin(),match_armors_.end(),[](const RotatedRect & rect1, const RotatedRect & rect2) {
                  float score1=1,score2=1;
                  //da zjb::score+=
                  float ratio1=(rect1.size.width/rect1.size.height);
                  float ratio2=(rect2.size.width/rect2.size.height);
                  if(ratio1<hero_zjb_ratio_max&&ratio1>hero_zjb_ratio_min)score1+=score_of_hero;
                  if(ratio2<hero_zjb_ratio_max&&ratio2>hero_zjb_ratio_min)score2+=score_of_hero;

                  rect1.size.area()>rect2.size.area()?score1+=score_of_area:score2+score_of_area;
                  (get_dis((record_history_arr.end()-1)->center,rect1.center)<get_dis((record_history_arr.end()-1)->center,rect2.center))?score1+=score_of_last:score2+=score_of_last;

                  return score1>score2;
                });
           }else if(hero_priority==1){//优先历史帧
                sort(match_armors_.begin(),match_armors_.end(),[](const RotatedRect & rect1, const RotatedRect & rect2)
                {
                    int score1=0,score2=0;
                    for(int tmp=record_history_arr.size();tmp>0;tmp--)
                    {(get_dis(record_history_arr[tmp-1].center,rect1.center))<(get_dis(record_history_arr[tmp-1].center,rect2.center))?score1++:score2++;}
                    return score1>score2;
                });
           }else if(hero_priority==2){//第一优先大装甲板，再二优先历史帧 
                sort(match_armors_.begin(),match_armors_.end(),[](const RotatedRect & rect1, const RotatedRect & rect2)
               {
                  float ratio1=rect1.size.width/rect1.size.height;
                  float ratio2=rect2.size.width/rect2.size.height;
                  if(ratio1>4.2)return false;
                  if(ratio2>4.2)return true;
                  if(ratio1>3.0&&ratio2<3.0)return true;
                  if(ratio2>3.0&&ratio1<3.0)return false;

                  int score1=0,score2=0;
                  for(int tmp=record_history_arr.size();tmp>0;tmp--)
                  {(get_dis(record_history_arr[tmp-1].center,rect1.center))<(get_dis(record_history_arr[tmp-1].center,rect2.center))?score1++:score2++;}
                  return score1>score2;
               });
           }else{//优先最大面积
                sort(match_armors_.begin(),match_armors_.end(),[](const RotatedRect & rect1, const RotatedRect & rect2) {
                   return rect1.size.area() > rect2.size.area();
               });
           }
        }
       
        int id=0;
        Point2f vertices[4];
        match_armors_[id].points(vertices);//?1

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
          //float temp=p_svm->getNum(warpPerspective_dst)-2;
          //短时间内调用两次会有内存报错，非常奇怪，怀疑是寄存器未清
       // 空，但是没找到解决方案。
          if(fabs(p_svm->getNum(warpPerspective_dst)-2)<0.1)
            {//如果当前锁定装甲板为2（工程），则重新判断下一装甲板（如果存在多装甲板）
                if((1+id)>=match_armors_.size())break;
                else match_armors_[++id].points(vertices);
            }else
            {
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
void ArmorDetector::ClearAll(){
    light_bars_.clear();
    match_armors_.clear();
}

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
 * @brief ArmorDetector::Yolov2Res
 * @details yolov5识别器，处理完成之后的结果将存到match_armors_的vector中
 */
void ArmorDetector::Yolov2Res(){
    this->detect_res_armor_ = this->yolov5_detector_->detect_yolov5(src_image_copy); // 模型推理到结果
    for(auto item : this->detect_res_armor_){ // 遍历识别到的结果
        cv::RotatedRect r_rect_ = cv::minAreaRect(item.points);
        this->match_armors_.push_back(r_rect_); // 将rect转换成RotatedRect
    }
}



vector<Point2d>& ArmorDetector::DetectObjectArmor(){
    // old detector
#ifdef USE_OLD_DETECTOR
    PretreatImage();
    DetectLightBar();
    DetectArmor();
    // new detector  ==> yolov5 deeplearning
#else
    Yolov2Res();
#endif
    ScreenArmor();
    ClearAll();

    return target_armor_point_set;
}

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


