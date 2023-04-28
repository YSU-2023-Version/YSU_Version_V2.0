#include "Main/headfiles.h"
#include "ArmorDetector/armor_detector.h"
#include "Pose/angle_solver.h"
#include<time.h>

#define DEBUG

float ArmorDetector:: hero_zjb_ratio=0;

ArmorDetector::ArmorDetector()://  1
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

         string file_path="/home/robomaster/qtcreater_workspace/YSU_2021_Infantrybb22.5.20/build-YSU_2021_Infantry-Desktop_Qt_5_14_1_GCC_64bit-Debug/armor_limited.xml";
         FileStorage fr;
         fr.open(file_path,FileStorage::READ);
         while(!fr.isOpened()){
               cout<<"xml floading failed..."<<endl;
              fr=FileStorage(file_path,FileStorage::READ);
         }

        fr["contour_area_min"]>>contour_area_min;
        fr["contour_area_max"]>>contour_area_max;

        fr["contour_width_height_ratio_max"]>>contour_width_height_ratio_max;
        fr["contour_width_height_ratio_min"]>>contour_width_height_ratio_min;

        fr["contour_angle_max"]>>contour_angle_max;
        fr["contour_div_rect"]>>contour_div_rect;

        fr["hero_zjb_ratio"]>>ArmorDetector:: hero_zjb_ratio;
        fr["two_light_strips_angle_sub"]>>two_light_strips_angle_sub;
        fr["two_light_strips_ratio_min"]>>two_light_strips_ratio_min;

        fr["objRect_angle_max"]>>objRect_angle_max;
        fr["height_width_ratio_max"]>>height_width_ratio_max;
        fr["height_width_ratio_min"]>>height_width_ratio_min;


        fr.release();
      //  record_max_angle=10;


}

void ArmorDetector::LoadImage(Mat frame)
{
    src_image_=frame;
    src_image_copy=frame.clone();
}

void ArmorDetector::PretreatImage(){
    //    cv::Mat diff[3];//分离后的单通道图像
    //    cv::split(src,diff); //分离通道 BGR
    //    diff[0]=diff[0]-diff[1]*0.4-diff[2]*0.4;
    //    thre_image_=diff[0]*1.3;
    //    double maxValue_gary,minValue_gary;
    //    minMaxLoc(thre_image_,&minValue_gary,&maxValue_gary,nullptr,nullptr);
    //    threshold(thre_image_,thre_image_,(maxValue_gary-minValue_gary)*0.3,maxValue_gary,CV_THRESH_BINARY);
    //    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(1,1));
    //    cv::erode(thre_image_,thre_image_,element);
    //    cv::dilate(thre_image_,thre_image_,element);

    /*=========================================*/
    Mat element = getStructuringElement(MORPH_ELLIPSE,Size(7,7));
    //Mat element1 = getStructuringElement(MORPH_ELLIPSE,Size(5,5));
    Mat element2 = getStructuringElement(MORPH_ELLIPSE,Size(3,3));

    if(!ENEMY_COLOR_IS_RED)
    {
        Mat thre_whole;
        cv::Mat diff[3];//分离后的单通道图像,填函数的输出数组或者输出的vector容器
        cv::split(src_image_,diff); //分离通道 BGR
        cvtColor(src_image_,thre_whole,CV_BGR2GRAY);
        threshold(thre_whole,thre_whole,60,255,THRESH_BINARY);

        subtract(diff[2],diff[0],thre_image_);//  2
        threshold(thre_image_,thre_image_,40,255,THRESH_BINARY);
        dilate(thre_image_,thre_image_,element);
        thre_image_ = thre_whole & thre_image_;
        dilate(thre_image_,thre_image_,element2);
    }//  3
    else
    {
        Mat thre_whole;
        cv::Mat diff[3];//分离后的单通道图像
        cv::split(src_image_,diff); //分离通道 BGR
        cvtColor(src_image_,thre_whole,CV_BGR2GRAY);
        threshold(thre_whole,thre_whole,128,255,THRESH_BINARY);

        subtract(diff[0],diff[2],thre_image_);
        threshold(thre_image_,thre_image_,46,255,THRESH_BINARY);
        dilate(thre_image_,thre_image_,element);
        thre_image_ = thre_whole & thre_image_;
        dilate(thre_image_,thre_image_,element2);
    }
}

void ArmorDetector::DetectLightBar(){
    std::vector<std::vector<cv::Point> > contours;  //轮廓集
    std::vector<cv::Vec4i> hierarchy;//

    cv::findContours(thre_image_,contours,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
    //cv::drawContours(result,contours,-1,cv::Scalar(0),1);
    if(contours.size()>1)
    {
        std::vector <std::vector<cv::Point> >::iterator iter; //轮廓类型迭代器
        for ( iter = contours.begin(); iter != contours.end() ; iter++ )
        {
            double g_dConArea = contourArea(*iter);
           if (g_dConArea>contour_area_max || g_dConArea<contour_area_min) continue;

            cv::RotatedRect rect = cv::minAreaRect(*iter); //用最小矩形包起来每个轮廓便于后续处理
            float max_radio=cv::max(rect.size.height/rect.size.width,rect.size.width/rect.size.height);
#ifdef DEBUG
//             Point2f p[4];
//             rect.points(p);
//           putText(src_image_,"max_radio"+to_string(max_radio),p[0],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255));
//           putText(src_image_,"c_area"+to_string(contourArea(*iter)),p[1],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));
//           putText(src_image_,"contour_angle"+to_string(rect.angle),p[2],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));



#endif
            if((g_dConArea/rect.size.area())<contour_div_rect)//yueshu faguang quyu xingzhuang guize chengdu.
            if(max_radio>contour_width_height_ratio_max)continue;
            if(max_radio<contour_width_height_ratio_min)continue;

            //xia mian liang ge zhi neng yong yi ge
            if(rect.angle>(-90 + contour_angle_max)&&rect.angle<(0 - contour_angle_max))continue;
//            if(rect.angle>contour_angle_max||rect.angle<(0-contour_angle_max))continue;



//            Point2f pt[4];
//            rect.points(pt);
//            float dx_0to2=fabs(pt[2].x-pt[0].x);
//            float dy_0to2=fabs(pt[2].y-pt[0].y);
//            if(dx_0to2>dy_0to2)continue;
               if((rect.angle<(-90+contour_angle_max)&&(rect.size.width<rect.size.height))
                       ||((rect.angle<(0-contour_angle_max)&&(rect.size.width>rect.size.height)))){continue;}

            light_bars_.push_back(rect);
#ifdef DEBUG
//            line(src_image_,p[0],p[1],Scalar(255,255,255));
//            line(src_image_,p[2],p[1],Scalar(255,255,255));
//            line(src_image_,p[2],p[3],Scalar(255,255,255));
//            line(src_image_,p[3],p[0],Scalar(255,255,255));
//              putText(src_image_,"p0",p[0],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));
//              putText(src_image_,"p1",p[1],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));
//              putText(src_image_,"p2",p[2],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));
//              putText(src_image_,"p3",p[3],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));

//              putText(src_image_,"angle"+to_string(rect.angle),Point2f(p[2].x+50,p[2].y),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,0));
#endif

        }
    }
}

//tong guo deng tiao pi pei zhuang jia ban
void ArmorDetector::DetectArmor(){
    putText(src_image_,to_string(light_bars_.size()),Point2d(900,900),cv::FONT_HERSHEY_SIMPLEX,2,cv::Scalar(255,255,255));
    if(light_bars_.size() > 1)
    {
        sort(light_bars_.begin(),light_bars_.end(),[](const RotatedRect & rect1, const RotatedRect & rect2) { return rect1.center.x < rect2.center.x; });
        //应该cong i xiang hou遍历
        for(size_t i=0;i<light_bars_.size()-1;i++)
        {


            //如果这里可以当作可以用的装甲板，那么把两个灯条定义为装甲板然后放进另一个数组里
            //rotatedRects[i]和rotatedRects[j]
            //首先通过平行度和center_y进行筛选，找到候选的装甲板
            //但是一个灯条只能构成一个装甲板，怎么衡量那个装甲板最需要这个灯条，但是如果判断错误了，那么可能会把中间图案的装甲板加进来了。
            //所以如果有可能的话，暂时先保存着这两个装甲板
            cv::Point2f leftCenter=light_bars_[i].center;
            cv::Point2f rightCenter=light_bars_[i+1].center;
            //cv::Point2f averageCenter= cv::Point2f((leftCenter.x + rightCenter.x)/2.0f,(leftCenter.y + rightCenter.y)/2.0f) ;

                    float xDiff = std::fabs(leftCenter.x - rightCenter.x);
                    float yDiff = std::fabs(leftCenter.y - rightCenter.y);
                    float dis = std::sqrt(xDiff * xDiff + yDiff * yDiff); // 计算两灯条距离

                    float leftAngle = light_bars_[i].angle;
                    float rightAngle = light_bars_[i+1].angle;
                    float angle_sub = std::fabs(leftAngle-rightAngle); // 计算2灯条角度cha zhi绝对值

                    float leftLength = std::max(light_bars_[i].size.width,light_bars_[i].size.height);
                    float rightLength = std::max(light_bars_[i+1].size.width,light_bars_[i+1].size.height);
                    float averageLength = (leftLength + rightLength) / 2; // 计算高度平均值



//                 putText(src_image_,"limited:"+std::to_string(two_light_strips_angle_sub),Point2d(800,20),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255));
          if(angle_sub < 90.0f-two_light_strips_angle_sub && angle_sub > two_light_strips_angle_sub){continue;}//jiao du cha zhi
//                 putText(src_image_,"angle_sub:"+std::to_string(angle_sub),Point2d(10,20),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255));
           if(cv::min(leftLength,rightLength) / cv::max(leftLength,rightLength) < two_light_strips_ratio_min){continue;}
//                 putText(src_image_,"length_ratio:"+std::to_string(cv::min(leftLength,rightLength) / cv::max(leftLength,rightLength)),Point2d(10,40),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255));
//                 putText(src_image_,"l_w_ratio:"+std::to_string(dis/averageLength),Point2d(800,60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255));
//                 putText(src_image_,"l_w_ratio_min:"+std::to_string(height_width_ratio_min),Point2d(800,80),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255));
//                 putText(src_image_,"l_w_ratio_max:"+std::to_string(height_width_ratio_max),Point2d(800,100),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255));
          if((dis / averageLength) < height_width_ratio_min || (dis / averageLength )> height_width_ratio_max) {continue;}
//                 putText(src_image_,"l_w_ratio:"+std::to_string(dis/averageLength),Point2d(10,60),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255));

                Point2f left_points[4],right_points[4],p[4];
                light_bars_[i].points(left_points);
                light_bars_[i+1].points(right_points);


//                  putText(src_image_,"angle_obj:"+std::to_string(obj_rect.angle),p[2],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,255,255));

                RotatedRect obj_rect = boundingRRect(light_bars_[i],light_bars_[i+1]);
//                obj_rect.points(p);
//                float dx_0to2=fabs(p[2].x-p[0].x);
//                float dy_0to2=fabs(p[2].y-p[0].y);

          if(obj_rect.angle>objRect_angle_max||obj_rect.angle<0-objRect_angle_max)continue;


//          if(obj_rect.angle>(-90+objRect_angle_max)&&obj_rect.angle<(0-objRect_angle_max))continue;
//                 putText(src_image_,"dx1to3:"+std::to_string(dx_1to3),p[1],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));
//                 putText(src_image_,"dy1to3:"+std::to_string(dy_1to3),p[3],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));



//            putText(src_image_,"p0:",p[0],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));
//            putText(src_image_,"p1:",p[1],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));
//            putText(src_image_,"p2:",p[2],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));
//            putText(src_image_,"p3:",p[3],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));
//            putText(src_image_,"obj_rect.angle"+to_string(obj_rect.angle),Point2f(10,100),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,255,255));



//            if(dx_0to2<dy_0to2)continue;




            match_armors_.push_back(obj_rect);
            //putText(src_image_,"obj.area():"+std::to_string(obj_rect.size.area()),Point2d(10,60),cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(0,0,255));





            //debug
            obj_rect.points(p);
            std::cout << "angle_sub: " << angle_sub << std::endl;
            std::cout << "two_light_strips_ratio: " << cv::min(leftLength,rightLength) / cv::max(leftLength,rightLength)<< std::endl;
            std::cout << "height_width_ratio: " << dis / averageLength<< std::endl;
//            line(src_image_, p[0], p[1], cv::Scalar(255, 0, 255), 1);
//            line(src_image_, p[1], p[2], cv::Scalar(255, 0, 255), 1);
//            line(src_image_, p[2], p[3], cv::Scalar(255, 0, 255), 1);
//            line(src_image_, p[3], p[0], cv::Scalar(255, 0, 255), 1);
            //record_max_angle=cv::max(record_max_angle,angle_sub);
            //putText(src_image_,"angle_sub:"+std::to_string(angle_sub),Point2d(10,10),cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(0,0,255));

//            //line(src_image_, leftCenter, rightCenter, cv::Scalar(0, 255, 100), 2);

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
    if(match_armors_.size()>0)
    {
        sort(match_armors_.begin(),match_armors_.end(),[](const RotatedRect & rect1, const RotatedRect & rect2) {
            float ratio1=cv::max(rect1.size.width/rect1.size.height,rect1.size.height/rect1.size.width);
            float ratio2=cv::max(rect2.size.width/rect2.size.height , rect2.size.height/rect2.size.width);

            if(ratio1>=(ratio2*ArmorDetector:: hero_zjb_ratio))return true;
            else if(ratio2>=(ratio1*ArmorDetector:: hero_zjb_ratio))return false;
            else return rect1.size.area() > rect2.size.area();
        });

        Point2f vertices[4];
        match_armors_[0].points(vertices);//?1
#ifdef DEBUG
        line(src_image_,vertices[0],vertices[1],Scalar(255,255,255),5);
        line(src_image_,vertices[1],vertices[2],Scalar(255,255,255),5);
        line(src_image_,vertices[2],vertices[3],Scalar(255,255,255),5);
        line(src_image_,vertices[3],vertices[0],Scalar(255,255,255),5);
#endif
        sort(vertices, vertices + 4, [](const Point2f & p1, const Point2f & p2)
        {return p1.x < p2.x; });
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
    imshow("src",src_image_);

//    imshow("src_copy",src_image_copy);
   imshow("dst",thre_image_);
//    imshow("warpPerspective",warpPerspective_dst);

    //if语句在imshow前src看不到绿色框(可以准确识别)，放在后面即使src显示绿色框，帧率调整好后录制效果也算满意(绿色框不会频繁显示）
    //cout<<src_image_.cols<<endl;
   //cout<<src_image_.rows<<endl;
    waitKey(1);
}

vector<Point2d>& ArmorDetector::DetectObjectArmor(){
    PretreatImage();
    DetectLightBar();
    DetectArmor();
    ScreenArmor();
    ClearAll();

    return target_armor_point_set;
}

void ArmorDetector::baocun()
{
    //char key = waitKey(1);
    //if (key == 27) break;  //按ESC退出
   // if (key == 'q' || key == 'Q')
    {
        imgname = "./debug_shortcut/"+to_string(f++) + ".jpg"; //输出文件名为 f.jpg, 保留在工程文件夹中
        imwrite(imgname, src_image_);
    }
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
    Perspective_Transformation_src[0].x=lu.x+0.05*(ru.x-lu.x);
    Perspective_Transformation_src[0].y=lu.y-0.45*(ld.y-lu.y);
    Perspective_Transformation_src[1].x=ru.x-0.05*(ru.x-lu.x);
    Perspective_Transformation_src[1].y=ru.y-0.45*(rd.y-ru.y);
    Perspective_Transformation_src[2].x=rd.x-0.05*(rd.x-ld.x);
    Perspective_Transformation_src[2].y=rd.y+0.45*(rd.y-ru.y);
    Perspective_Transformation_src[3].x=ld.x+0.05*(rd.x-ld.x);
    Perspective_Transformation_src[3].y=ld.y+0.45*(ld.y-lu.y);

    Perspective_Transformation_dst[0]=Point2f(0,0);
    Perspective_Transformation_dst[1]=Point2f(320,0);
    Perspective_Transformation_dst[2]=Point2f(320,320);
    Perspective_Transformation_dst[3]=Point2f(0,320);
    warpPerspective_dst = Mat::zeros(320, 320, src_image_copy.type());
    warpPerspective_mat = getPerspectiveTransform(Perspective_Transformation_src, Perspective_Transformation_dst);
    warpPerspective(src_image_copy, warpPerspective_dst, warpPerspective_mat, warpPerspective_dst.size());

    circle(src_image_, Perspective_Transformation_src[0], 3, Scalar(0, 255, 0), -1);  // 画半径为1的圆(画点）
    circle(src_image_, Perspective_Transformation_src[1], 3, Scalar(0, 255, 0), -1);  // 画半径为1的圆(画点）
    circle(src_image_, Perspective_Transformation_src[2], 3, Scalar(0, 255, 0), -1);  // 画半径为1的圆(画点）
    circle(src_image_, Perspective_Transformation_src[3], 3, Scalar(0, 255, 0), -1);  // 画半径为1的圆(画点）


}
