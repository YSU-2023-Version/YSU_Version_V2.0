#include "Main/headfiles.h"
#include "RuneDetector/rune_detector.h"

//#ifndef DEBUG
//#define DEBUG
//#endif // !DEBUG



using namespace std;
using namespace cv;

inline double getDistance(const Point2f& a, const Point2f& b)
{
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
inline Point2f getMiddle(const Point2f& a, const Point2f& b)
{
    return Point2f(0.5 * (a.x + b.x), 0.5 * (a.y + b.y));
}

RuneDetector::RuneDetector()
{
    readFromXML();
     i = -1;
     buff_mode = WAIT_TO_CHECK;
}

void RuneDetector::getShootAim(const Mat & src,double time,std::promise<Point2f> & shoot)
{

        //imwrite("/home/robomaster/qtcreator_workspace/114514/"+to_string(time)+".jpg",src);
    //cout<<"                             1111"<<DELAY_TIME<<endl;
    if (src.empty()) {
       shoot.set_value(Point2f(320,240));

       return;//没图，给电控传0，底盘放松。
    }
    src.copyTo(src_);
    preDelBuff();//图像预处理
#ifdef DEBUG
    cout<<"\npreDelBuff complete\n";
#endif
    if (!searchOfAim()){//查找目标Aim和R
        shoot.set_value(Point2f(320,240));
#ifdef DEBUG
        cout << "\n warning:find Aim failed" << endl;
#endif


        return;//图像中没有目标点，给电控传0，底盘放松。
    }



    //添加卡尔曼滤波

    history[++i % 30] = record(aim, r, time);
    checkBuffMode();  //如果电控给了大符模式标志位，就把这句注释掉。

    getforecastAim(); //告诉操作手，前几帧没法预测，让稍微等几秒钟。

#ifdef DEBUG
circle(src_,Point(out.x,out.y),5,Scalar(0,255,0),3);
imshow("src",src_);
imwrite("/home/robomaster/qtcreator_workspace/114514/"+to_string(time)+".jpg",src_);
cout<<"\n\n\n\n"<<"114514"<<"\n\n\n\n\n";
//waitKey(1);
#endif
    shoot.set_value(out);

    return ;

}

void RuneDetector::preDelBuff() {
    if (pretreatMode == PRETREAT_HSV) {//pretreatMode == PRETREAT_HSV
        if(src_.type()==CV_8UC3)
            cvtColor(src_, dst, CV_BGR2GRAY);
        else
        {
            src_.copyTo(dst);
#ifdef DEBUG
            imwrite("/home/robomaster/qtcreator_workspace/114514/error/0.jpg",src_);
            cout<<"\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n dst is not CV_8UC3\n\n";

#endif
        }
        //cvtColor(src_, dst, CV_RGB2GRAY);为什么要写RGB？？？


        threshold(dst, dst, 70, 255, CV_THRESH_BINARY);
        Mat hsv, mask;
        cvtColor(src_, hsv, CV_RGB2HSV);
        cv::Mat gray_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        inRange(hsv, Scalar(0, 10, 46), Scalar(180, 60, 255), mask);  // ti qu yan mo    blue mask
        dilate(mask, mask, gray_element);
        dst = dst - mask;  // xiang jian de dao red mask

        dilate(dst, dst, gray_element);
        Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        erode(dst, dst, element);
        imshow("dst1", dst);

        return;
    }
    else{
        //通道相减，灰度阈值二值
        Mat gray;
        if(src_.type()==CV_8UC3)
            cvtColor(src_, gray, COLOR_BGR2GRAY);
        else
        {
#ifdef DEBUG
            src_.copyTo(gray);
            imwrite("/home/robomaster/qtcreator_workspace/114514/error/1.jpg",src_);
#endif

            cout<<"\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n gray is not CV_8UC3\n\n";
        }

        threshold(gray, gray, 50, 255, THRESH_BINARY);
        cv::Mat core = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        dilate(gray, gray, core);

        // 红蓝通道相减
        Mat temp_binary;
        vector<Mat> splited;
        split(src_, splited);
        if (color_aim == RUNE_DETECTOR_IS_RED)  // to red
        {
            addWeighted(splited[2], 1, splited[0], -0.5, 30, temp_binary);

        }
        else{  // to blue
            addWeighted(splited[0], 1, splited[2], -0.5, 30, temp_binary);

        }
        threshold(temp_binary, temp_binary, 30, 255, THRESH_BINARY);
        cv::Mat core_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        dilate(temp_binary, temp_binary, core_);

        dst = temp_binary & gray;
        dilate(dst, dst, core_,Point(-1,-1),2);
        //3.24外加
        //闭运算
        Mat procImg;
        int structElementSize = 3;
        Mat element = getStructuringElement(MORPH_RECT,Size(2 * structElementSize + 1,2 * structElementSize + 1),Point(structElementSize,structElementSize));
        morphologyEx(dst,procImg,MORPH_CLOSE,element);


#ifdef DEBUG
        imshow("procImg", procImg);
#endif

        //imshow("dst",dst);
        return;
    }
}

//功能：发现R点和装甲板击打点
//return false：未发现R或Aim，给电控传0。true：发现了R和Aim，给电控传1
bool RuneDetector::searchOfAim()
{
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<vector<Point>>::iterator iter;
    vector<RotatedRect> R_, Aim_;
    findContours(dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);


#ifdef DEBUG
    cout<<"\nfindcontourse complete\n";
#endif

    //子轮廓统计
    vector<int>	num_of_son(contours.size(), 0);
    for (int t = 0; t < hierarchy.size(); t++)
    {//父轮廓存在，则父轮廓的子轮廓数量++
        int index_temp = hierarchy[t][3];
        if (index_temp == -1)continue;
        num_of_son[index_temp]++;


    }

    for (iter = contours.begin(); iter != contours.end(); iter++)
    {
        if (contourArea(*iter) < src_.rows * AREA_LIMIT_MIN)continue;//面积限制
        RotatedRect rect = minAreaRect(*iter);
        if ((rect.size.width / rect.size.height > 0.7 && rect.size.width / rect.size.height < 1.2)
            || (rect.size.height / rect.size.width > 0.7 && rect.size.height / rect.size.width < 1.2)) //认为是R
        {
            //子轮廓限制，但是由于现场效果未知，是写0还是写1要等到赛场提前上去试一下。
           // if (num_of_son[iter - contours.begin()] != R_SON_CONTOURS_NUM)continue;

            //R在图像中间限制（如果R不在中间，边缘会超出）
            if (rect.center.x < MIN_R_CENTER * src_.cols || rect.center.x>MAX_R_CENTER * src_.cols || rect.center.y < MIN_R_CENTER * src_.rows || rect.center.y>MAX_R_CENTER * src_.rows)continue;

            R_.push_back(rect);
        }

        //if(num_of_son[iter - contours.begin()] == AIM_SON_CONTOURS_NUM)
        if ((rect.size.width / rect.size.height > 1.5 && rect.size.width / rect.size.height < 8)
            || (rect.size.height / rect.size.width > 1.5 && rect.size.height / rect.size.width < 8)
                )//认为是扇叶
        {
            //子轮廓检测(AIM_SON_CONTOURS_NUM=1)赛场上前试一下，如果结果不理想，就这句注释掉
            if (num_of_son[iter - contours.begin()] != AIM_SON_CONTOURS_NUM)continue;
            cout<<"\n\n\nnum_of_son\n"<<num_of_son[iter - contours.begin()]<<"\nend\n\n";
            //邻域检测
            Point2f point[4];
            rect.points(point);
            if (getDistance(point[0], point[1]) > getDistance(point[1], point[2]))  swap(point[0], point[2]);//[0][1]是短边，[1][2]长边
            Point2f longMiddle = getMiddle(point[1], point[2]); // small
#ifdef DEBUG
    cout<<"1\n1\n1\n1\n"<<aim<<"1\n1\n1\n1\n";
    Mat m;
    src_.copyTo(m);

    rectangle(m,rect.boundingRect(),Scalar(255,0,0),4);
    imshow("preforcast",m);
    waitKey(10);
#endif


            Rect neighborhood = Rect(longMiddle.x - NEIGHBORHOOD_SCALE_AIM / 2, longMiddle.y - NEIGHBORHOOD_SCALE_AIM / 2, NEIGHBORHOOD_SCALE_AIM, NEIGHBORHOOD_SCALE_AIM);

            if(neighborhood.x<=0||neighborhood.y<=0)
            {
                //cout << "\n\n\n" << "114514" << "\n\n\n";
                continue;
            }

            neighborhood.y = min(neighborhood.y, dst.rows);
            neighborhood.x = min(neighborhood.x, dst.cols);


            cout<<"\n\n\nneighborhood.y.x.+:"<<neighborhood.y<<"  "<<neighborhood.y + neighborhood.height<<"  "<<neighborhood.x<<" "<<neighborhood.x + neighborhood.width<<endl;
            cout << "dst.x: " << dst.rows << "   dst.y: " << dst.cols << endl << endl;

            Mat roi = dst(Range(neighborhood.y, neighborhood.y + neighborhood.height>dst.rows?dst.rows:neighborhood.y + neighborhood.height),
                          Range(neighborhood.x, neighborhood.x + neighborhood.width>dst.cols?dst.cols:neighborhood.x + neighborhood.width)
                          );
            cout<<"/nyes/n";
            if (static_cast<double>(mean(roi).val[0]) != 0)continue;
            //cout << "\n\n\n" <<"114514"<< "\n\n\n";
            int temp = hierarchy[iter-contours.begin()][2];

            if(temp != -1)
            {
                //cout<<"\n\n\n\ntemp:"<<temp<<endl;
                Aim_.push_back(minAreaRect(contours[temp]));//待击打扇叶的子轮廓为装甲板。
            }
        }
    }
#ifdef DEBUG
    cout<<"\n\ncontourse find complete\n\n";
#endif

    if (R_.size() == 0)
    {
#ifdef DEBUG
        cout << "warning:R_.size==0" << endl;
        imshow("preforcast",src_);
#endif

        return false;
    }
    if (Aim_.size() == 0)
    {
#ifdef DEBUG
        cout << "warning:Aim_.size==0" << endl;
        imshow("preforcast",src_);
#endif

        return false;
    }

    sort(R_.begin(), R_.end(), [](const RotatedRect& a, const RotatedRect& b) {return a.size.area() > b.size.area(); });
    sort(Aim_.begin(), Aim_.end(), [](const RotatedRect& a, const RotatedRect& b) {return a.size.area() > b.size.area(); });

    R = *R_.begin();
    Aim = *Aim_.begin();
    r = R.center;
    aim = Aim.center;


#ifdef DEBUG
    cout<<"1\n1\n1\n1\n"<<aim<<"1\n1\n1\n1\n";
    Mat m;
    src_.copyTo(m);
    circle(m,aim,10,Scalar(255,0,0),4);
    imshow("preforcast",m);
    waitKey(10);
#endif


    return true;
}

void RuneDetector::checkBuffMode()
{
    if (i < 2)
    {
        buff_mode = WAIT_TO_CHECK;
        return;//帧数未到，不预测。
    }double distance[8];
    double time[8];
    for (int t=0;t<8;t++)
    {
        distance[t] = getDistance(history[(i % 30) - t].aim, history[(i % 30) - t - 1].aim);
        time[t] = fabs(history[(i % 30) - t].time - history[(i % 30) - t - 1].time)/getTickFrequency();
    }
    int grade = 0;//积分制判断能量机关模式，消除突变影响。
    for (int t = 0; t < 7; t++)
    {
        if ((distance[t] / time[t]) - (distance[t + 1] / time[t + 1]) > D_VELOCITY_SAME_MAX) {
            grade++;
        }
    }

    if (grade > 4)
    {
        buff_mode = LARGE_BUFF_MODE;
    }
    else
    {
        buff_mode = SMALL_BUFF_MODE;
    }
    return;
}

void RuneDetector::getforecastAim()
{//小符V=10rpm
//大符spd = 0.785 ∗ sin (1.884 ∗ t) + 1.305	rad/s
    if (buff_mode == WAIT_TO_CHECK)
    {
        out = aim;
        return;
    }

    vector<double> d_time;
    vector<double> d_angle;
    d_time.clear();
    d_angle.clear();
    if(buff_mode==SMALL_BUFF_MODE){
        for (int t = 1; t >= 0; t--){
            d_time.push_back((history[(i - t) % 30].time - history[(i % 30)].time)/getTickFrequency());
            d_angle.push_back(history[(i - t ) % 30].angle);
        }
    }
    else{//buff_mode==LARGE_BUFF_MODE
        for (int t = 3; t >=0; t--){
            d_time.push_back((history[(i - t) % 30].time - history[(i % 30)].time)/getTickFrequency());
            d_angle.push_back(history[(i - t) % 30].angle);
        }
    }
    float forecast_angle = Lagrange(d_time, d_angle, DELAY_TIME);
    double out_x = src_.cols/2 + history[i % 30].distance * cos((forecast_angle * 3.1415926) / 180);
    double out_y = src_.rows/2 + history[i % 30].distance * sin((forecast_angle * 3.1415926) / 180);/*
    double out_x = r.x + history[i % 30].distance * cos((forecast_angle * 3.1415926) / 180);
    double out_y = r.y + history[i % 30].distance * sin((forecast_angle * 3.1415926) / 180);*/
    out= Point2f(out_x, out_y);
    return;
}

double RuneDetector::Lagrange(const vector<double>& X, const vector<double>& Y, double x)
{
    if (X.size() == 0)return -1.0;
    if (X.size() != Y.size())return -1.0;

    double result = 0;
    for (int xj = 0; xj < X.size(); xj++) {
        double temp = Y[xj];
        for (int yj = 0; yj < X.size(); yj++) {
            if (xj != yj) {
                temp = temp * (x - X[yj]);
                temp = temp / (X[xj] - X[yj]);
            }
        }
        result += temp;
    }
    return result;
};

void RuneDetector::readFromXML()
{
    FileStorage fs("/home/robomaster/qtcreator_workspace/YSU_2022_sentry_20220605/YSU_2022_stentry/RuneDetector/buff_.xml", FileStorage::READ);//要求xml文件根节点必须为<opencv_storage>
    //关于图像预处理
    fs["pretreatMode"] >> pretreatMode;
    fs["color_aim"] >> color_aim;

    fs["R_SON_CONTOURS_NUM"] >> R_SON_CONTOURS_NUM;//0还是1要等现场测试。效果不确定的话，直接弃用。
    fs["AIM_SON_CONTOURS_NUM"] >> AIM_SON_CONTOURS_NUM;
    fs["NEIGHBORHOOD_SCALE_AIM"] >> NEIGHBORHOOD_SCALE_AIM;
    fs["MIN_R_CENTER"] >> MIN_R_CENTER;
    fs["MAX_R_CENTER"] >> MAX_R_CENTER;
    fs["DELAY_TIME"] >> DELAY_TIME;
    fs["D_VELOCITY_SAME_MAX"] >> D_VELOCITY_SAME_MAX;
    fs["AREA_LIMIT_MIN"] >> AREA_LIMIT_MIN;

    fs.release();
    return;
}

RuneDetector::record::record()
{
}

RuneDetector::record::record(Point2f aim_, Point2f r_, double time_)
    :aim(aim_), r(r_), time(time_)
   // , angle_vector(Vec2f(aim.x - r.x, aim.y - r.y))
    ,distance(getDistance(aim_,r_))
    , angle(atan2(aim_.y - r_.y, aim_.x - r_.x) * 180 / 3.1415926)
{}

double RuneDetector::record::operator-(const record& a)//this.angle-a.angle.-180~180
{
    double temp;
    temp = this->angle - a.angle;
    if (temp < -180.0) {
        temp += 360;
    }
    if (temp > 180.0) {
        temp -= 360;
    }
    return temp;
}

void RuneDetector::record::print()
{
    cout << "aim:" << aim << " ";
    cout << "r:" << r << " ";
    cout << "distance:" << distance << " ";
    cout << "angle:" << angle << " ";
    cout << "time:" << time<<endl;
}

