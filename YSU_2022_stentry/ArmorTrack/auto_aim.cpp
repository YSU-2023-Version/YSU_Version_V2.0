#include "auto_aim.h"

void AutoAim::detect_image2res(cv::Mat src_image){
    this->src_image_ = src_image;
    // 获得输出结果
    vector<DetectRect> temp_detectRect = this->armor_detector.detect_yolov5(src_image_);
    // 如何处理输出结果
}

void AutoAim::init_auto_aim(){
    std::cout << "**    auto aim init begin    **" << std::endl;
    // 打开xml文件读取参数数据并进行初始化赋值
    std::string file_path="../xml_path/auto_aim.xml";
    cv::FileStorage fr;
    fr.open(file_path,cv::FileStorage::READ);
    while(!fr.isOpened()){
        std::cout<<"auto_aim_xml floading failed..." << std::endl;
        fr=cv::FileStorage(file_path,cv::FileStorage::READ);
        fr.open(file_path,cv::FileStorage::READ);
    }
    this->armor_detector = new Yolov5("../model/model/opt-0625-001.xml", "../model/model/opt-0625-001.bin", 416, 416); // 创建yolov5detector对象
    this->armor_detector.init_yolov5_detector();
    std::cout << "**    auto aim init done     **" << std::endl;
}

void update()

/**
 * @brief 逻辑主函数
*/
std::vector<cv::Point2f>& detect_track_armor(cv::Mat& src_image_){
    detect_image2res(src_image_);

}