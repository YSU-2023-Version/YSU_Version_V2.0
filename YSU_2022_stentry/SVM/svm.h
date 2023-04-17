#ifndef SVM_H
#define SVM_H

#include "Main/headfiles.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/bufferpool.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/ml.hpp"




class YSU_SVM
{
public:
    YSU_SVM();
    void InitSVM();
    float getNum(cv::Mat& original);

private:
    void PretreatImage(cv::Mat &original);
    void convertTo3Channels( cv::Mat &binImg);
    cv::Mat three_channel;
    cv::Mat copy_resize;
  //  int width,height;
    cv::Ptr <cv::ml::SVM> ysu_svm;

    std::string xml_path;
    cv::HOGDescriptor hog;
    std::vector<float> descriptors;

};

#endif
