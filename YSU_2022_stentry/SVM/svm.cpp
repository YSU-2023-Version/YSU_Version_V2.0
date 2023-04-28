#include"SVM/svm.h"
// #define DEBUG

/*
svm，用于筛选当前结果是否为工程/误识别
@auther：金厚羽
*/

YSU_SVM::YSU_SVM()
    :ysu_svm(cv::ml::SVM::create())
{

}
void YSU_SVM::InitSVM(){


    std::string file_path="../xml_path/svm.xml";
    cv::FileStorage fr;
    fr.open(file_path,cv::FileStorage::READ);
    while(!fr.isOpened()){
          std::cout<<"YSU_SVM_xml loading failed..."<<std::endl;
         fr.open(file_path,cv::FileStorage::READ);
    }
    fr["svm_xml"]>>xml_path;
    //ysu_svm=ml::SVM::load(xml_path);
    ysu_svm=cv::ml::StatModel::load<cv::ml::SVM>(xml_path);
     //  ysu_svm=Algorithm::load<ml::SVM>(xml_path);

    while(!ysu_svm)
    {
        cout<<"svm_xml loading failed..."<<endl;
         ysu_svm=cv::ml::StatModel::load<cv::ml::SVM>(xml_path);
    }
    fr.release();

}

float YSU_SVM::getNum(cv::Mat& original)
{
     PretreatImage(original);
     resize(original,copy_resize,cv::Size(64,64),0,0,cv::INTER_LINEAR);
     convertTo3Channels(copy_resize);

     descriptors.clear();
     hog=cv::HOGDescriptor(cv::Size(64,64),cv::Size(16,16),cv::Size(8,8),cv::Size(8,8),9);
     hog.compute(copy_resize,descriptors,cv::Size(8,8));
    return ysu_svm->predict(descriptors);
}

void YSU_SVM::PretreatImage(cv::Mat &original){
     cvtColor(original,original,CV_BGR2GRAY);
     threshold(original,original,0,255,cv::THRESH_BINARY|cv::THRESH_OTSU);
}


void YSU_SVM::convertTo3Channels(cv:: Mat& binImg)
{
    three_channel = cv::Mat::zeros(binImg.rows,binImg.cols,CV_8UC3);
    vector<cv::Mat> channels;
    for (int i=0;i<3;i++)
    {
        channels.push_back(binImg);
    }
    merge(channels,three_channel);

}
