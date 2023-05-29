/**
 * @author FoPluto
 * @brief 本文件是非多线程的Yolov5模型实现调用，如果急用可以用这个
 * @details 通过openvino实现调用模型，加速模型推理
*/

#include "DeepLearning/yolov5.h"

//#define DEBUG
#define WINDOW_NAME "res_show"

float sigmoid_function(float a)
{
    float b = 1. / (1. + exp(-a));
    return b;
}

Yolov5::Yolov5(){
    this->m_xml_path = "../model/best.xml"; // 默认模型路径一般不用，在初始化中被覆盖
    this->m_bin_path = "../model/best.bin";
}

// create function to define some element, change the xml some time
Yolov5::Yolov5(std::string xml_path, std::string bin_path, int input_width, int input_height){
    this->m_xml_path = xml_path;
    this->m_bin_path = bin_path;
    this->m_input_height = input_height;
    this->m_input_width = input_width;
}



void Yolov5::init_yolov5_detector(){
    printf("--------------start read network--------------\n");
    this->read_network();
    this->threshold = 0.6;
    this->color_num = 2;
    this->class_num = 8;
    this->class_names.push_back("sentry");
    this->class_names.push_back("a");
    this->class_names.push_back("hero_1");
    this->class_names.push_back("hero_1");
    this->class_names.push_back("energee_2");
    this->class_names.push_back("infantry_3");
    this->class_names.push_back("infantry_4");
    this->class_names.push_back("infantry_5");
    this->class_names.push_back("d");
    this->class_names.push_back("base");
    this->class_names.push_back("f");
    printf("---------------------done---------------------\n");
}




void Yolov5::read_network(){
    this->m_ie.SetConfig({{InferenceEngine::PluginConfigParams::KEY_DYN_BATCH_ENABLED, "YES"}}, "GPU");

    std::vector<std::string> availableDevice = this->m_ie.GetAvailableDevices();
    for(size_t i = 0;i < availableDevice.size();i++){
        printf("avaliable device: %s\n", availableDevice[i].c_str());
    }
    // 加载 IR 模型
    InferenceEngine::CNNNetwork network = m_ie.ReadNetwork(this->m_xml_path, this->m_bin_path);


    // get info from network
    this->m_input_info = InferenceEngine::InputsDataMap(network.getInputsInfo());
    this->m_output_info = InferenceEngine::OutputsDataMap(network.getOutputsInfo());

    //获取输入并进行设置（第一种方式）
    auto item = m_input_info.begin();
    image_info_name = item->first;         //获取image_info输入的名字
    auto image_info_ptr = item->second;    //获取image_info输入的指针
    //配置input_tensor输入：U8,NCHW, 保持默认的禁止自动放缩输入图像和禁止自动转换颜色通道
    image_info_ptr->setPrecision(InferenceEngine::Precision::FP32);
    image_info_ptr->setLayout(InferenceEngine::Layout::NCHW);
    image_info_ptr->getPreProcess().setColorFormat(InferenceEngine::ColorFormat::RGB);
    image_info_ptr->getPreProcess().setResizeAlgorithm(RESIZE_BILINEAR);
    //获取输出并进行设置(第二种方式)
    for (auto &output : m_output_info){
        output.second->setPrecision(Precision::FP32);
    }

    // 指定GPU插件名称
    std::string device_name = "GPU";
    this->m_executable_network = m_ie.LoadNetwork(network, device_name);
}






bool Yolov5::is_allready()
{
    if(m_input_info.size()>0 && m_output_info.size()>0)
        return true;
    else
        return false;
}

// 清除历史工作
void Yolov5::clear_work(){
    this->res_rects.clear();
}
/**
 * @brief 主要模型推理的函数
 * @author 可莉不知道哦
*/
vector<DetectRect>& Yolov5::infer2res(cv::Mat& src_){
    #ifdef DEBUG
    // 获取开始时间戳
    auto start = std::chrono::system_clock::now();
    #endif // DEBUG

    this->clear_work();

    scale_x = (float)src_.cols / m_input_width;
    scale_y = (float)src_.rows / m_input_height;

    cout <<"cols" <<  src_.cols << ", rows:" << src_.rows << endl;

    // 如果太大
    if(scale_x > 1 || scale_y > 1){
        max_scale = std::max(scale_x, scale_y);
        int res_width = src_.cols / max_scale;
        int res_height = src_.rows / max_scale;
        cv::resize(src_, src_, cv::Size(res_width, res_height));
    }
    // 如果是小了
    if(src_.cols != m_input_width || src_.rows != m_input_height) {
        scale_x = (float)src_.cols / m_input_width;
        scale_y = (float)src_.rows / m_input_height;

        if(scale_x < 1 && scale_y < 1){
            if(scale_x > scale_y){
                max_scale = scale_x;
                cv::resize(src_, src_, cv::Size(src_.cols / max_scale, src_.rows / max_scale));
            } else {
                max_scale = scale_y;
                cv::resize(src_, src_, cv::Size(src_.cols / max_scale, src_.rows / max_scale));
            }
        }
    }

    if(scale_x < 1){
        cv::copyMakeBorder(src_, src_, 0, 0, 0, this->m_input_width - src_.cols, BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    } else if(scale_y < 1){
        cv::copyMakeBorder(src_, src_, 0, this->m_input_height - src_.rows, 0, 0, BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    }

    cv::Mat pre;
    cv::Mat pre_split[3];
    src_.convertTo(pre,CV_32F);
    cv::split(pre,pre_split);
    // 创建推理请求
    InferenceEngine::InferRequest infer_request = m_executable_network.CreateInferRequest();

    size_t original_height = src_.rows;
    size_t original_width = src_.cols;
    /* 获得模型的image_info输入的数据缓冲 */
    Blob::Ptr image_info_blob = infer_request.GetBlob(image_info_name);
    /** 向模型的image_info输入的数据缓冲填入数据: height, width, scale=1.0 **/
    InferenceEngine::LockedMemory<void> blobMapped = InferenceEngine::as<MemoryBlob>(image_info_blob)->wmap();
    auto data = blobMapped.as<float*>();

    size_t img_size = src_.cols * src_.rows;
    // 参照沈航的使用指针copy，可以稍微快点
    for(int c = 0;c < 3;c++){
        memcpy(data, pre_split[c].data, original_width * original_height * sizeof(float));
        data += img_size;
    }

    // 推理
    infer_request.Infer();

    #ifdef DEBUG

    // 获取结束时间戳
    auto end = std::chrono::system_clock::now();

    // 计算时间差
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "推理时间花费: " << duration.count() << "ms" << std::endl;

    #endif // DEBUG

    // 获取输出 blob
    auto output_blob = infer_request.GetBlob("output");

    // 获取 blob 数据指针
    auto output_data = output_blob->buffer().as<InferenceEngine::PrecisionTrait<InferenceEngine::Precision::FP32>::value_type*>();

    // 获取输出 blob 大小和维度信息
    // auto output_size = output_blob->size();
    auto output_dims = output_blob->getTensorDesc().getDims();

    // 获取检测到的框数量
    int num_detections = output_dims[1];

    // 处理输出结果
    // 遍历检测到的框
    int dims = output_dims[2]; // weidu

    std::vector<DetectRect> rects;

    std::vector<DetectRect> final_rects;

    for (int i = 0; i < num_detections; ++i) {
        // 获得当前的grid, xy方向上的偏移数量
        int grid, x_num, y_num;
        if(i < 2704){
            grid = 8;
            x_num = i % 52;
            y_num = i / 52;
        } else if(i < 3380) {
            grid = 16;
            x_num = i % 26;
            y_num = i / 26;
        } else {
            grid = 32;
//            x_num = i % 13;
            y_num = i / 13;
        }
        // 获取框的置信度
        float demo[dims];
        int basic_pos = i * dims;
        float confidence = output_data[basic_pos + 8];

        if(confidence >= 0.64) {
            DetectRect temp_rect;
            float x_1 = (output_data[basic_pos + 0] + x_num) * max_scale * grid;
            float y_1 = (output_data[basic_pos + 1] + y_num) * max_scale * grid;
            float x_2 = (output_data[basic_pos + 2] + x_num) * max_scale * grid;
            float y_2 = (output_data[basic_pos + 3] + y_num) * max_scale * grid;
            float x_3 = (output_data[basic_pos + 4] + x_num) * max_scale * grid;
            float y_3 = (output_data[basic_pos + 5] + y_num) * max_scale * grid;
            float x_4 = (output_data[basic_pos + 6] + x_num) * max_scale * grid;
            float y_4 = (output_data[basic_pos + 7] + y_num) * max_scale * grid;

            for(int j = 0;j < 21;j++) demo[j] = output_data[basic_pos + j];

            // 获得最大概率的类别和颜色，取值 9 or 10 or 11        red : 10 blue : 9 dead : 11
            int box_color = -1;
            for(int j = 9;j <= 11;j++){
                if(box_color  == -1 || output_data[box_color + basic_pos] < output_data[j + basic_pos]) box_color = j;
            }
            // 类别索引
            int box_class = -1;
            for(int j = 12;j < 21;j++){
                if(box_class  == -1 || output_data[box_class + basic_pos] < output_data[j + basic_pos]) box_class = j;
            }
            // 类别置信度
            float class_p = output_data[box_class + basic_pos];
            float color_p = output_data[box_color + basic_pos];
            // 如果最大的类别置信度过低，就舍去

            if(box_color ==  11) { // dead
                continue;
            }
//            if(box_color == 9){ // blue not
//                continue;
//            }
//            if(box_color == 10){ // red not
//                continue;
//            }


            #ifdef DEBUG
            cv::circle(this->m_src_image, cv::Point(x_1, y_1), 3, cv::Scalar(0, 255, 0), 2);
            cv::circle(this->m_src_image, cv::Point(x_2, y_2), 3, cv::Scalar(0, 255, 0), 2);
            cv::circle(this->m_src_image, cv::Point(x_3, y_3), 3, cv::Scalar(0, 255, 0), 2);
            cv::circle(this->m_src_image, cv::Point(x_4, y_4), 3, cv::Scalar(0, 255, 0), 2);
            #endif // DEBUG item point 测试角点是否正确检测
            // 获取最大矩形框，垂直的
            int max_x = std::max(x_1, std::max(x_2, std::max(x_3, x_4)));
            int max_y = std::max(y_1, std::max(y_2, std::max(y_3, y_4)));
            int min_x = std::min(x_1, std::min(x_2, std::min(x_3, x_4)));
            int min_y = std::min(y_1, std::min(y_2, std::min(y_3, y_4)));
            // 将计算得到的结果保存到容器元素中
            temp_rect.min_point = cv::Point(min_x, min_y);
            temp_rect.max_point = cv::Point(max_x, max_y);
            temp_rect.rect = cv::Rect(temp_rect.min_point, temp_rect.max_point);
            temp_rect.points.push_back(cv::Point2f(x_1, y_1));
            temp_rect.points.push_back(cv::Point2f(x_2, y_2));
            temp_rect.points.push_back(cv::Point2f(x_3, y_3));
            temp_rect.points.push_back(cv::Point2f(x_4, y_4));
            temp_rect.cen_p = cv::Point2f((x_1 + x_2 + x_3 + x_4) / 4.0, (y_1 + y_2 + y_3 + y_4) / 4.0);
            temp_rect.class_id = box_class;
            temp_rect.class_p = class_p;
            temp_rect.color_id = box_color;
            temp_rect.color_p = color_p;
            temp_rect.area = temp_rect.r_rect.size.area();
            std::string color_name = temp_rect.color_id == 9 ? "blue_" : temp_rect.color_id == 10 ? "red_" : "dead_";
            temp_rect.class_name = color_name + this->class_names[temp_rect.class_id - 11];
            getSystime(temp_rect.time); // 获取时间戳
            #ifdef DEBUG
//            std::cout << "confidence: " << confidence << std::endl;
            // circle(src_, temp_rect.cen_p, 4, cv::Scalar(255, 0, 0), 4);
            #endif // DEBUG 调试此时的src_image，查看角点的情况

            // add to rects
            rects.push_back(temp_rect);
        }
        // ...
    }
    // sort
    if(rects.size()){
        std::sort(rects.begin(), rects.end(), [](const DetectRect& d1, const DetectRect& d2){
            if(d1.class_p != d2.class_p) return d1.class_p > d2.class_p;
            return d1.class_p * d1.color_p > d2.class_p * d2.color_p;
        });
        // IOU
        res_rects.push_back(rects[0]); // push the best to the res_vector
        bool flag = true;
        for(unsigned int i = 1;i < rects.size();i++){
            for(auto item_max_rect : res_rects){
                cv::Rect max_p_rect = item_max_rect.rect; // max p rect
                cv::Rect item_p_rect = rects[i].rect; // item p rect

                int x1 = std::max(max_p_rect.x, item_p_rect.x);
                int y1 = std::max(max_p_rect.y, item_p_rect.y);
                int x2 = std::min(max_p_rect.x + max_p_rect.width, item_p_rect.x + item_p_rect.width);
                int y2 = std::min(max_p_rect.y + max_p_rect.height, item_p_rect.y + item_p_rect.height);

                int intersectionArea = std::max(0, x2 - x1) * std::max(0, y2 - y1);

                int or_space = max_p_rect.area() + item_p_rect.area() - intersectionArea;

                float IOU_with_the_max = (float)intersectionArea / or_space;
                // 如果IOU过大就将其舍去
                if(IOU_with_the_max >= 0.83) {
                    flag = false;
                    break;
                }
            }
            if(flag){
                res_rects.push_back(rects[i]);
            }
        }
    }

    #ifdef DEBUG  // 单独开发深度学习模块时用的，已经弃用
//    this->draw_res();
//    std::cout << "num: " << sum << std::endl;
//    cv::imshow("dst_image", m_src_image);
//    cv::waitKey(1);
    #endif // DEBUG 调试最终得到的点

    return res_rects;

}

void Yolov5::draw_res(){
    // iterate the res rect
    for(auto item_res_rect : res_rects){
        cv::Point p01 = item_res_rect.points[0]; // zuo shang
        cv::Point p02 = item_res_rect.points[1]; // zuo xia
        cv::Point p03 = item_res_rect.points[2]; // you xia
        cv::Point p04 = item_res_rect.points[3]; // you shang

        cv::circle(m_src_image, p01, 2, cv::Scalar(0, 0, 255), 2);
        cv::circle(m_src_image, p02, 2, cv::Scalar(0, 0, 255), 2);
        cv::circle(m_src_image, p03, 2, cv::Scalar(0, 0, 255), 2);
        cv::circle(m_src_image, p04, 2, cv::Scalar(0, 0, 255), 2);
        cv::line(m_src_image, p01, p02, cv::Scalar(0, 255, 0));
        cv::line(m_src_image, p01, p03, cv::Scalar(0, 255, 0));
        cv::line(m_src_image, p01, p04, cv::Scalar(0, 255, 0));
        cv::line(m_src_image, p02, p03, cv::Scalar(0, 255, 0));
        cv::line(m_src_image, p02, p04, cv::Scalar(0, 255, 0));
        cv::line(m_src_image, p03, p04, cv::Scalar(0, 255, 0));
    }
}


vector<DetectRect>& Yolov5::detect_yolov5(cv::Mat& src_){
    this->m_src_image = src_;
//    this->m_src_image.copyTo(m_src_copy_image);
//    this->infer2res(m_src_copy_image);
    // 优化改成
    return infer2res(m_src_image); // 添加到qt项目中不需要copy
}


void Yolov5::show_res(){
    cv::imshow(WINDOW_NAME, this->m_src_copy_image);
}
