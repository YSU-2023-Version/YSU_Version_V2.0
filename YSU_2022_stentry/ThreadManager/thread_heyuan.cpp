#include "thread_heyuan.h"

threadManager::threadManager():
    p_armor_detector_(std::make_unique<ArmorDetector>()),
    p_forecast_(std::make_unique<Forecast>()),
    p_angle_solver_(std::make_unique<AngleSolver>()),
    p_camera_manager_(std::make_unique<CameraManager>()),
    p_communication_(std::make_unique<Communication>()),
    p_run_detector_(std::make_unique<RuneDetector>()),
    i(0),
    j(0),
    j2(0)
{
    if(BUFFER_LENGTH < 3){
        std::cout << "buffer is too short, set at least 3"<<std::endl;
        exit(1);
    }
    p_camera_manager_ -> InitCamera();
    p_armor_detector_ -> InitArmor();
    p_communication_ -> InitCom();
    p_angle_solver_ ->InitAngle();
    p_communication_ -> open();


    std::string xml_path = "../xml_path/thread_hhy.xml";
    cv::FileStorage fr;
    fr.open(xml_path,cv::FileStorage::READ);
    while(!fr.isOpened()){
        std::cout << "armor_xml loading failed..." << std::endl;
        fr=cv::FileStorage(xml_path, cv::FileStorage::READ);
        fr.open(xml_path, cv::FileStorage::READ);
    }
    fr["Communit_FPS_"] >> Communit_FPS_; 
    fr["Camera_FPS_"] >> Camera_FPS_;
}

void threadManager::Produce(){
    int camera_base_time = static_cast<int>(1000/Camera_FPS_) ;
    int sleep_time;


    std::time_t duration_time = 0;
    int sleep_sum = 0;
    int num = 0;
    while(1) {
        auto start = std::chrono::system_clock::now();
        if (++i % BUFFER_LENGTH == 0){
            i = 0;
            if (duration_time > 5000){
                std::cout << "avg Camera Hz: "<< 1000 /(duration_time / num) << " ,avg sleep time: "<< sleep_sum / num << std::endl;
                duration_time = 0;
                sleep_sum = 0;
                num = 0;
            }
        }

        locker[i].lock();
        p_camera_manager_ -> ReadImage(buffer_mat[i]);
        locker[i].unlock();

        auto end_time = std::chrono::system_clock::now();
        sleep_time = camera_base_time - (std::chrono::duration_cast<std::chrono::milliseconds>(end_time.time_since_epoch() - start.time_since_epoch()).count());
        if (sleep_time > 0) {
            sleep_sum += sleep_time;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
        }

        auto end = std::chrono::system_clock::now();
        num++;
        duration_time += std::chrono::duration_cast<std::chrono::milliseconds>(end.time_since_epoch()- start.time_since_epoch()).count();
    }
}

void threadManager::Consume(){
    //一个子线程，用于后处理。
    auto sub_consumer = std::thread(&threadManager::subConsume,this);
    std::time_t duration_time = 0;
    int num = 0;
    while(1){
        auto start = std::chrono::system_clock::now();
        if(++j % BUFFER_LENGTH == 0){
            j = 0;
            if (duration_time > 5000){
                std::cout << "avg infer time: "<<(duration_time / num) <<  std::endl;
                duration_time = 0;
                num = 0;
            }


        }
        if (buffer_mat[j].cols == 0){
            continue;
        }

        //两块buffer区均采用地址传递，减少不必要的copy
        locker[j].lock();
        locker_point[j].lock();

         buffer_points[j] = p_armor_detector_ -> DetectObjectArmor(buffer_mat[j]);
//         p_armor_detector_ -> Show();

        locker_point[j].unlock();
        locker[j].unlock();

        auto end = std::chrono::system_clock::now();
        num++;
        duration_time += std::chrono::duration_cast<std::chrono::milliseconds>(end.time_since_epoch()- start.time_since_epoch()).count();
    }

    sub_consumer.join();
}


void threadManager::subConsume(){
    //通过camera的fps控制，自动实现每一帧之间等△T。
    //推理比camera快，会在lock的时候等camera，同理后处理等推理，因此kalman时只需要下标 +（1*参数），不需要知道单位1的确切含义，因此不需要传递sys_time。
    //此处接口未适配去掉system time。
    
    double* p_y_err;
    std::time_t duration_time = 0;
    int num = 0;
    while(1){
        auto start = std::chrono::system_clock::now();

        if(++j2 % BUFFER_LENGTH == 0){
            j2 = 0;
            if (duration_time > 5000){
                std::cout <<"avg Postprecess Hz: "<< (duration_time / BUFFER_LENGTH) << std::endl;
                duration_time = 0;
                num = 0;
            }
        }
        if(buffer_points[j2].size() == 0){
            continue;
        }

        locker_point[j2].lock();
        p_y_err = p_angle_solver_ -> SolveAngle(buffer_points[j2]);
        locker_point[j2].unlock();

        p_communication_ ->UpdateData(p_y_err);
        p_communication_ ->shoot_err(p_angle_solver_ -> shoot_get());

        auto end = std::chrono::system_clock::now();
        num++;
        duration_time += std::chrono::duration_cast<std::chrono::milliseconds>(end.time_since_epoch()- start.time_since_epoch()).count();
    }
}



void threadManager::Communicate(){ //传递信息就直接修改p_communication_->Infantry中对应的数值即可
    int communit_base_time = static_cast<int>(1000000/Communit_FPS_);
    int sleep_time;

    while(1)
    {
        auto start_time = std::chrono::system_clock::now();

        p_communication_->communication(p_communication_ -> Infantry);


        auto end_time = std::chrono::system_clock::now();
        sleep_time = communit_base_time - (std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
        }
    }
}
