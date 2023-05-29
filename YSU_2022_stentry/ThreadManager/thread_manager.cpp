#include "Main/headfiles.h"
#include "ThreadManager/thread_manager.h"
#include "ArmorDetector/armor_detector.h"
#include "CameraManager/camera_manager.h"
#include "Communication/communication.h"
#include "RuneDetector/rune_detector.h"
// 是否使用预测
#define ISFORECAST

ThreadManager::ThreadManager():
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
    p_forecast_ -> Init();
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

void ThreadManager::Produce(){
    int camera_base_time = static_cast<int>(1000000/Camera_FPS_);
    int sleep_time;
    while(1) {
        if (++i % BUFFER_LENGTH == 0){
            i = 0;
        }

        locker[i].lock();

        bool IsRecv=false;

        p_communication_->RecvMcuData(y_p_recv[i],IsRecv);
        if(!IsRecv)
        {
            y_p_recv[i][0]=y_p_recv[i][1]=y_p_recv[i][2]=y_p_recv[i][3]=0;
        }

        auto start_time = std::chrono::system_clock::now();
        p_camera_manager_ -> ReadImage(buffer_mat[i]);
        locker[i].unlock();

        auto end_time = std::chrono::system_clock::now();
        sleep_time = camera_base_time - (std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count());
        if (sleep_time > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_time));
        }
    }
}

void ThreadManager::Consume(){
    //一个子线程，用于后处理。
    auto sub_consumer = std::thread(&ThreadManager::subConsume,this);
    sub_consumer.detach();

    int vision_base_time = static_cast<int>(1000000/Vision_FPS_);
    int sleep_time;
    while(1){
        if(++j % BUFFER_LENGTH == 0){
            j = 0;
        }

        //两块buffer区均采用地址传递，减少不必要的copy
        locker[j].lock();
        locker_point[j].lock();
        buffer_points[j] = p_armor_detector_ -> DetectObjectArmor(buffer_mat[j]);
        // p_armor_detector_ -> Show();
        locker_point[j].unlock();
        locker[j].unlock();
    }
}


void ThreadManager::subConsume(){
    //通过camera的fps控制，自动实现每一帧之间等△T。推理比camera快，会在lock的时候等camera，同理后处理等推理，因此kalman时只需要下标 +（1*参数），不需要传递sys_time。
    //此处接口未适配去掉system time。
    while(1){
        if(++j2 % BUFFER_LENGTH == 0){
            j2 = 0;
        }
        locker_point[j2].lock();
        auto &tmp = p_forecast_-> forcast(buffer_points[j2], sys_time[j] ,y_p_recv[j]);
        locker_point[j2].unlock();

        p_communication_ ->UpdateData(p_angle_solver_ -> SolveAngle(tmp, y_p_recv[j]));
        p_communication_ ->shoot_err(p_angle_solver_ -> shoot_get());
    }
}



void ThreadManager::Communicate(){ //传递信息就直接修改p_communication_->Infantry中对应的数值即可
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
