#include "Main/headfiles.h"
#include "ThreadManager/thread_manager.h"
#include "ArmorDetector/armor_detector.h"
#include "CameraManager/camera_manager.h"
#include "Communication/communication.h"
#include "RuneDetector/rune_detector.h"
ThreadManager::ThreadManager():
    p_armor_detector_(std::make_unique<ArmorDetector>()),
    p_forecast_(std::make_unique<Forecast>()),
    p_angle_solver_(std::make_unique<AngleSolver>()),
    p_camera_manager_(std::make_unique<CameraManager>()),
    p_communication_(std::make_unique<Communication>()),
    p_run_detector_(std::make_unique<RuneDetector>()),
    i(0),
    j(0)
{}

void ThreadManager::Init(){
    p_camera_manager_ -> InitCamera();
    p_armor_detector_ -> InitArmor();
    p_communication_ -> InitCom();
    p_angle_solver_ ->InitAngle();
    p_forecast_->Init();
    p_communication_->open();
}

void ThreadManager::Produce(){
    while(1)
    {
        auto t1 = std::chrono::high_resolution_clock::now();


        buffer[i] = p_camera_manager_ -> ReadImage();
        getSystime(sys_time[i]);
        //cout << "i : " << i << " j: " << j;
        condition.notify_one(); //通知wait()函数，解除阻止
        if( (++i) % 30 == 0 )
        {
            i = 0;
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        // 稳定帧率每秒100帧
        int time = 10 - ((static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count());
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + std::chrono::milliseconds(time);
        // 使用循环和 std::this_thread::yield 函数来让当前线程让出CPU，直到指定的时间到达为止。
        while (std::chrono::steady_clock::now() < end_time) {
            std::this_thread::yield();
        }
        auto t3 = std::chrono::high_resolution_clock::now();
        //！std::cout << "ProducerFPS: " << 1000/(static_cast<std::chrono::duration<double, std::milli>>(t3 - t1)).count() << std::endl;
//        std::cout << "ProducerTime: " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << std::endl;
    }

}

void ThreadManager::Consume(){
    //！cout<<"consume is run"<<endl;
    while(1)//图像处理，可根据实际需求在其中添加，仅需保证consume处理速度>communicate即可。
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        if(j == i)
        {
            std::unique_lock <std::mutex> lock(mutex);
            condition.wait(lock);
        }
        p_armor_detector_ -> LoadImage(buffer[j]);
        p_communication_ ->UpdateData( p_angle_solver_ ->SolveAngle(  p_armor_detector_ -> DetectObjectArmor() )   );
        //p_communication_ ->UpdateData( p_angle_solver_ ->SolveAngle(p_forecast_->forcast ( p_armor_detector_ -> DetectObjectArmor(),sys_time[j]  )  )   );
        p_communication_ ->shoot_err(p_angle_solver_ ->shoot_get());
        // std::promise<Point2f> shoot;
        // p_run_detector_ -> getShootAim(buffer[i], sys_time[j], shoot);
        //debug
        p_armor_detector_ -> Show();
        //p_armor_detector_ -> baocun();
        if( (++j) % 30 == 0 )
        {
            j = 0;
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        //！std::cout << "ConsumerTime: " << (static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << std::endl;
        //！std::cout << "ConsumerFPS: " << 1000/((static_cast<std::chrono::duration<double, std::milli>>(t2 - t1)).count()) << std::endl;
    }

}

void ThreadManager::Communicate(){ //传递信息就直接修改p_communication_->Infantry中对应的数值即可
    while(1)
    {
      // p_communication_->ref_amorAttackMsg(p_communication_->Infantry.amorAttackmsg.yawErr, p_communication_->Infantry.amorAttackmsg.pitchErr,0, p_communication_->Infantry.amorAttackmsg.shootFlag, p_communication_->Infantry.amorAttackmsg.holderflag, true);
        p_communication_->communication(p_communication_ -> Infantry);

    }


}


