#include "Main/headfiles.h"
#include "ThreadManager/thread_manager.h"

int main()
{
    ThreadManager thread_manager;
    thread_manager.Init();

    std::thread producer( &ThreadManager::Produce,&thread_manager );

    std::thread consumer( &ThreadManager::Consume,&thread_manager );
    std::thread communication( &ThreadManager::Communicate,&thread_manager );

    producer.join();

    consumer.join();
    communication.join();


    return 0;
}
