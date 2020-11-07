
#include<thread>
#include"rm_vision.h"
using namespace horizon;

int main(){
    RmVision vision;
    std::thread thread1(&RmVision::ImageProducer, vision);
    std::thread thread2(&RmVision::ImageConsumer, vision);
    
    thread1.join();
    thread2.join();

    return 0;
}