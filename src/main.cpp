#include<thread>
#include"rm_vision.h"
using namespace horizon;

int main(){
    RmVision vision;
    std::thread thread1(RmVision::ImageProducer, std::ref(vision));
    std::thread thread2(RmVision::ImageConsumer, std::ref(vision));
    std::thread thread3(RmVision::Serial, std::ref(vision));

    thread1.join();
    thread2.join();
    thread3.join();
    return 0;
}