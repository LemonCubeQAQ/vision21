#include"rm_vision.h"

namespace horizon{ //命名空间以小写字母命名. 最高级命名空间的名字取决于项目名称

int RmVision::i = 0;
std::mutex RmVision::exchange_mutex_;
cv::Mat RmVision::image_;

RmVision::RmVision(){}

RmVision::~RmVision(){}

void RmVision::ImageProducer(){
    while(1){
    exchange_mutex_.lock();
    cout<<i++%10<<endl;
    exchange_mutex_.unlock();
    }
}
void RmVision::ImageConsumer(){
    while(1){
    exchange_mutex_.lock();
    cout<<i++%10<<endl;
    exchange_mutex_.unlock();
    }
}




}