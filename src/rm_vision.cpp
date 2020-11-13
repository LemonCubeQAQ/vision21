#include"debug.h"
#include"constants.h"
#include"rm_vision.h"
#include"DaHengCamera.h"

#include<thread>
//命名空间以小写字母命名. 最高级命名空间的名字取决于项目名称

namespace horizon{ 

DetectMode RmVision::detect_mode_(DetectMode::ARMOR);
unsigned  int RmVision::image_buffer_front_(0);
unsigned int RmVision::image_buffer_rear_(0);
cv::Mat RmVision::image_buffer_[5] = {};

io_service RmVision::iosev_;
serial_port RmVision::sp_(iosev_, "/dev/base_controller_usb");

RmVision::RmVision(){
    sp_.set_option(serial_port::baud_rate(115200));                              // 设置波特率
    sp_.set_option(serial_port::flow_control(serial_port::flow_control::none));  // 设置控制方式
    sp_.set_option(serial_port::parity(serial_port::parity::none));              // 设置奇偶校验
    sp_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));      // 设置停止位
    sp_.set_option(serial_port::character_size(8)); // 设置字母位数为8位

}

RmVision::~RmVision(){}


void RmVision::ImageProducer(){

#ifndef DEBUG_

    DaHengCamera Camera;
init_camera:
    Camera.SetResolution();
    Camera.SetExposureTime();
    //设置曝光增益000
    Camera.SetGain();
    //设置是否自动白平衡
    Camera.Set_BALANCE_AUTO(0);
    //手动设置白平衡通道及系数，此之前需关闭自动白平衡
    // camera.Set_BALANCE(0,40);
    Camera.SetExposureTime(constants::kExposureTime);
    Camera.SetGain(3, constants::kExposureGain);

    while(!Camera.StreamOn());

    while(true){

        switch(vision_mode_){
            case VisionTarget::ARMOR:{
                while(image_buffer_rear_ - image_buffer_front_ < 5);

                if(!Camera.GetMat(image_buffer_[image_buffer_rear_%5])){
                    Camera.~DaHengCamera();
                    goto init_camera;                  
                }
                ++image_buffer_rear_;
                break;
            }
            case VisionTarget::RUME:{

                break;
            }
            default:{
                break;
            }
        }

    }

#else

    DaHengCamera Camera;
init_camera:
    image_buffer_front_ = 0;
    image_buffer_rear_ = 0;
    Camera.SetResolution();
    Camera.SetExposureTime();
    //设置曝光增益000
    Camera.SetGain();
    //设置是否自动白平衡
    Camera.Set_BALANCE_AUTO(0);
    //手动设置白平衡通道及系数，此之前需关闭自动白平衡
    // camera.Set_BALANCE(0,40);
    Camera.SetExposureTime(constants::kExposureTime);
    Camera.SetGain(3, constants::kExposureGain);

    while(!Camera.StreamOn());

    while(true){

        switch(detect_mode_){
            case DetectMode::ARMOR:{
                while(image_buffer_rear_ - image_buffer_front_ < 5);

                if(!Camera.GetMat(image_buffer_[image_buffer_rear_%5])){
                    Camera.~DaHengCamera();
                    goto init_camera;                  
                }
                ++image_buffer_rear_;
                break;
            }
            case DetectMode::RUNE:{

                break;
            }
            default:{
                break;
            }
        }

    }


#endif

}

void RmVision::ImageConsumer(){

#ifdef DEBUG_
    ArmorDector armordector;

    while(true){

    while(image_buffer_rear_ <= image_buffer_front_);


        //TODO: Recieve serial information
        armordector.GetTarget(detect_mode_, image_buffer_[image_buffer_front_%5]);
        ++image_buffer_front_;

        //TODO: Sent data
 

    }

#else 

#ifdef SHOW_DEBUG_MESSAGES_

#endif

    ArmorDector armordector;

    while(image_buffer_rear_ <= image_buffer_front_);

    armordector.GetTarget(detect_mode_, image_buffer_[image_buffer_front_%5]);
    ++image_buffer_front_;



#endif
}

void RmVision::Serial(){
/*     write(sp, buffer("Hello world", 12)); */

    // 向串口读数据
    while(true){
        char buf[1];
        read(sp_, buffer(read_bytes));
        for(int i = 0; i < 14;i++){
            cout<<hex<<read_bytes[i]<<" ";
        }
        cout<<endl;
        iosev_.run();
    }
}



}