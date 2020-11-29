#include"debug.h"
#include"constants.h"
#include"rm_vision.h"
#include"DaHengCamera.h"

#include<thread>
//命名空间以小写字母命名. 最高级命名空间的名字取决于项目名称

namespace horizon{ 

DetectMode RmVision::detect_mode_(DetectMode::ARMOR);
Mat RmVision::image_buffer_[] = {};
int64 RmVision::time_buffer_[] = {};
unsigned  int RmVision::image_buffer_front_(0);
unsigned int RmVision::image_buffer_rear_(0);

io_service RmVision::iosev_;
serial_port RmVision::sp_(iosev_, "/dev/base_controller_usb");

RmVision::RmVision()
    :   
    camera_point_(nullptr),
    sent_bytes_{},  
    read_bytes_{}{
    sp_.set_option(serial_port::baud_rate(115200));                              // 设置波特率
    sp_.set_option(serial_port::flow_control(serial_port::flow_control::none));  // 设置控制方式
    sp_.set_option(serial_port::parity(serial_port::parity::none));              // 设置奇偶校验
    sp_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));      // 设置停止位
    sp_.set_option(serial_port::character_size(8)); // 设置字母位数为8位
}

RmVision::~RmVision(){}

void RmVision::ImageProducer(){

    while(true){

        switch(detect_mode_){
            case DetectMode::ARMOR:{
                if(camera_point_ = nullptr){
                    image_buffer_front_ = 0;
                    image_buffer_rear_ = 0;                    
                    camera_point_ = new DaHengCamera;
                    camera_point_->SetResolution();
                    camera_point_->SetExposureTime();
                    //设置曝光增益000
                    camera_point_->SetGain();
                    //设置是否自动白平衡
                    camera_point_->Set_BALANCE_AUTO(0);
                    //手动设置白平衡通道及系数，此之前需关闭自动白平衡
                    // camera.Set_BALANCE(0,40);
                    camera_point_->SetExposureTime(constants::kExposureTime);
                    camera_point_->SetGain(3, constants::kExposureGain);

                    while(camera_point_->StreamOn());
                    
                }
                else{

                    while(image_buffer_rear_ - image_buffer_front_ > IMGAE_BUFFER);

                    if(camera_point_->GetMat(image_buffer_[image_buffer_rear_%IMGAE_BUFFER])){
                        time_buffer_[image_buffer_rear_%IMGAE_BUFFER] = getTickCount();       
                        ++image_buffer_rear_;                                 
                    }
                    else{
                        delete camera_point_;
                        camera_point_ = nullptr;
                    }

                }
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

}

void RmVision::ImageConsumer(){

    //TODO(YeahooQAQ): I need to fix it that Enemycolor should be easy to configure before contest 
    ArmorDector armordector(EnemyColor::RED);

    while(true){

        while(image_buffer_rear_ <= image_buffer_front_);

        //Recieve serial information
        unsigned short buffer_index = image_buffer_front_;
        Mat src = image_buffer_[image_buffer_front_%IMGAE_BUFFER].clone();
        ++image_buffer_front_;  

        //TODO(YeahooQAQ): If you want use ROI, just do it here and 

        armordector.GetTarget(detect_mode_, src, time_buffer_[image_buffer_front_%IMGAE_BUFFER]);

        //TODO(YeahooQAQ): Sent data
 

    }


}

void RmVision::Serial(){
/*     write(sp, buffer("Hello world", 12)); */

    // 向串口读数据
    while(true){
        char buf[1];
        read(sp_, buffer(read_bytes_));
        for(int i = 0; i < 14;i++){
            cout<<hex<<read_bytes_[i]<<" ";
        }
        cout<<endl;
        iosev_.run();
    }
}



}