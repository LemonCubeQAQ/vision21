#include"rm_vision.h"
#include<thread>
//命名空间以小写字母命名. 最高级命名空间的名字取决于项目名称

namespace horizon{ 

DetectMode RmVision::detect_mode_(DetectMode::ARMOR);
Mat RmVision::image_buffer_[] = {};
int64 RmVision::time_buffer_[] = {};
unsigned  int RmVision::image_buffer_front_(0);
unsigned int RmVision::image_buffer_rear_(0);

RmVision::SerialState RmVision::serial_state_(SerialState::WAIT_DATA);
RmVision::float2uc RmVision::read_pitch_{};
RmVision::float2uc RmVision::read_yaw_{};
RmVision::float2uc RmVision::read_distance_{};
RmVision::float2uc RmVision::send_pitch_{}; 
RmVision::float2uc RmVision::send_yaw_{};
RmVision::float2uc RmVision::send_distance_{};
io_service RmVision::iosev_;
serial_port RmVision::sp_(iosev_, "/dev/base_controller_usb");

RmVision::RmVision()
    :   
    camera_ptr_(nullptr),
    sent_bytes_{},  
    read_bytes_{}{
    sp_.set_option(serial_port::baud_rate(115200));                              // 设置波特率
    sp_.set_option(serial_port::flow_control(serial_port::flow_control::none));  // 设置控制方式
    sp_.set_option(serial_port::parity(serial_port::parity::none));              // 设置奇偶校验
    sp_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));      // 设置停止位
    sp_.set_option(serial_port::character_size(8)); // 设置字母位数为8位
}

RmVision::~RmVision(){
    if(camera_ptr_ != nullptr){
        delete camera_ptr_;
        camera_ptr_ = nullptr;
    }
}

void RmVision::ImageProducer(){
    while(true){

        switch(detect_mode_){
            case DetectMode::ARMOR:{
                if(camera_ptr_ != nullptr){
                    while(image_buffer_rear_ - image_buffer_front_ > IMGAE_BUFFER);
                    if(camera_ptr_->GetMat(image_buffer_[image_buffer_rear_%IMGAE_BUFFER])){
                        time_buffer_[image_buffer_rear_%IMGAE_BUFFER] = getTickCount();       
                        ++image_buffer_rear_;
                    }
                    else{
                        delete camera_ptr_;
                        camera_ptr_ = nullptr;
                    }
                }
                else{
                    camera_ptr_ = new DaHengCamera;
                    while(!camera_ptr_->StartDevice());
                    camera_ptr_->SetResolution();
                    while(!camera_ptr_->StreamOn());
                    camera_ptr_->SetExposureTime();
                    //设置曝光增益000
                    camera_ptr_->SetGain();
                    //设置是否自动白平衡
                    camera_ptr_->Set_BALANCE_AUTO(1);
                    //手动设置白平衡通道及系数，此之前需关闭自动白平衡
                    // camera.Set_BALANCE(0,40);
                    camera_ptr_->SetExposureTime(constants::kExposureTime);
                    camera_ptr_->SetGain(3, constants::kExposureGain);
                    image_buffer_front_ = 0;
                    image_buffer_rear_ = 0;                
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
    ArmorDector armordector;

    while(true){

        while(image_buffer_rear_ <= image_buffer_front_);
        image_buffer_front_ = image_buffer_rear_-1;
        //Recieve serial information
        Mat& src = image_buffer_[image_buffer_front_%IMGAE_BUFFER];
        Mat src_show = src.clone();
        cv::imshow("src", src_show);
        cv::waitKey(1);

        armordector.ConfigureParameters(send_pitch_.f, send_yaw_.f, time_buffer_[image_buffer_front_%IMGAE_BUFFER]);
        Eigen::Vector3f pos = armordector.GetHitPos(detect_mode_, src);

        while(serial_state_ == SerialState::SEND_DATA);
        ConfigureSendData();
        serial_state_ == SerialState::SEND_DATA;

    }
}

void RmVision::Serial(){

    // 向串口读数据
    while(true){
        read(sp_, buffer(read_bytes_));

        if(read_bytes_[0] == 0xaa && read_bytes_[READ_BYTES_SIZE-1] == 0xbb){
            //TODO(YeahooQAQ): get the data from the read_bytes_
            ConfigureReadData();
        }
        else{
            char trash[1] = {0x00};
            for(int i = 0; i < READ_BYTES_SIZE; i++){ cout<<hex<<read_bytes_[i]<<" "; }
            do{
                read(sp_, buffer(trash));
            }while(trash[0] != 0xbb);
        }

        if(serial_state_ == SerialState::SEND_DATA){
            write(sp_, buffer(sent_bytes_, SENT_BYTES_SIZE));
            serial_state_ = SerialState::WAIT_DATA;
        }
        iosev_.run();
    }
}



}