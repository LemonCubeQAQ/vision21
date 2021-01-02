#include"rm_vision.h"
#include<thread>
//命名空间以小写字母命名. 最高级命名空间的名字取决于项目名称

namespace horizon{ 

DetectMode RmVision::detect_mode_(DetectMode::ARMOR);
Mat RmVision::image_buffer_[] = {};
int64 RmVision::time_buffer_[] = {};
unsigned  int RmVision::image_buffer_front_(0);
unsigned int RmVision::image_buffer_rear_(0);

RmVision::SerialState RmVision::serial_state_(SerialState::WATING);
RmVision::float2uc RmVision::read_pitch_{};
RmVision::float2uc RmVision::read_yaw_{};
RmVision::float2uc RmVision::read_distance_{};
RmVision::float2uc RmVision::send_pitch_{}; 
RmVision::float2uc RmVision::send_yaw_{};
uchar RmVision::firing_rate_(0);
io_service RmVision::iosev_;
serial_port RmVision::sp_(iosev_, "/dev/base_controller_usb");
mutex RmVision::serial_mutex_;

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
    
        serial_mutex_.lock();
        armordector.ConfigureParameters(read_pitch_.f, read_yaw_.f, firing_rate_, time_buffer_[image_buffer_front_%IMGAE_BUFFER]);
        serial_mutex_.unlock();

        Eigen::Vector3f pos = armordector.GetHitPos(detect_mode_, src);

        //serial_mutex_.lock();
        ConfigureSendData(pos, armordector.IsFindTarget());
        serial_state_ = SerialState::SEND_DATA;
        //serial_mutex_.unlock();
    }
}

void RmVision::Serial(){
    int count_read = 0;
    int count_sent = 0;
    int err_Count = 0;
    // 向串口读数据
    while(true){
//        read(sp_, buffer(read_bytes_));
/*         if(read_bytes_[0] == 0xaa && read_bytes_[READ_BYTES_SIZE-1] == 0xbb){
            serial_mutex_.lock();
            ConfigureReadData();
            serial_mutex_.unlock();
            count_read++;
        }
        else{
            err_Count++;
            cout<<"err: "<<err_Count<<"\n";
            char trash[1] = {0x00};
            do{
                read(sp_, buffer(trash));
            }while(trash[0] != 0xbb);
        }      */
        if(serial_state_ == SerialState::SEND_DATA){
            sent_bytes_[0] = 0xaa;
            sent_bytes_[SENT_BYTES_SIZE-1] = 0xbb;
            write(sp_, buffer(sent_bytes_, SENT_BYTES_SIZE));
            for(int i = 0; i < SENT_BYTES_SIZE-1; i++){
                cout<<hex<<sent_bytes_[i]<<" ";
            }
            cout<<'\n';
            serial_state_ = SerialState::WATING;
            count_sent++;
        }
        cout<<"serial count: "<<dec<<count_read<<" "<<count_sent<<" "<<err_Count<<'\n';
    }
    iosev_.run();
}



}