#ifndef RM_VISION_H_
#define RM_VISION_H_
#include"variables.h"
#include"armordector.h"
#include"DaHengCamera.h"
#include<thread>
#include<mutex>
#include<iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

//命名空间以小写字母命名. 最高级命名空间的名字取决于项目名称
//类型名称的每个单词首字母均大写
//普通变量命名 小写字母加下划线
//类数据成员在普通变量基础上后加下划线

namespace horizon{ 
using namespace std;
using namespace cv;
using namespace boost::asio;

// Vision main fuction
// Using imageProducer function to get image
// Using ImageConsumer function to process image
// Passing the parameters to Serial function for communicating with STM32
class RmVision{

enum SerialSize{
    SENT_BYTES_SIZE = 14,
    READ_BYTES_SIZE = 13
};

enum BufferSize{
    IMGAE_BUFFER = 5
};

enum class SerialState{
    READ_DATA,
    SEND_DATA,
    WATING
};

union float2uc{
    float f;
    uchar uc[4];
};

public:
    RmVision();
    ~RmVision();

public:
    void ImageProducer(); 
    void ImageConsumer(); 
    void Serial();

private:
    DaHengCamera* camera_ptr_;

private:
    Mat image_buffer_[IMGAE_BUFFER];
    int64 time_buffer_[IMGAE_BUFFER];
    unsigned int image_buffer_front_;
    unsigned int image_buffer_rear_;

private:
    DetectMode detect_mode_;
    float2uc read_pitch_;
    float2uc read_yaw_;
    float2uc read_distance_;
    float2uc send_pitch_;
    float2uc send_yaw_;
    uchar firing_rate_;

private:
    io_service iosev_;
    serial_port sp_;
    SerialState serial_state_;
    mutex serial_mutex_;
    unsigned char sent_bytes_[SENT_BYTES_SIZE];
    unsigned char read_bytes_[READ_BYTES_SIZE];

private:
    inline void ConfigureSendData(const Eigen::Vector3f& pos, const bool& is_find_target_){
        sent_bytes_[5] = send_pitch_.f < 0.0f ? 0x00:0x01;
        sent_bytes_[10] = send_yaw_.f < 0.0f ? 0x00:0x01;
        send_pitch_.f = pos[0] - read_pitch_.f;
        send_yaw_.f = pos[1] - read_yaw_.f;
    
/*         sent_bytes_[1] = send_pitch_.uc[0];
        sent_bytes_[2] = send_pitch_.uc[1];
        sent_bytes_[3] = send_pitch_.uc[2];
        sent_bytes_[4] = send_pitch_.uc[3];
        sent_bytes_[6] = send_yaw_.uc[0];
        sent_bytes_[7] = send_yaw_.uc[1];
        sent_bytes_[8] = send_yaw_.uc[2];
        sent_bytes_[9] = send_yaw_.uc[3];
        sent_bytes_[11] = is_find_target_;
        sent_bytes_[12] = static_cast<uchar>(pos[2]); */
        sent_bytes_[1] = 1;
        sent_bytes_[2] = 2;
        sent_bytes_[3] = 3;
        sent_bytes_[4] = 4;
        sent_bytes_[5] = 5;        
        sent_bytes_[6] = 6;
        sent_bytes_[7] = 7;
        sent_bytes_[8] = 8;
        sent_bytes_[9] = 9;
        sent_bytes_[10] = 10;
        sent_bytes_[11] = 11;
        sent_bytes_[12] = 12;
    }
    inline void ConfigureReadData(){
        read_pitch_.uc[0] = read_bytes_[1];
        read_pitch_.uc[1] = read_bytes_[2];
        read_pitch_.uc[2] = read_bytes_[3];
        read_pitch_.uc[3] = read_bytes_[4];
        read_yaw_.uc[0] = read_bytes_[6];
        read_yaw_.uc[1] = read_bytes_[7];
        read_yaw_.uc[2] = read_bytes_[8];
        read_yaw_.uc[3] = read_bytes_[9];
        firing_rate_ = read_bytes_[11];
        cout<<"read: \npitch: "<<read_pitch_.f<<" yaw "<<read_yaw_.f<<" "<<firing_rate_<<endl;
    }
};


}
#endif