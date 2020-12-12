#ifndef RM_VISION_H_
#define RM_VISION_H_
#include"debug.h"
#include"variables.h"
#include"armordector.h"
#include"DaHengCamera.h"
#include<thread>
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
    READ_BYTES_SIZE = 12
};

enum BufferSize{
    IMGAE_BUFFER = 5
};

enum class SerialState{
    WAIT_DATA,
    SEND_DATA
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
    static Mat image_buffer_[IMGAE_BUFFER];
    static int64 time_buffer_[IMGAE_BUFFER];
    static unsigned int image_buffer_front_;
    static unsigned int image_buffer_rear_;

private:
    static SerialState serial_state_;
    static DetectMode detect_mode_;
    static float2uc read_pitch_;
    static float2uc read_yaw_;
    static float2uc read_distance_;
    static float2uc send_pitch_;
    static float2uc send_yaw_;
    static float2uc send_distance_;

private:
    static io_service iosev_;
    static serial_port sp_;
    unsigned char sent_bytes_[SENT_BYTES_SIZE];
    unsigned char read_bytes_[READ_BYTES_SIZE];

private:
    inline void ConfigureSendData(){
        sent_bytes_[1] = send_pitch_.uc[0];
        sent_bytes_[2] = send_pitch_.uc[1];
        sent_bytes_[3] = send_pitch_.uc[2];
        sent_bytes_[4] = send_pitch_.uc[3];
        sent_bytes_[5] = send_yaw_.uc[0];
        sent_bytes_[6] = send_yaw_.uc[1];
        sent_bytes_[7] = send_yaw_.uc[2];
        sent_bytes_[8] = send_yaw_.uc[3];
        sent_bytes_[9] = send_distance_.uc[0];
        sent_bytes_[10] = send_distance_.uc[1];
        sent_bytes_[11] = send_distance_.uc[2];
        sent_bytes_[12] = send_distance_.uc[3];
    }
    inline void ConfigureReadData(){
        read_pitch_.uc[0] = read_bytes_[1];
        read_pitch_.uc[1] = read_bytes_[2];
        read_pitch_.uc[2] = read_bytes_[3];
        read_pitch_.uc[3] = read_bytes_[4];
        read_yaw_.uc[0] = read_bytes_[5];
        read_yaw_.uc[1] = read_bytes_[6];
        read_yaw_.uc[2] = read_bytes_[7];
        read_yaw_.uc[3] = read_bytes_[8];
        read_distance_.uc[0] = read_bytes_[9];
        read_distance_.uc[1] = read_bytes_[10];
        read_distance_.uc[2] = read_bytes_[11];
        read_distance_.uc[3] = read_bytes_[12];
    }
};


}
#endif