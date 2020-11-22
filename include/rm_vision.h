#ifndef RM_VISION_H_
#define RM_VISION_H_

#include"armordector.h"
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
    READ_BYTES_SIZE = 12
};

enum BufferSize{
    IMGAE_BUFFER = 5
};

public:
    RmVision();
    ~RmVision();

public:
    void ImageProducer(); 
    void ImageConsumer(); 
    void Serial();

private:
    DaHengCamera* camera_point_;

private:
    static DetectMode detect_mode_;
    static Mat image_buffer_[IMGAE_BUFFER];
    static int64 time_buffer_[IMGAE_BUFFER];
    static unsigned int image_buffer_front_;
    static unsigned int image_buffer_rear_;

private:
    static io_service iosev_;
    static serial_port sp_;
    unsigned char sent_bytes_[SENT_BYTES_SIZE];
    unsigned char read_bytes_[READ_BYTES_SIZE]; 

};


}
#endif