#ifndef RM_VISION_H_
#define RM_VISION_H_

#include"armordector.h"
#include<mutex>
#include<iostream>
#include<opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

//命名空间以小写字母命名. 最高级命名空间的名字取决于项目名称
//类型名称的每个单词首字母均大写
//普通变量命名 小写字母加下划线
//类数据成员在普通变量基础上后加下划线

namespace horizon{ 
using namespace std;
using namespace boost::asio;

enum class DetectMode{
    RUME,
    ARMOR
};

// Vision main fuction
// Using imageProducer function to get image
// Using ImageConsumer function to process image
// Passing the parameters to Serial function for communicating with STM32
class RmVision{

public:
    RmVision();
    ~RmVision();

public:
    void ImageProducer(); 
    void ImageConsumer(); 
    void Serial();

private:
    static DetectMode detect_mode_;
    static cv::Mat image_buffer_[5];
    static unsigned int image_buffer_front_;
    static unsigned int image_buffer_rear_;

private:
    static io_service iosev_;
    static serial_port sp_;
    unsigned char sent_bytes[14];
    unsigned char read_bytes[12]; 

};


}
#endif