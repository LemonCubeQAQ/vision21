#ifndef RM_VISION_H_
#define RM_VISION_H_
#include<mutex>
#include<iostream>
#include<opencv2/opencv.hpp>

//命名空间以小写字母命名. 最高级命名空间的名字取决于项目名称
//类型名称的每个单词首字母均大写
//普通变量命名 小写字母加下划线
//类数据成员在普通变量基础上后加下划线

namespace horizon{ 
using namespace std;

class RmVision{
public:
    RmVision();
    ~RmVision();
    void ImageProducer(); //生产者
    void ImageConsumer(); //消费者

private: 

private:
    static cv::Mat image_;
    static std::mutex exchange_mutex_; //数据交换锁
};


}
#endif