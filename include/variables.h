#ifndef VARIABLES_H_
#define VARIABLES_H_

#include <opencv2/opencv.hpp>
//在程序运行期间其值始终保持不变的, 命名时以 “k” 开头, 大小写混合

namespace horizon{
using namespace cv;

//敌方颜色
enum class EnemyColor{
    RED,
    BLUE
};
//射击模式
enum class DetectMode{
    RUNE,
    ARMOR, 
    NONE
};

//装甲板模式
enum class ArmorType{
    SMALL, //小装甲
    LARGE, //大装甲
    RUNE //大符
};
//预测状态

class Armor{
public:
    Armor()
        :   
        armor_type_(ArmorType::SMALL),
        tx_(0), ty_(0), tz_(0),
        left_led_{}, right_led_{}
    {};
    ~Armor(){};
public:
    ArmorType armor_type_;
    Point2f left_led_[2];     //左灯条
    Point2f right_led_[2];        //右灯条

    float tx_;       //x 轴偏移量
    float ty_;       //y 轴偏移量
    float tz_;       //z 轴偏移量

};

class Led{
public:
    Led()
       :     
       slope_(0), length_(0), width_(0),ratio_(0),
        top_point_(), bottom_point_(), center_()     
    {};
    ~Led(){};

public:
    float slope_;
    float length_;
    float width_;
    float ratio_;
    Point2f top_point_;
    Point2f bottom_point_;    
    Point2f center_;
};
namespace constants{

const EnemyColor kEnemyColor = EnemyColor::RED;

const int kExposureTime = 8000;
const int kExposureGain = 3000;

const int kMinLedArea = 50;
const int kMaxLedArea = 3e3;

const float kLedMinHeightVSWidth = 1.7f;
const float kLedMaxHeightVSWidth = 11.0f;
const float kLedMaxSlope = 0.8f;

const float kLedMinLengthRatio = 0.95f;
const float kLedMaxlengthRatio = 1.05f;

const float kLedSlopeDelta = 0.01f;
const float kLedRatioDelta = 0.01f;
const float kArmorCenterSlope = 3.0f;
const float kAromorWidthCmpLedWidth = 15.0f;
const int kLedMaxRatioWidthCmpHeight = 6;
const int kArmorTypeThreshold = 5;

const float kRealSmallArmorWidth = 13.5f;
const float kRealSmallArmorHeight = 5.5f;
const float kRealLargeArmorWidth = 22.5f;
const float kRealLargeArmorHeight = 5.5f;
const float kRealRuneWidth = 24.0f;
const float kRealRuneHeight = 18.0f;


//相机内参
static const cv::Mat caremaMatrix_shoot = (
        cv::Mat_<float>(3, 3) << 648.4910,                  0,                         328.2316,
                                                        0,                                     652.0198,         254.6992,
                                                        0,                                     0,                                          1
                                  );
    //畸变参数
static const cv::Mat distCoeffs_shoot = (
        cv::Mat_<float>(1, 5) <<-0.2515, 
                                                        0.2977,
                                                        0,
                                                        0,
                                                        0);  
}




}

#endif