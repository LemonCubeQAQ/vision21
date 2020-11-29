#ifndef ARMORDECTOR_H_
#define ARMORDECTOR_H_

#include<opencv2/opencv.hpp>

//常规函数使用大小写混合, 取值和设值函数则要求与变量名匹配
//枚举，全大写加下划线

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
       slope_(0), length_(0), 
        top_point_(), bottom_point_(), center_()     
    {};
    ~Led(){};

public:
    float slope_;
    float length_;
    Point2f top_point_;
    Point2f bottom_point_;    
    Point2f center_;
};


//Processing function class
class ArmorDector{
    enum ArraySize{
        LED_SIZE = 20,
        ARMOR_SIZE = 10
    };
    
public:
    ArmorDector(EnemyColor enemy_color);
    ~ArmorDector();

public:
    void GetTarget(DetectMode detect_mode, Mat& src_image, int64& tick);

private:
    bool GetAllTarget();
    bool GetAllRune();
    void SelectArmor();
    void SelectRune();

private:
    void SeparateColor();
    void AdditionalProcess();
    unsigned short GetAllLed();
    unsigned short GetAllArmor(const unsigned short& led_size);
    void  AngleResolving(const unsigned short& armor_num);
    void GetImagePointData(const Armor& allArmor, std::vector<cv::Point2f>& point2D);
    void GetObjectPointData(const Armor& armor, std::vector<cv::Point3f>& point3D);
    void SolveAngle(Armor& armor, const std::vector<cv::Point2f>& point2D, const std::vector<cv::Point3f>& point3D);
private:
    const EnemyColor enemy_color_;
    const double time_per_tick_;    
    short frame_counter_;
    int64 start_tick_;
    int64 end_tick;
    int64 average_tick_per_frame_;

private:
    DetectMode detect_mode_;
    
private:
    Mat src_image_;
    Mat binary_image_;
    Mat aultimate_image_;

private:
    Led led_array_[LED_SIZE];
    unsigned short armor_num_;
    Armor armor_array_[ARMOR_SIZE];

private:
    static bool ledCmp(const Led a, const Led b){
        return a.slope_ < b.slope_; //竖直的优先
    }
};


}

#endif