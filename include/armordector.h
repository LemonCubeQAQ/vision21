#ifndef ARMORDECTOR_H_
#define ARMORDECTOR_H_
#include"predictor.h"
//常规函数使用大小写混合, 取值和设值函数则要求与变量名匹配
//枚举，全大写加下划线

namespace horizon{

//Processing function class
class ArmorDector{
    enum ArraySize{
        LED_SIZE = 20,
        ARMOR_SIZE = 10
    };
    
public:
    ArmorDector();
    ~ArmorDector();

public:
    const Vector3f& GetHitPos(DetectMode detect_mode, Mat& src_image, int64& tick);

private:
    bool GetAllTarget();
    bool GetAllRune();

private:
    void SeparateColor();
    void AdditionalProcess();
    unsigned short GetAllLed();
    unsigned short GetAllArmor(const unsigned short& led_size);
    void  AngleResolving(const unsigned short& armor_num);

private:
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
    Mat src_image_;
    Mat binary_image_;
    Mat aultimate_image_;
    
private:
    unsigned short armor_num_;
    Led led_array_[LED_SIZE];
    Armor armor_array_[ARMOR_SIZE];

private:
    Predictor predictor_;

private:
    //竖直优先
    static bool ledCmpSlope(const Led a, const Led b){
        return abs(a.slope_) < abs(b.slope_); 
    }
    //y优先
    static bool ledCmpY(const Point2f a, const Point2f b){
        return a.y < b.y; 
    }
    //x优先
    static bool ledCmpX(const Point2f a, const Point2f b){
        return a.x < b.x; 
    }
};


}

#endif