#ifndef ARMORDECTOR_H_
#define ARMORDECTOR_H_

#include<opencv2/opencv.hpp>

//常规函数使用大小写混合, 取值和设值函数则要求与变量名匹配
//枚举，全大写加下划线

namespace horizon{
using namespace cv;

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

struct ArmorData{
    ArmorType armor_type;
    cv::Point2f leftLed[2];     //左灯条
    cv::Point2f rightLed[2];        //右灯条

    float tx;       //x 轴偏移量
    float ty;       //y 轴偏移量
    float tz;       //z 轴偏移量
    ArmorData():armor_type(ArmorType::SMALL),tx(0),ty(0),tz(0){};
};

//Processing function class
class ArmorDector{
public:
    ArmorDector();
    ~ArmorDector();

public:
    void GetTarget(DetectMode detect_mode, Mat src_image);

private:
    bool GetAllArmor();
    bool GetAllRune();
    void SelectArmor();
    void SelectRune();    

private:
    void SeparateColor();
    void AdditionalProcess();
private:


private:
    Mat src_image_;
    Mat binary_image_;
    Mat aultimate_image_;

};


}

#endif