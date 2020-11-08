#ifndef ARMORDECTOR_H_
#define ARMORDECTOR_H_

//常规函数使用大小写混合, 取值和设值函数则要求与变量名匹配
//枚举，全大写加下划线

namespace horizon{

//Processing function class
//
class ArmorDector{
public:
    ArmorDector();
    ~ArmorDector();
public:
    void GetTarget(DetectMode detect_mode_, cv::Mat src_image);
    


private:


};


}

#endif