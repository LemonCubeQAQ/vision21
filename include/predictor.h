#ifndef PREDICTOR_H_
#define PREDICTOR_H_
#include"variables.h"
#include"kalman_filter.h"
#include<opencv2/opencv.hpp>

namespace horizon{
using namespace cv;
using namespace Eigen;

class Predictor{
    enum THRESHOLD{
        LOST_TARGET = 10
    };
public:
    Predictor(); 
    ~Predictor();
    void  GetArmorData(const Armor armor_array[], const unsigned short& armor_num);
    void  GetRuneData(const Armor armor_array[], const unsigned short& armor_num);
    void ConfigureData(const float& pitch, const float& yaw, const int64& tick){
        pitch_ = pitch;
        yaw_ = yaw;
        start_tick_ = tick;
    }
    void LostTarget(){
        //kalman_filter_.Predictor();
        lost_target_counter_++;
        if(lost_target_counter_ > LOST_TARGET){
            lost_target_counter_ = 0;
            is_find_target_ = false;
        }   
    }
    const Vector3f& GetPredictedTarget(){frame_counter_++; return predicted_target_;}

private:
    void AntiRotation(){};
    Vector3f BallisticCalculate(const Vector3f);

private:
    bool is_find_target_;
    short frame_counter_;
    const double time_per_tick_;
    int64 start_tick_;
    int64 end_tick;
    int64 average_tick_per_frame_;

private:
    DetectMode detect_mode_;
    unsigned int lost_target_counter_;

private:
    float last_pitch_;
    float last_yaw_;
    float pitch_;
    float yaw_;

private:
    KalmanFilter kalman_filter_;
    ArmorType last_target_type_;
    Vector3f last_target_;
    Vector3f predicted_target_;

};




}



#endif