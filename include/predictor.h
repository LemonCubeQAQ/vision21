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
    void ConfigureData(const float& pitch, const float& yaw, const float& firing_rate, const int64& tick){
        start_tick_ = tick;
        pitch_ = pitch * trans2theta_;
        yaw_ = yaw * trans2theta_;
        firing_rate_ = firing_rate;
        SetRotationMatrix(pitch, yaw);
    }
    void LostTarget(){
        frame_counter_++;
        total_tick_frames_ += getTickFrequency() - start_tick_;
        if(is_find_target_){
            lost_target_counter_++;
            predicted_target_ = kalman_filter_.Predictor(dur_time_);
            if(lost_target_counter_ > LOST_TARGET){
                frame_counter_ = 0;
                total_tick_frames_ = 0;
                lost_target_counter_ = 0;
                is_find_target_ = false;
            }
        }
    }
    const Vector3f& GetPredictedTarget();

private:
    void AntiRotation(){};
    Vector3f BallisticCalculate(const Vector3f);
    void SetRotationMatrix(const float& pitch, const float& yaw);
private:
    bool is_find_target_;
    short frame_counter_;
    const float time_per_tick_;
    const float trans2theta_;
    int64 start_tick_;
    int64 total_tick_frames_;

private:
    DetectMode detect_mode_;
    unsigned int lost_target_counter_;

private:
    ArmorType locked_target_type_;
    Vector3f locked_target_;
    float firing_rate_;
    float dur_time_;
    float locked_pitch_;
    float locked_yaw_;
    float pitch_;
    float yaw_;

private:
    KalmanFilter kalman_filter_;
    Vector3f predicted_target_;
    Matrix3f pitch_rotation_matrix_;
    Matrix3f yaw_rotation_matrix_;
};




}



#endif