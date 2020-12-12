#ifndef PREDICTOR_H_
#define PREDICTOR_H_
#include "variables.h"
#include<opencv2/opencv.hpp>
#include<eigen3/Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Dense>

namespace horizon{
using namespace cv;
using namespace Eigen;

class Predictor{
public:
    Predictor(); 
    ~Predictor();
    void  GetArmorData(
        const DetectMode& mode,
        const bool is_anti_rotation,
        const unsigned short& armor_num,
        const Armor& armor_array
    );
    void LostTarget(){lost_target_counter_++;}
    const Vector3f& GetPredictedTarget(){return predicted_target_;}

private:
    const double time_per_tick_;    
    short frame_counter_;
    int64 start_tick_;
    int64 end_tick;
    int64 average_tick_per_frame_;

private:
    bool is_anti_rotation_;
    DetectMode detect_mode_;

private:
    bool is_lock_target_;
    unsigned int lost_target_counter_;

private:
    Vector3f predicted_target_;

};




}



#endif