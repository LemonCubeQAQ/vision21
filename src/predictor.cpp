#include"predictor.h"
#include<iostream>
namespace horizon{
 using namespace std;

Predictor::Predictor()
    :
    is_find_target_(false),
    frame_counter_(0),
    time_per_tick_(1.0/getTickFrequency()),    
    trans2theta_(0.0174532925199f),
    start_tick_(0),
    total_tick_frames_(0),
    detect_mode_(DetectMode::NONE),
    lost_target_counter_(0),
    locked_target_type_(ArmorType::NONE),
    locked_target_{},
    firing_rate_(6.0f),
    dur_time_(0.0f),
    locked_pitch_(0.0f),
    locked_yaw_(0.0f),
    pitch_(0.0f),
    yaw_(0.0f),
    kalman_filter_(),
    predicted_target_(),
    pitch_rotation_matrix_(),
    yaw_rotation_matrix_()
{}

Predictor::~Predictor(){}

void  Predictor::GetArmorData(const Armor armor_array[], const unsigned short& armor_num){
    if(is_find_target_){
        unsigned short selected_index = 0;
        float min_distance = 0x7F800000;
        cout<<"init: "<<min_distance;
        for(int index = 0; index < armor_num; index++){ 
            const Armor& armor = armor_array[index];
            if(armor.armor_type_ == locked_target_type_){
                Vector3f transed_pos = pitch_rotation_matrix_ * yaw_rotation_matrix_ * armor.pos;
                Vector3f delta_pos = transed_pos - locked_target_;
                float distance = sqrt(delta_pos[0] * delta_pos[0] + delta_pos[1] * delta_pos[1] + delta_pos[2] *  delta_pos[2]);
                if(distance < min_distance){
                    min_distance = distance;
                    selected_index = index;
                }
            }
        }
        frame_counter_++;
        total_tick_frames_ += getTickFrequency() - start_tick_;
        dur_time_ = total_tick_frames_ * time_per_tick_ / frame_counter_ + min_distance / firing_rate_;
        if(min_distance == 0x7F800000){
            LostTarget();
        }
        else{
            lost_target_counter_ = 0;
            locked_target_ = pitch_rotation_matrix_ * yaw_rotation_matrix_ * armor_array[selected_index].pos;
            predicted_target_ = kalman_filter_.Predictor(locked_target_, dur_time_);
        }
    }
    else{
        unsigned short selected_index = 0;
        const Vector3f& pos = armor_array[0].pos;
        float min_distance = sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] *  pos[2]);
        for(int index = 1; index < armor_num; index++){ 
            const Vector3f& pos_t = armor_array[index].pos;
            float distance = sqrt(pos_t[0] * pos_t[0] + pos_t[1] * pos_t[1] + pos_t[2] *  pos_t[2]);
            if(distance < min_distance){
                min_distance = distance;
                selected_index = index;
            }
        }
        is_find_target_ = true;
        locked_target_type_ = armor_array[selected_index].armor_type_;
        locked_pitch_ = pitch_;
        locked_yaw_ = yaw_;
        locked_target_ = pitch_rotation_matrix_ * armor_array[selected_index].pos;
        predicted_target_ = locked_target_;
        kalman_filter_.Initialize(locked_target_);
        frame_counter_++;
        total_tick_frames_ += getTickFrequency() - start_tick_;
        dur_time_ = total_tick_frames_ * time_per_tick_ / frame_counter_ + min_distance / firing_rate_;
    }
}
void  Predictor::GetRuneData(const Armor armor_array[], const unsigned short& armor_num){

}

void Predictor::SetRotationMatrix(const float& pitch, const float& yaw){
    pitch_rotation_matrix_ 
    <<
    1,  0,                  0,
    0,  cos(pitch), sin(pitch),
    0,  -sin(pitch), cos(pitch);

    yaw_rotation_matrix_
    <<
    cos(locked_yaw_ - yaw_), 0, sin(locked_yaw_ - yaw_),
    0,                                            1, 0,
    -sin(locked_yaw_ - yaw_), 0, cos(locked_yaw_ - yaw_);

}

const Vector3f Predictor::GetPredictedTarget(){
    predicted_target_ =  yaw_rotation_matrix_.transpose() * pitch_rotation_matrix_.transpose() * predicted_target_;
    predicted_target_ = pitch_rotation_matrix_ * predicted_target_;
    float h = predicted_target_[1] + 4.9f * dur_time_ * dur_time_;
    float s = sqrt(predicted_target_[0] * predicted_target_[0] + predicted_target_[2] *  predicted_target_[2]);
    
    float pitch =atan(h/s);
    float yaw = atan(predicted_target_[0]/predicted_target_[2]);

    return Vector3f(pitch, yaw, s);
}





}