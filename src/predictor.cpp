#include"predictor.h"

namespace horizon{
 
Predictor::Predictor()
    :
    time_per_tick_(1.0/getTickFrequency()),    
    is_find_target_(false),
    frame_counter_(0),
    start_tick_(0),
    end_tick(0),
    average_tick_per_frame_(0),
    pitch_(0.0f),
    yaw_(0.0f),
    detect_mode_(DetectMode::NONE),
    lost_target_counter_(0),
    last_target_type_(ArmorType::NONE),
    last_target_{},
    predicted_target_{}
{};

Predictor::~Predictor(){}

void  Predictor::GetArmorData(const Armor armor_array[], const unsigned short& armor_num){
    if(is_find_target_){

    }
    else{

    }
}
void  Predictor::GetRuneData(const Armor armor_array[], const unsigned short& armor_num){

}







}