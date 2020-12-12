#include"predictor.h"

namespace horizon{

Predictor::Predictor()
    :
    time_per_tick_(1.0/getTickFrequency()),    
    frame_counter_(0),
    start_tick_(0),
    end_tick(0),
    average_tick_per_frame_(0),
    is_anti_rotation_(false),
    detect_mode_(DetectMode::NONE),
    is_lock_target_(false),
    lost_target_counter_(0)
{};

Predictor::~Predictor(){}





}