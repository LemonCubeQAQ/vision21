#include"predictor.h"

namespace horizon{

Predictor::Predictor()
    :
    is_anti_rotation_(false),
    detect_mode_(DetectMode::NONE),
    is_lock_target_(false),
    lost_target_counter_(0)
{};

Predictor::~Predictor(){}





}