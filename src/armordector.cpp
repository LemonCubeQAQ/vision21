#ifndef ARMORDECTOR_H_
#define ARMORDECTOR_H

#include "armordector.h"

namespace horizon{


ArmorDector::ArmorDector()
    :   src_image_(),
        binary_image_(),
        aultimate_image_()
{};
ArmorDector::~ArmorDector(){};


void ArmorDector::GetTarget(DetectMode detect_mode, cv::Mat src_image){
    src_image_ = src_image;

    //TODO: pre-processing
    switch(detect_mode){
        case DetectMode::ARMOR:{
            if(GetAllArmor()){
                SelectArmor();
            }
            else{

            }
            break;
        }
        case DetectMode::RUNE:{
            if(GetAllRune()){
                SelectRune();
            }
            else{

            }
            break;
        }
        default:
            break;
    }


    //Prediction

}

bool ArmorDector::GetAllArmor(){
    unsigned short armor_num;
    unsigned short led_num;
    

    SeparateColor();
    AdditionalProcess();




    return true;
}

bool ArmorDector::GetAllRune(){
    unsigned short armor_num;
    unsigned short led_num;
    

    SeparateColor();
    AdditionalProcess();




    return true;
}

void ArmorDector::SeparateColor(){
    binary_image_;
}

void ArmorDector::AdditionalProcess(){
    aultimate_image_;
}


void ArmorDector::SelectArmor(){

}

void ArmorDector::SelectRune(){
    
}


}
#endif