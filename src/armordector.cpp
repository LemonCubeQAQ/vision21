#ifndef ARMORDECTOR_H_
#define ARMORDECTOR_H

#include "armordector.h"
#include "constants.h"

namespace horizon{
using namespace std;
using namespace constants;


ArmorDector::ArmorDector()
    :
    enemy_color_(EnemyColor::RED),
    time_per_tick_(1.0/getTickFrequency()),    
    frame_counter_(0),
    start_tick_(0),
    end_tick(0),
    average_tick_per_frame_(0),
    src_image_(),
    binary_image_(),
    aultimate_image_()
{};

ArmorDector::~ArmorDector(){};

void ArmorDector::GetTarget(DetectMode detect_mode, cv::Mat& src_image, int64& tick){
    start_tick_ = tick;
    src_image_ = src_image;

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


    //TODO(Yeahoo): Prediction


}

bool ArmorDector::GetAllTarget(){

    SeparateColor();
    AdditionalProcess();

    unsigned short led_num = GetAllLed();
    if(led_num < 2) return false;

    unsigned short armor_num = GetAllArmor();
    if(armor_num == 0) return false;

    AngleResolving();
    
    return true;
}

bool ArmorDector::GetAllRune(){

    return true;
}

void ArmorDector::SelectArmor(){}

void ArmorDector::SelectRune(){}


void ArmorDector::SeparateColor(){
    //TODO: separate color witch you want
    
    //Example is copied from Horizon-2020
    cv::Mat splited_image[3];
    split(src_image_, splited_image);

    float RGrayWeightValue = 97;
    Mat binary_image_t = Mat::zeros(src_image_.size(),CV_8UC1);

    switch(enemy_color_){
        case EnemyColor::RED:{
            unsigned short image_rows = src_image_.rows;
            unsigned short image_cols = src_image_.cols;
            for(unsigned short image_row_index = 0; image_row_index < image_rows; image_row_index++){
                uchar* bin = binary_image_t.ptr<uchar>(image_row_index);
                uchar* r = splited_image[2].ptr<uchar>(image_row_index);
                uchar* g = splited_image[1].ptr<uchar>(image_row_index);
                uchar* b= splited_image[0].ptr<uchar>(image_row_index);

                for(int image_col_index = 0; image_col_index < image_cols; image_col_index++){
                    bin[image_col_index] = b[image_col_index]<(g[image_col_index]+r[image_col_index])*RGrayWeightValue*0.01?255:0;
                }
            }
            binary_image_ = splited_image[2] > 130;
            break;
        }
        case EnemyColor::BLUE:{
            unsigned short image_rows = src_image_.rows;
            unsigned short image_cols = src_image_.cols;
            for(unsigned short image_row_index = 0; image_row_index < image_rows; image_row_index++){
                uchar* bin = binary_image_t.ptr<uchar>(image_row_index);
                uchar* r = splited_image[2].ptr<uchar>(image_row_index);
                uchar* g = splited_image[1].ptr<uchar>(image_row_index);
                uchar* b= splited_image[0].ptr<uchar>(image_row_index);

                for(int image_col_index = 0; image_col_index < image_cols; image_col_index++){
                    bin[image_col_index] = r[image_col_index]<(g[image_col_index]+b[image_col_index])*RGrayWeightValue*0.01?255:0;
                }
            }
            binary_image_ = splited_image[0] > 130;
            break;
        }
    }

    binary_image_ -= binary_image_t;

}

void ArmorDector::AdditionalProcess(){
    blur(binary_image_, binary_image_,Size(5,5));
    medianBlur(binary_image_, binary_image_,3);
}


unsigned short ArmorDector::GetAllLed(){

    vector<std::vector<cv::Point>> led_contours;
    vector<cv::Vec4i> hierarchy;
    RotatedRect led_rect;
    findContours(aultimate_image_, led_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    unsigned short contours_size = static_cast<unsigned short>(led_contours.size());

    for(unsigned short contour_index = 0; contour_index < contours_size; contour_index++){

        vector<cv::Point> & led_contour = led_contours[contour_index];

        double area = contourArea(led_contour);
        if(area < kMinLedArea || area > kMaxLedArea) continue;

        led_rect = minAreaRect(led_contour);

        LED led;
        led_rect.points(led.vertex_);        
        //The order is bottomLeft, topLeft, topRight, bottomRight.clockwise
        const Point2f& bottomLeft = led.vertex_[0];
        const Point2f& topLeft = led.vertex_[1];
        const Point2f& topRight = led.vertex_[2];
        const Point2f& bottomRight = led.vertex_[3];
        led.top_point_ = (topLeft + topRight) * 0.5; 
        led.bottom_point_ = (bottomLeft + bottomRight) * 0.5;
        led.center_ =  (led.top_point_ + led.bottom_point_) * 0.5;



    }



}

unsigned short GetAllArmor(){



}


}
#endif