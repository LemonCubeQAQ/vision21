#ifndef ARMORDECTOR_H_
#define ARMORDECTOR_H

#include "armordector.h"
#include "constants.h"

namespace horizon{
using namespace std;

ArmorDector::ArmorDector(EnemyColor enemy_color)
    :
    enemy_color_(enemy_color),
    time_per_tick_(1.0/getTickFrequency()),    
    frame_counter_(0),
    start_tick_(0),
    end_tick(0),
    average_tick_per_frame_(0),
    detect_mode_(DetectMode::NONE),
    src_image_(),
    binary_image_(),
    aultimate_image_(), 
    led_array_{},
    armor_num_(0),
    armor_array_{}{



};

ArmorDector::~ArmorDector(){};

void ArmorDector::GetTarget(DetectMode detect_mode, cv::Mat& src_image, int64& tick){
    start_tick_ = tick;
    src_image_ = src_image;

    switch(detect_mode){
        case DetectMode::ARMOR:{
            if(GetAllTarget()){
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


    //TODO(YeahooQAQ): Prediction


}

bool ArmorDector::GetAllTarget(){

    SeparateColor();
    AdditionalProcess();

    unsigned short led_num = GetAllLed();
    if(led_num < 2) return false;

    unsigned short armor_num = GetAllArmor(led_num);
    if(armor_num == 0) return false;

    AngleResolving(armor_num);
    armor_num_ = armor_num;
    return true;
}

bool ArmorDector::GetAllRune(){

    return true;
}

void ArmorDector::SelectArmor(){





}

void ArmorDector::SelectRune(){}


void ArmorDector::SeparateColor(){
    //TODO: separate color witch you want
    
    //Example is copied from Horizon-2020
    cv::Mat splited_image[3];
    split(src_image_, splited_image);

    float RGrayWeightValue = 97;
    Mat binary_image_t = Mat::zeros(src_image_.size(), CV_8UC1);

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
    unsigned short led_num= 0;

    for(unsigned short contour_index = 0; contour_index < contours_size && led_num < LED_SIZE; contour_index++){

        vector<cv::Point> & led_contour = led_contours[contour_index];

        double area = contourArea(led_contour);
        if(area < constants::kMinLedArea || area > constants::kMaxLedArea) continue;

        led_rect = minAreaRect(led_contour);

        Led led;
        //TODO(YeahooQAQ): Maybe I should add vertex to the LED data
        Point2f vertex[4];    
        led_rect.points(vertex);        
        //The order is bottomLeft, topLeft, topRight, bottomRight.clockwise
        const Point2f& bottomLeft = vertex[0];
        const Point2f& topLeft = vertex[1];
        const Point2f& topRight = vertex[2];
        const Point2f& bottomRight = vertex[3];
        led.top_point_ = (topLeft + topRight) * 0.5f; 
        led.bottom_point_ = (bottomLeft + bottomRight) * 0.5f;
        led.center_ =  (led.top_point_ + led.bottom_point_) * 0.5f;

        Point2f point_delta_vertical = led.top_point_ - led.bottom_point_;
        led.length_ = sqrt(point_delta_vertical.x * point_delta_vertical.x + point_delta_vertical.y * point_delta_vertical.y);
        led.slope_ = point_delta_vertical.x / point_delta_vertical.y;
        Point2f point_delta_horizontal = (topRight + bottomRight - topLeft - bottomLeft) *0.5f;
        float led_width = sqrt(point_delta_horizontal.x * point_delta_horizontal.x + point_delta_horizontal.y * point_delta_horizontal.y);
        float led_heightVSwidth = led.length_ / led_width;

        if( led_heightVSwidth > constants::kLedMaxHeightVSWidth 
            || led_heightVSwidth < constants::kLedMinHeightVSWidth
            || led.slope_ > constants::kLedMaxSlope
        ) continue;

        led_array_[led_num++] = led;

    }

    return led_num;
}

unsigned short ArmorDector::GetAllArmor(const unsigned short& led_num){

    unsigned short armor_num = 0;
    bool is_selected[led_num];
    //some problems
    memset(is_selected, false, led_num);
    sort(led_array_, led_array_ + led_num, ledCmp);

    for(unsigned short i = 0; i < led_num; i++){
        for(unsigned short j = i + 1; j < led_num; j++){
            if(is_selected[i] || is_selected[j]) continue;
            
            const Led& led1 = led_array_[i];
            const Led& led2 = led_array_[j];
            float length_ratio = led1.length_ / led2.length_;
            if(length_ratio < constants::kLedMinLengthRatio || length_ratio > constants::kLedMaxlengthRatio) continue;
            if(abs(led1.slope_ - led2.slope_) > constants::kLedSlopeDelta) continue;
            
            Point2i center_delta = led1.center_ - led2.center_;
            int center_distance = sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y);

            if(abs(center_delta.x) > abs(center_delta.y) 
                || abs(center_delta.x) * constants::kLedMaxRatioWidthCmpHeight < abs(center_delta.y)
            ) continue;
            is_selected[i] = true;
            is_selected[j] = true; 
            int RatioWidthCmpHeight = 2 * center_distance / (led1.length_ + led2.length_);

            Armor& armor = armor_array_[armor_num];
            if(RatioWidthCmpHeight < constants::kArmorThreshold){
                armor.armor_type_ = ArmorType::SMALL;
            }
            else{
                armor.armor_type_ = ArmorType::LARGE;
            }
            armor.left_led_[0] = led1.top_point_;
            armor.left_led_[1] = led1.bottom_point_;
            armor.right_led_[0] = led2.top_point_;
            armor.right_led_[1] = led2.bottom_point_;

            if(led1.center_.x > led2.center_.x){
                swap(armor.left_led_, armor.right_led_);
            }

            armor_num++;
            break;
        }
        if(armor_num == ARMOR_SIZE) break;
    }
    return armor_num;
}

void  ArmorDector::AngleResolving(const unsigned short& armor_num){
    vector<cv::Point2f> image_coodinate_points;
    vector<cv::Point3f> object_coodinate_points;

    for(unsigned short index = 0; index < armor_num; index++){
        GetImagePointData(armor_array_[index], image_coodinate_points);
        GetObjectPointData(armor_array_[index], object_coodinate_points);
        SolveAngle(armor_array_[index], image_coodinate_points, object_coodinate_points);
    }


}

inline void ArmorDector::GetImagePointData(const Armor & armor, std::vector<cv::Point2f> & point2D){
    point2D.push_back(armor.left_led_[0]);
    point2D.push_back(armor.right_led_[0]);
    point2D.push_back(armor.right_led_[1]);
    point2D.push_back(armor.left_led_[1]);
}

inline void ArmorDector::GetObjectPointData(const Armor & armor, std::vector<cv::Point3f> & point3D){
    float fHalfX;
    float fHalfY;

    switch(armor.armor_type_){
        case ArmorType::SMALL:{
            fHalfX = constants::kRealSmallArmorWidth * 0.5f;
            fHalfY = constants::kRealSmallArmorHeight * 0.5f;
            break;
        }
        case ArmorType::LARGE:{
            fHalfX = constants::kRealLargeArmorWidth * 0.5f;
            fHalfY = constants::kRealLargeArmorHeight * 0.5f;
            break;
        }
        case ArmorType::RUNE:{
            fHalfX = constants::kRealRuneWidth * 0.5f;
            fHalfY = constants::kRealRuneHeight * 0.5f;
            break;
        }
        default:
            break;
    }

    point3D.push_back(cv::Point3f(-fHalfX,fHalfY, 0.0f));
    point3D.push_back(cv::Point3f(fHalfX,fHalfY, 0.0f));
    point3D.push_back(cv::Point3f(fHalfX,-fHalfY, 0.0f));
    point3D.push_back(cv::Point3f(-fHalfX,-fHalfY, 0.0f));

}
void ArmorDector::SolveAngle(Armor& armor, const std::vector<cv::Point2f>& point2D, const std::vector<cv::Point3f>& point3D){

    cv::Mat rvecs = cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat tvecs = cv::Mat::zeros(3,1,CV_64FC1);
    double tx;
    double ty;
    double tz;

    switch(detect_mode_){
        case DetectMode::ARMOR:{
           //!辅助射击
            cv::Mat caremaMatrix = constants::caremaMatrix_shoot;
            cv::Mat distCoeffs = constants::distCoeffs_shoot;
            cv::solvePnP(point3D, point2D, caremaMatrix, distCoeffs, rvecs, tvecs);  //解算x，y，z 三轴偏移量

            tx = tvecs.ptr<double>(0)[0];
            ty = tvecs.ptr<double>(0)[1];
            tz = tvecs.ptr<double>(0)[2];
            break;
        }
/*         case DetectMode::Rune:{
            //!大能量机关
            cv::Mat caremaMatrix = Constants::caremaMatrix_shoot;
            cv::Mat distCoeffs = Constants::distCoeffs_shoot;
            cv::solvePnP(point3D, point2D, caremaMatrix, distCoeffs, rvecs, tvecs);

            tx = tvecs.ptr<double>(0)[0];
            ty = -tvecs.ptr<double>(0)[1];
            tz = tvecs.ptr<double>(0)[2];

        } */
    }

    armor.tx_ = static_cast<float>(tx);
    armor.ty_ = static_cast<float>(ty);
    armor.tz_ = static_cast<float>(tz);
}

}
#endif