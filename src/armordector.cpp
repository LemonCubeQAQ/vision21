#ifndef ARMORDECTOR_H_
#define ARMORDECTOR_H

#include "armordector.h"

namespace horizon{
using namespace std;

ArmorDector::ArmorDector()
    :
    enemy_color_(constants::kEnemyColor),
    detect_mode_(DetectMode::NONE),
    src_image_(),
    binary_image_(),
    aultimate_image_(),
    led_array_{},
    armor_num_(0),
    armor_array_{},
    predictor_()
{};

ArmorDector::~ArmorDector(){};

const Vector3f& ArmorDector::GetHitPos(DetectMode detect_mode, cv::Mat& src_image){
    src_image_ = src_image;

    switch(detect_mode){
        case DetectMode::ARMOR:{
            if(GetAllTarget()){
                predictor_.GetArmorData(armor_array_, armor_num_);
            }
            else{
                predictor_.LostTarget();
            }
            break;
        }
        case DetectMode::RUNE:{
            if(GetAllRune()){
                //predictor_.GetRuneData();
            }
            else{
                predictor_.LostTarget();
            }
            break;
        }
        default:
            predictor_.LostTarget();
            break;
    }
    
    return predictor_.GetPredictedTarget();
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


void ArmorDector::SeparateColor(){

    cv::Mat splited_image[3];
    split(src_image_, splited_image);

    switch(enemy_color_){
        case EnemyColor::RED:{
            binary_image_ = splited_image[2] - 0.2f*splited_image[1] - 0.8f*splited_image[0];
            
            break;
        }
        case EnemyColor::BLUE:{
            

            break;
        }
    }

}

void ArmorDector::AdditionalProcess(){
    threshold(binary_image_, binary_image_, 120, 255, CV_THRESH_BINARY);
    blur(binary_image_, binary_image_,Size(3,5));
    threshold(binary_image_, binary_image_, 20, 255, CV_THRESH_BINARY);
    blur(binary_image_, binary_image_,Size(3,3));
    imshow("b" , binary_image_);
    waitKey(1);
    aultimate_image_ = src_image_.clone();
}


unsigned short ArmorDector::GetAllLed(){

    vector<std::vector<cv::Point>> led_contours;
    vector<cv::Vec4i> hierarchy;
    RotatedRect led_rect;
    findContours(binary_image_, led_contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    unsigned short contours_size = static_cast<unsigned short>(led_contours.size());
    unsigned short led_num= 0;

    for(unsigned short contour_index = 0; contour_index < contours_size && led_num < LED_SIZE; contour_index++){

        vector<cv::Point> & led_contour = led_contours[contour_index];

        double area = contourArea(led_contour);
        if(area < constants::kMinLedArea || area > constants::kMaxLedArea) continue;

        led_rect = minAreaRect(led_contour);

        Led led;
        Point2f vertex[4];    
        led_rect.points(vertex);        
        sort(vertex, vertex + 4, ledCmpY);
        sort(vertex, vertex + 2, ledCmpX);
        sort(vertex+2, vertex+4, ledCmpX);
        const Point2f& bottomLeft = vertex[2];
        const Point2f& topLeft = vertex[0];
        const Point2f& topRight = vertex[1];
        const Point2f& bottomRight = vertex[3];

        led.top_point_ = (topLeft + topRight) * 0.5f; 
        led.bottom_point_ = (bottomLeft + bottomRight) * 0.5f;
        led.center_ =  (led.top_point_ + led.bottom_point_) * 0.5f;

        Point2f point_delta_vertical = led.bottom_point_ - led.top_point_;
        led.length_ = sqrt(point_delta_vertical.x * point_delta_vertical.x + point_delta_vertical.y * point_delta_vertical.y);
        led.slope_ = point_delta_vertical.x / point_delta_vertical.y;

        Point2f point_delta_horizontal = (topRight - topLeft);        
        led.width_ = sqrt(point_delta_horizontal.x * point_delta_horizontal.x + point_delta_horizontal.y * point_delta_horizontal.y);
        led.ratio_ = led.length_ / led.width_;

        if( led.ratio_ > constants::kLedMaxHeightVSWidth
            || led.ratio_ < constants::kLedMinHeightVSWidth
            || abs(led.slope_) > constants::kLedMaxSlope
        ) continue;

        led_array_[led_num++] = led;
        line(src_image_, topLeft, topRight,Scalar(0, 0, 255), 2, 8);
        line(src_image_, topLeft, bottomLeft,Scalar(0, 255, 255), 2, 8);
        line(src_image_, bottomRight, topRight,Scalar(0, 255, 255), 2, 8);
        line(src_image_, bottomRight, bottomLeft,Scalar(0, 0, 255), 2, 8);        

    }
    imshow("led", src_image_);
    waitKey(1);
    return led_num;
}

unsigned short ArmorDector::GetAllArmor(const unsigned short& led_num){

    unsigned short armor_num = 0;
    bool is_selected[led_num];
    memset(is_selected, false, led_num);
    sort(led_array_, led_array_ + led_num, ledCmpLength);

    for(unsigned short i = 0; i < led_num; i++){
        for(unsigned short index = i+1; index < led_num; index++){
            if(is_selected[i] || is_selected[index]) continue; //未被配对

            float lRatio = led_array_[i].length_ / led_array_[index].length_; //长度相近
            if(lRatio > 1.25f || lRatio < 0.8) continue;

            float AngleMinus = led_array_[i].slope_ - led_array_[index].slope_; //角度相近
            if(abs(AngleMinus) > 0.1f) continue;

            cv::Point2f center_minus = led_array_[i].center_ - led_array_[index].center_;

            float centerDeltaY = abs(center_minus.y);
            float centerDeltaX = abs(center_minus.x);
            float centerDistance = sqrt(center_minus.y * center_minus.y + center_minus.x * center_minus.x);
            float length = (led_array_[i].length_+led_array_[index].length_) * 0.5f;
            float hRatio =  centerDeltaY / length;
            float wRatio =  centerDeltaX / length;
            float whRatio = centerDistance / length;

            if( hRatio > 0.4f || wRatio > 3.0f || whRatio > 15.0f || whRatio < 0.6f) continue;

            is_selected[i] = true;
            is_selected[index] = true;
            Armor& armor = armor_array_[armor_num];
            if(whRatio < constants::kArmorTypeThreshold){
                armor.armor_type_ = ArmorType::SMALL;
            }
            else{
                armor.armor_type_ = ArmorType::LARGE;
            }
            armor.left_led_[0] = led_array_[i].top_point_;
            armor.left_led_[1] = led_array_[i].bottom_point_;
            armor.right_led_[0] = led_array_[index].top_point_;
            armor.right_led_[1] = led_array_[index].bottom_point_;
            if(led_array_[i].center_.x > led_array_[index].center_.x){
                swap(armor.left_led_, armor.right_led_);
            }
            line(aultimate_image_, armor.left_led_[0], armor.left_led_[1],Scalar(0, 0, 255), 2, 8);
            line(aultimate_image_, armor.right_led_[0], armor.left_led_[0],Scalar(0, 0, 255), 2, 8);
            line(aultimate_image_, armor.right_led_[1], armor.left_led_[1],Scalar(0, 0, 255), 2, 8);
            line(aultimate_image_, armor.right_led_[1], armor.right_led_[0],Scalar(0, 0, 255), 2, 8);        
            armor_num++;
        }
        if(armor_num == ARMOR_SIZE) break;
    }

    imshow("armor", aultimate_image_);
    waitKey(1);
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
            ty = -tvecs.ptr<double>(0)[1];
            tz = tvecs.ptr<double>(0)[2];
            break;
        }
         case DetectMode::RUNE:{
            //!大能量机关

            break;
        } 
    }

    armor.vec[0] = static_cast<float>(tx);
    armor.vec[1] = static_cast<float>(ty);
    armor.vec[2] = static_cast<float>(tz);
}

}
#endif