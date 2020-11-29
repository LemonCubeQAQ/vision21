#ifndef CONSTANTS_H_
#define CONSTANTS_H_

//在程序运行期间其值始终保持不变的, 命名时以 “k” 开头, 大小写混合


namespace horizon{

namespace constants{

const int kExposureTime = 2000;
const int kExposureGain = 300;

const int kMinLedArea = 15;
const int kMaxLedArea = 3e3;

const float kLedMinHeightVSWidth = 2.0f;
const float kLedMaxHeightVSWidth = 12.0f;
const float kLedMaxSlope = 0.8f;

const float kLedMinLengthRatio = 0.9f;
const float kLedMaxlengthRatio = 1.1f;

const float kLedSlopeDelta = 0.1f;
const int kLedMaxRatioWidthCmpHeight = 6;
const int kArmorThreshold = 5;

const float kRealSmallArmorWidth = 13.5f;
const float kRealSmallArmorHeight = 5.5f;
const float kRealLargeArmorWidth = 22.5f;
const float kRealLargeArmorHeight = 5.5f;
const float kRealRuneWidth = 24.0f;
const float kRealRuneHeight = 18.0f;


//相机内参
static const cv::Mat caremaMatrix_shoot = (
        cv::Mat_<float>(3, 3) << 648.4910,                  0,                         328.2316,
                                                        0,                                     652.0198,         254.6992,
                                                        0,                                     0,                                          1
                                  );
    //畸变参数
static const cv::Mat distCoeffs_shoot = (
        cv::Mat_<float>(1, 5) <<-0.2515, 
                                                        0.2977,
                                                        0,
                                                        0,
                                                        0);  
}




}

#endif