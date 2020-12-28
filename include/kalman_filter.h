#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include<eigen3/Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Dense>
namespace horizon {
using namespace Eigen;

class KalmanFilter{
public:
    KalmanFilter();
    ~KalmanFilter();
    void Initialize(const Vector3f& pos);
    Vector3f Predictor(const Vector3f& vec, const float& t);
    Vector3f Predictor(const float& t);

private:
    const float R;//系统误差的方差 视觉
    const float R_p;
    const float q; //过程误差(计算误差的方差)
    const float p;//初始化时初始位置误差的平方

    Vector3f H;
    Vector3f x;//当前状态
    Vector3f y;
    Vector3f z;
    Vector3f x_;//预测值
    Vector3f y_;//预测值
    Vector3f z_;//预测值
    MatrixXf Kx;//卡尔曼增益
    MatrixXf Ky;//卡尔曼增益
    MatrixXf Kz;//卡尔曼增益
    Matrix<float, 3,3> E;//单位矩阵
    Matrix<float, 3,3> Px;//状态协方差
    Matrix<float, 3,3> Px_;//预测协方差
    Matrix<float, 3,3> Py;//状态协方差
    Matrix<float, 3,3> Py_;//预测协方差
    Matrix<float, 3,3> Pz;//状态协方差
    Matrix<float, 3,3> Pz_;//预测协方差
    Matrix<float, 3,3> F;//状态转移矩阵
    Matrix<float, 3,3> Q;//状态转移协方差矩阵
};

}

#endif