#include"kalman_filter.h"
namespace horizon{


KalmanFilter::KalmanFilter()
    : 
    R(9), R_p(25),  q(81), p(100),
    H(Eigen::Vector3f(1.0,0.0,0.0)),
    x(Eigen::Vector3f::Ones()), x_(Eigen::Vector3f::Ones()),
    y(Eigen::Vector3f::Ones()), y_(Eigen::Vector3f::Ones()),
    z(Eigen::Vector3f::Ones()), z_(Eigen::Vector3f::Ones()),
    Px(Eigen::Matrix3f::Identity(3,3)), Py(Eigen::Matrix3f::Identity(3,3)), Pz(Eigen::Matrix3f::Identity(3,3)),
    Px_(Eigen::Matrix3f::Identity(3,3)), Py_(Eigen::Matrix3f::Identity(3,3)), Pz_(Eigen::Matrix3f::Identity(3,3)),
    E(Eigen::Matrix3f::Identity(3,3)){
    F <<1,0,0,  // 1,1,0.5t^2
            0,1,0,     // 0,1,t
            0,0,1;     // 0,0,1
    Q <<  q, 0, 0, //状态转移协方差矩阵
                0, q, 0,
                0, 0, q;
}
KalmanFilter::~KalmanFilter(){}
Vector3f KalmanFilter::Predictor(const Vector3f& vec, const float& t){
    F << 1,t,0.5*t*t,
              0,1,t,
              0,0,1;
    Kx = Px_*H/(H.transpose()*Px_*H+R);
    x = x_+Kx*(vec[0]-H.transpose()*x);
    Px = (E-Kx*H.transpose())*Px_;
    Px_ = F*Px*F.transpose()+Q;
    x_=F*x;

    Ky = Py_*H/(H.transpose()*Py_*H+R);
    y = y_+Ky*(vec[1]-H.transpose()*y);
    Py = (E-Ky*H.transpose())*Py_;
    Py_ = F*Py*F.transpose()+Q;
    y_=F*y;

    Kz = Pz_*H/(H.transpose()*Pz_*H+R);
    z = z_+Kz*(vec[2]-H.transpose()*z);
    Pz = (E-Kz*H.transpose())*Pz_;
    Pz_ = F*Pz*F.transpose()+Q;
    z_=F*z;
//        std::cout<<"predict "<<std::endl<<"matrix = "<<x<<std::endl<<"matrix_ = "<<x_<<std::endl;
    return Vector3f(x[0], y[0], z[0]);
}

Vector3f KalmanFilter::Predictor(const float& t){
    F << 1.0f, t,0.5f*t*t,
              0,1.0f, t,
              0,0,1.0f;
    float x_t = x_(0);
    float y_t = y_(0);
    float z_t = z_(0);
    Kx = Px_*H/(H.transpose()*Px_*H+R);
    x = x_+Kx*(x_t-H.transpose()*x);
    Px = (E-Kx*H.transpose())*Px_;
    Px_ = F*Px*F.transpose()+Q;
    x_=F*x;

    Ky = Py_*H/(H.transpose()*Py_*H+R);
    y = y_+Ky*(y_t-H.transpose()*y);
    Py = (E-Ky*H.transpose())*Py_;
    Py_ = F*Py*F.transpose()+Q;
    y_=F*y;

    Kz = Pz_*H/(H.transpose()*Pz_*H+R);
    z = z_+Kz*(z_t-H.transpose()*z);
    Pz = (E-Kz*H.transpose())*Pz_;
    Pz_ = F*Pz*F.transpose()+Q;
    z_=F*z;
    return Vector3f(x[0], y[0], z[0]);
}
void KalmanFilter::Initialize(const Vector3f& pos){
    x << pos[0], 0, 0;
    y << pos[1], 0, 0;
    z << pos[2], 0, 0;
    Px << 1,0,0,  //不确定度
                0,1,0,
                0,0,1;
    Py << 1,0,0,  //不确定度
               0,1,0,
               0,0,1;
    Pz << 1,0,0,  //不确定度
                0,1,0,
                0,0,1;
    Px_ = Px+Q;
    Py_ = Py+Q; 
    Pz_ = Pz+Q; 
}








}