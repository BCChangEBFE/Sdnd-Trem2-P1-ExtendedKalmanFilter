#include "kalman_filter.h"
#include "tools.h"
#include <iostream>
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_laser_in, MatrixXd &R_radar_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  //std::cout << "==Start Predict==" << std::endl ;
  x_ = F_ * x_ ;
  MatrixXd Ft_ = F_.transpose();
  P_ = F_ * P_ * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  //std::cout << "==Start Update==" << std::endl ;
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //std::cout << "==Start UpdateEKF==" << std::endl ;
  Tools tools = Tools::Tools();
  MatrixXd Hj = tools.CalculateJacobian(x_);
    
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rho = sqrt(pow(px,2) + pow(py,2));
  float rho_dot = ((px*vx) + (py*vy))/rho;
  float phi;
  if (fabs(px) > 0.0001)
  {
    phi = atan2(py,px);
    MatrixXd z_pred = VectorXd(3);
    z_pred << rho,
              phi,
              rho_dot;
    
    VectorXd y = z - z_pred;
    MatrixXd Hjt = Hj.transpose();
    MatrixXd S = Hj * P_ * Hjt + R_radar_;
    MatrixXd Si = S.inverse();
    MatrixXd PHjt = P_ * Hjt;
    MatrixXd K = PHjt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj) * P_;
  }
  //else do not update
}

