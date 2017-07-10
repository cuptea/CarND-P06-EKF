#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;


using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  
  x_ = F_*x_ ;
	P_ = F_*P_*F_.transpose() + Q_;
	cout << " status predict after:   "
  		 <<x_(0)<<" "
  		 <<x_(1)<<" "
  		 <<x_(2)<<" "
  		 <<x_(3)<<endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_*x_;
	MatrixXd S = H_*P_*H_.transpose() + R_;
	MatrixXd K = P_*H_.transpose()*S.inverse();
	x_ = x_ + (K*y);

	MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
	P_ = (I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  cout << " status:   "
  		 <<px<<" "
  		 <<py<<" "
  		 <<vx<<" "
  		 <<vy<<endl;

  //check division by zero
  if(px == 0) {
    return;
  }

  if(px == 0 && py==0) {
    return;
  }
  

  VectorXd z_pred(3,1);
  z_pred << sqrt(px*px + py*py), 
  					atan2(py, px), 
  					(vx*px + vy*py)/sqrt(px*px + py*py);

 
  
  VectorXd y = z - z_pred;
  
  
  float tmp = fmod(y(1),2*M_PI);
  
  if(tmp>=M_PI){
  	y(1)=tmp -2*M_PI;
  }else{
  	y(1)=tmp;
  }


  MatrixXd S = H_*P_*H_.transpose() + R_;
	MatrixXd K = P_*H_.transpose()*S.inverse();
	x_ = x_ + (K*y);

	MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
	P_ = (I-K*H_)*P_;

}



