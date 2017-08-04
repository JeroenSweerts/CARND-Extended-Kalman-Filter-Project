#include "kalman_filter.h"
#include "Eigen/Dense"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

#define PI 3.14159265

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
	x_ = F_ * x_;	
	MatrixXd Ft_ = F_.transpose();
	P_ = F_ * P_ *Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
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
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	//cout << "z: " << z[1] << endl;
	VectorXd z_pred;
	z_pred = VectorXd(3);
	z_pred[0] = sqrt((x_[0] * x_[0]) + (x_[1] * x_[1]));
	//z_pred[1] = atan(x_[1] / x_[0]);		
	z_pred[1] = atan2(x_[1],x_[0]);
	//if (z_pred[1] < -1*PI) { z_pred[1] = z_pred[1] + (2*PI); }
	//if (z_pred[1] > PI) { z_pred[1] = z_pred[1] - (2 * PI); }

	if (fabs(z_pred[0]) < 0.0001) {
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		z_pred[2] = 0;
	}
	else {
		z_pred[2] = ((x_[0] * x_[2]) + (x_[1] * x_[3])) / (z_pred[0]);
	}
	
	VectorXd y = z - z_pred;
	if (y[1] < -1*PI) { y[1] = y[1] + (2*PI); }
	if (y[1] > PI) { y[1] = y[1] - (2 * PI); }
	cout << "y: " << y[1] << endl;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;


}
