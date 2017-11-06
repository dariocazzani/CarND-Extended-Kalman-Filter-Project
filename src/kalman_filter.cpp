#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::estimate(const VectorXd &y)
{
  /**
  These operations are the same for the Udpate for KF and EKF
  once the Matrices H and R are correctly initialized
  */
  // Same as in the function of KalmanFilter::Update()
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::Predict()
{
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z)
{
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;

  VectorXd y = z - z_pred;

  estimate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Used to calculate Jacobian and convert cartesian to polar
  Tools tools;

  // map predicted location from cartesian to polar coordinates
  VectorXd h_x = tools.Convert2Polar(x_);
  VectorXd y = z - h_x;

  // Normalize angle between -pi and pi
  y(1) = atan2(sin(y(1)), cos(y(1)));

  estimate(y);
}
