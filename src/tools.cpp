#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  Hj << 0,0,0,0,
        0,0,0,0,
        0,0,0,0;
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float c1 = pow(px, 2.) + pow(py, 2.);
	float c2 = sqrt(c1);
	float c3 = pow(c1, 3./2.);

	//check division by zero
  if(fabs(c1) < 0.0001)
  {
    std::cout << "CalculateJacobian() has an error: Division by Zero" << std::endl;
    return Hj;
  }

	float d_ro_px = px / c2;
	float d_ro_py = py / c2;
	float d_phi_px = - py / c1;
	float d_phi_py = px / c1;
	float d_roprime_px = py*(vx*py - vy*px) / c3;
	float d_roprime_py = px*(vy*px - vx*py) / c3;
	float d_roprime_vx = px / c2;
	float d_roprime_vy = py / c2;

	//compute the Jacobian matrix
	Hj << d_ro_px, d_ro_py, 0.,  0.,
				d_phi_px, d_phi_py, 0., 0.,
				d_roprime_px, d_roprime_py, d_roprime_vx, d_roprime_vy;

	return Hj;
}

VectorXd Tools::Convert2Polar(const VectorXd& x_state) {
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float rho, phi, rho_dot;
  rho = sqrt(px*px + py*py);
  phi = atan2(py, px);  // returns values between -pi and pi

  // avoid division by 0t
  double epsilon = 0.000001;
  if(rho < epsilon)
    rho = epsilon;

  rho_dot = (px * vx + py * vy) / rho;

  VectorXd h_x = VectorXd(3);
  h_x << rho, phi, rho_dot;

  return h_x;
}

void Tools::constrainAngle(double &x)
{
  while(x < - M_PI)
    x += M_PI;
  while(x > M_PI)
    x -= M_PI;
  // return x;
}
