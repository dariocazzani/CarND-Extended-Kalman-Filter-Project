#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
  * A helper method to calculate h_x that maps cartesian coordinates to polar coordinates.
  */
  VectorXd Convert2Polar(const VectorXd& x_state);


private:
  constexpr static float epsilon = 0.00001;
  float px, py, vx, vy = 0.0f;
  float rho, phi, rho_dot = 0.0f;

};

#endif /* TOOLS_H_ */
