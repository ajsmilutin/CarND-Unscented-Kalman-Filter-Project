//
// Created by milutin on 12.4.17..
//

#include "RadarUKFMeasurement.h"
using namespace Eigen;


/// Constructor
RadarUKFMeasurement::RadarUKFMeasurement(double std_radr, double std_radphi, double std_radrd):std_radr_(std_radr),
std_radphi_(std_radphi), std_radrd_(std_radrd){
  R_ = MatrixXd::Zero(3,3);
  R_(0, 0) = std_radr_*std_radr_;
  R_(1, 1) = std_radphi_*std_radphi_;
  R_(2, 2) = std_radrd_*std_radrd_;
}

/// Radar inverse measurement function
/// Assumes that car goes radially from the origin
VectorXd RadarUKFMeasurement::Inverse(const VectorXd &measurement) {
  VectorXd result(5);
  double rho = measurement(0);
  double phi = measurement(1);
  double rhodot = measurement(2);
  result<< rho * cos(phi), rho*sin(phi), rhodot, phi, 0;
  return  result;
}

double sqr(double x){return x*x;};


/// Radar Measurement function
MatrixXd RadarUKFMeasurement::MeasurementFunction(const MatrixXd states) {
  MatrixXd Z_pred(3, states.cols() );

  for (int i=0; i < states.cols(); i++)
  {
    Z_pred(0, i)= sqrt(sqr(states(0,i)) + sqr(states(1,i)));
    Z_pred(1, i)= atan2(states(1,i),states(0,i));
    // 1e-6 is added to avoid division by zero
    Z_pred(2, i)= states(2,i)*(states(0,i)*cos(states(3,i)) + states(1,i)*sin(states(3,i)))
                  /(Z_pred(0, i)+1e-6);
  }
  return Z_pred;
}


/// Post process to keep angles in range (-pi, pi]
void RadarUKFMeasurement::Process(Eigen::MatrixXd& Z) {
  VectorXd pi;
  pi = (Z.row(1) )/2/M_PI;
  for (int i=0; i < Z.cols() ; i++)
  {
    Z(1, i) -= floor(pi(i)+0.5)*2*M_PI;
  }
}
