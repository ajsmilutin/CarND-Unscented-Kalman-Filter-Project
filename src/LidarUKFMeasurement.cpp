//
// Created by milutin on 12.4.17..
//

#include "LidarUKFMeasurement.h"
using namespace Eigen;

/// constructor
LidarUKFMeasurement::LidarUKFMeasurement(double std_px, double std_py) : std_laspx_(std_px),std_laspy_(std_py){
  R_ = MatrixXd::Zero(2,2);
  R_(0,0) = std_laspx_*std_laspx_;
  R_(1,1) = std_laspy_*std_laspy_;

};

/// Inverse measurement function.
/// Assumes that the speed, angle, and angular velocity are ziro
VectorXd LidarUKFMeasurement::Inverse(const VectorXd &measurement) {
  VectorXd result;
  result = MatrixXd::Zero(5,1);
  result.head(2) = measurement;
  return result;
}

/// Lidar measurement function
MatrixXd LidarUKFMeasurement::MeasurementFunction(const MatrixXd states) {
  MatrixXd result(2, states.cols());
  result = states.block(0, 0, 2, states.cols());
  return result;
}
