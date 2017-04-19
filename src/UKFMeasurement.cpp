//
// Created by milutin on 2.4.17..
//

#include "UKFMeasurement.h"
#include <iostream>
using namespace Eigen;
/*
 * Update implementation
 */
double UKFMeasurement::Update(VectorXd& x, MatrixXd& P, const VectorXd &measurement, const MatrixXd& Xsig, const MatrixXd& Xsig_err,
                            const VectorXd& weights) {
  MatrixXd Z;
  // apply measurement function to sigma points
  Z = MeasurementFunction(Xsig);
  VectorXd z0;
  // calculate predicted measurement based on sigma points.
  // Here a strange behavior appeared when calculating prediction for angles
  // For example, when angles were
  // pi-0.001, -pi+0.001, 0.99*pi, -0.99*pi
  // the predicted angles would be 0, instead of pi
  // This situation might happen when measurement is near +/-pi so to avoid that
  // the angles should be in range (-measurement-pi, measurement+pi] which is done by
  // next 4 lines of code
  z0 = measurement;
  Z.colwise()-=z0;
  Process(Z);
  Z.colwise()+=z0;

  MatrixXd z_predicted;
  z_predicted = Z*weights;


  // calculate error for each sigma point
  MatrixXd Zerr = Z;
  Zerr.colwise() -= z_predicted.col(0);

  MatrixXd S;

  // calculate mesuremen covariance
  S = Zerr* weights.asDiagonal()* Zerr.transpose() + R_;

  MatrixXd NIS;
  MatrixXd err;
  err = measurement - z_predicted;
  Process(err);

  // calculate nis
  NIS = err.transpose()*S.inverse()*err;

  // calculate gain
  MatrixXd Tc;
  Tc = Xsig_err*weights.asDiagonal()*Zerr.transpose();

  MatrixXd K;
  K = Tc*S.inverse();

  // update predicted state and model covariance matrix
  x += K*(err);
  P -= K*S*K.transpose();

  // keep angle in range (-pi, pi]
  x(3) -= floor(x(3)/2.0/M_PI+0.5)*2*M_PI;

  return NIS(0,0);
}

/**
 * Blank Post processing function
 * @param z - measurement
 */
void UKFMeasurement::Process(Eigen::MatrixXd& z) {
  return;
}