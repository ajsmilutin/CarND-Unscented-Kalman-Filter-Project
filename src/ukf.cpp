#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include "math.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // initial state vector
  x_ = VectorXd::Zero(5);

  // initial covariance matrix
  P_ = 0.5*MatrixXd::Identity(5, 5);
  P_(0,0) = 0.15;
  P_(1,1) = 0.15;

  n_x_  = 5;
  n_aug_ = 7;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.45;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

  // Initialize weights
  weights_ = 0.5/(lambda_+ n_aug_)*VectorXd::Ones(2*n_aug_ + 1);
  weights_(0) = ((float)lambda_)/(lambda_+ n_aug_);

  // initialize augmented sigma points
  Xsig_aug_ = MatrixXd::Zero(n_aug_, 2*n_aug_+1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // initialize state vector
  if (!is_initialized_){
    last_timestamp_ = meas_package.timestamp_;
    x_ = Units_[meas_package.sensor_type_]->Inverse(meas_package.raw_measurements_);
    is_initialized_ = true;
  }
  // Predict
  Prediction(meas_package.timestamp_);

  // Update based on the measurement
  NIS_ = Units_[meas_package.sensor_type_]->Update(x_, P_, meas_package.raw_measurements_,Xsig_pred_, Xsig_err_, weights_);

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {long long} timestamp - the timestamp of the latest measurement
 * measurement and this one.
 */
void UKF::Prediction(long long timestamp) {
  // calculate dt
  double dt = (timestamp - last_timestamp_)/1000000.0;
  last_timestamp_ = timestamp;

   // generate sigma points
  CreateAugmentedSigmaPoints();
  // predict the sigma points
  Xsig_pred_ = Xsig_aug_.block(0, 0, n_x_, 2*n_aug_ + 1);

  for (int i=0; i<2*n_aug_ + 1 ; i++){
    double px = Xsig_aug_(0, i);
    double py = Xsig_aug_(1, i);
    double v = Xsig_aug_(2, i);
    double phi = Xsig_aug_(3, i);
    double dphi = Xsig_aug_(4, i);
    double dv = Xsig_aug_(5, i);
    double ddphi = Xsig_aug_(6, i);


    if (abs(dphi)<1e-4){
      Xsig_pred_(0, i)+=v*cos(phi)*dt;
      Xsig_pred_(1, i)+=v*sin(phi)*dt;
    }
    else {
      Xsig_pred_(0, i)+=v/dphi*(sin(phi+dt*dphi)-sin(phi));
      Xsig_pred_(1, i)+=v/dphi*(-cos(phi+dt*dphi)+cos(phi));
    }
    Xsig_pred_(0, i) += 0.5*dt*dt*cos(phi)*dv;
    Xsig_pred_(1, i) += 0.5*dt*dt*sin(phi)*dv;
    Xsig_pred_(2, i) += dt*dv;
    Xsig_pred_(3, i) += dphi*dt + 0.5*dt*dt*ddphi;
    Xsig_pred_(4, i) += dt*ddphi;

  }

  // calculate the mean and covariance;
  x_ = Xsig_pred_*weights_;
  x_(3) -= floor(x_(3)+0.5)*2*M_PI;
  Xsig_err_ = Xsig_pred_;
  Xsig_err_.colwise()-= x_;

  VectorXd xrep = Xsig_err_.row(3);

  xrep = xrep/2/M_PI;

  // keep all angles in (-pi, pi]
  for (int i=0; i<2 * n_aug_ + 1; i++)
    Xsig_err_(3, i) -= floor(xrep(i)+0.5)*2*M_PI;


  P_ = Xsig_err_*(weights_.asDiagonal())*Xsig_err_.transpose();

}



void UKF::CreateAugmentedSigmaPoints() {
  // create augmented vector
  VectorXd x_aug(n_aug_);
  x_aug.setZero();
  x_aug.head(n_x_) = x_;

  // create augmented matrix
  MatrixXd P_aug;
  P_aug = MatrixXd::Zero(n_aug_, n_aug_);

  P_aug.block(0, 0, 5, 5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  MatrixXd root = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug_.colwise() = x_aug;
  Xsig_aug_.block(0, 1, n_aug_, n_aug_) += sqrt(lambda_ + n_aug_)*root;
  Xsig_aug_.block(0, n_aug_ + 1, n_aug_, n_aug_) -=sqrt(lambda_ + n_aug_)*root;

}


void UKF::AddUKFMeasurement(UKFMeasurement *unit, MeasurementPackage::SensorType type) {
  Units_[type] = unit;
}