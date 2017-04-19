#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include "tools.h"
#include "UKFMeasurement.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  /// last timestamp
  long long last_timestamp_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;


  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* prediction error
  MatrixXd Xsig_err_;

  ///* Augmented sigma points
  MatrixXd Xsig_aug_;

  /// Measurement units
  std::map<MeasurementPackage::SensorType, UKFMeasurement*>  Units_;

  ///* Function for creating Agumented sigma points
  void CreateAugmentedSigmaPoints();

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;


  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;


  ///* Weights of sigma points
  VectorXd weights_;

  ///* the current NIS
  double NIS_;


public:

double NIS(){return NIS_;};

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param timestamp Last Time stamp
   */
  void Prediction(long long timestamp);

  /**
   * Add measurement unit.
   * @param unit - pointer to the measurement unit
   * @param type - type of the measurement unit
   */
  void AddUKFMeasurement(UKFMeasurement* unit, MeasurementPackage::SensorType type);

  /**
   * Return the state vector
   */
   VectorXd getX(){return x_;};

};

#endif /* UKF_H */
