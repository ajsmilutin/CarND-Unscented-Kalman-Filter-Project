//
// Created by milutin on 12.4.17..
//

#ifndef UNSCENTEDKF_LIDARUKFMEASUREMENT_H
#define UNSCENTEDKF_LIDARUKFMEASUREMENT_H

#include "UKFMeasurement.h"

class LidarUKFMeasurement: public UKFMeasurement {

private:
  /// Laser measurement noise standard deviation position1 in m
  double std_laspx_;
  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;
public:
  /// Constructor
  LidarUKFMeasurement(double std_px, double std_py);

  /// Lidar Measurement function
  Eigen::MatrixXd MeasurementFunction(const Eigen::MatrixXd states);

  /// Lidar Inverse function
  Eigen::VectorXd Inverse(const Eigen::VectorXd &measurement);


};


#endif //UNSCENTEDKF_LIDARUKFMEASUREMENT_H
