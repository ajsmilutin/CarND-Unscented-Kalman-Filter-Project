//
// Created by milutin on 12.4.17..
//

#ifndef UNSCENTEDKF_RADARMEASUREMENTUKF_H
#define UNSCENTEDKF_RADARMEASUREMENTUKF_H

#include "UKFMeasurement.h"

class RadarUKFMeasurement: public UKFMeasurement {

  /// Radar Radius standard deviation
  double std_radr_;
  /// Radar angle standard deviation
  double std_radphi_;
  /// Radar angular velocity standard deviation
  double std_radrd_;

public:
  /// Constructor
  RadarUKFMeasurement(double std_radr_, double std_radphi_, double std_radrd_);

  /// Radar Measurement function
  Eigen::MatrixXd MeasurementFunction(const Eigen::MatrixXd states);

  /// Radar Inverse measurement function
  Eigen::VectorXd Inverse(const Eigen::VectorXd &measurement);

  /// Post process to keep angles in certain range
  virtual void Process(Eigen::MatrixXd& Z);


};


#endif //UNSCENTEDKF_RADARMEASUREMENTUKF_H
