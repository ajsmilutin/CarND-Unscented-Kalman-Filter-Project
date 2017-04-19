//
// Created by milutin on 2.4.17..
//

#ifndef UNSCENTEDKF_UKFMEASUREMENT_H
#define UNSCENTEDKF_UKFMEASUREMENT_H
#include "Eigen/Dense"
#include "measurement_package.h"

class UKFMeasurement {
/***
 * Base class for Unscented Kalman Filter measurement.
 * There is a lot of common work for each measurement unit, like Updating state based on
 * a measurement.
 * The Update step is impremented in this function and a virtual measurement functions
 * are defined
 */
protected:
  // Covariance matrix
  Eigen::MatrixXd R_;

public:

  /**
   * Measurement function which calculates expected measurement based on
   * model state. It is used to calculate measurement error during the update step.
   *
   * Has to be implemented for each subclass
   *
   * @param states - state of the model
   * @return - expected measurement based on the state
   */
  virtual Eigen::MatrixXd MeasurementFunction(const Eigen::MatrixXd states)=0;

  /**
   * Function that returns a model state that produces provided measurement.
   * It is inverse of MeasurementFunction
   * Of course, multiple model states can produce the provided measurement,
   * but only one is selected.
   *
   * This function is used to initialize the model.
   *
   * Has to be implemented for each subclass
   *
   * @param measurement - measurement to be matched
   * @return - the model state that produces given measurement
   */
  virtual Eigen::VectorXd Inverse(const Eigen::VectorXd& measurement)=0;

  /**
   * Update step of the Unscented kalman filter.
   *
   * @param x - predicted state
   * @param P - process covariance matrix
   * @param measurement - new measurement
   * @param Xsig - sigma points
   * @param Xsig_err - difference between sigma points and predicted state
   * @param weights - weights used to calculate prediction based on sigma points
   * @return - NIS Normalized inovation squared
   */
  double Update(Eigen::VectorXd& x, Eigen::MatrixXd& P, const Eigen::VectorXd& measurement, const Eigen::MatrixXd& Xsig,
              const Eigen::MatrixXd& Xsig_err, const Eigen::VectorXd& weights) ;

  /**
   * Function that post-processes the measurement vector. There could be no post processing,
   * but if we have angles in measurement vector, they should be kept between -pi and pi
   * @param z
   */
  virtual void Process(Eigen::MatrixXd& z);

};


#endif //UNSCENTEDKF_UKFMEASUREMENT_H
