#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd result;

  /// check if estimations and ground truth are the same size
  if (estimations.size()<1 || estimations.size()!=ground_truth.size()) {
    /// error state
    result<< -1;
    return  result;
  }

  /// check if estimation vector and ground_truth vector have the same length
  if (estimations[0].size()!= ground_truth[0].size()){
    result<< -1;
    return result;
  }

  /// intialize result
  result = VectorXd(4);
  result<<0,0,0,0;

  /// calculate the sum of squares
  for (int i=0; i<estimations.size(); i++){
    VectorXd error = estimations[i] - ground_truth[i];
    error = error.array().square();
    result += error;
  }

  /// calculate average and its square root
  result /= estimations.size();
  result = result.array().sqrt();

  return result;

}