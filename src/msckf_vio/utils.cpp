/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#include <msckf_vio/utils.h>
#include <vector>

namespace msckf_vio {
namespace utils {

Eigen::Isometry3d getTransformEigen(const cv::Mat &c) 
{
  Eigen::Isometry3d T;
  T.linear()(0, 0)   = c.at<double>(0, 0);
  T.linear()(0, 1)   = c.at<double>(0, 1);
  T.linear()(0, 2)   = c.at<double>(0, 2);
  T.linear()(1, 0)   = c.at<double>(1, 0);
  T.linear()(1, 1)   = c.at<double>(1, 1);
  T.linear()(1, 2)   = c.at<double>(1, 2);
  T.linear()(2, 0)   = c.at<double>(2, 0);
  T.linear()(2, 1)   = c.at<double>(2, 1);
  T.linear()(2, 2)   = c.at<double>(2, 2);
  T.translation()(0) = c.at<double>(0, 3);
  T.translation()(1) = c.at<double>(1, 3);
  T.translation()(2) = c.at<double>(2, 3);
  return T;
}

} // namespace utils
} // namespace msckf_vio
