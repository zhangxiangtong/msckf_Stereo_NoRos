/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_UTILS_H
#define MSCKF_VIO_UTILS_H

// #include <ros/ros.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>

namespace msckf_vio {
/*
 * @brief utilities for msckf_vio
 */
namespace utils 
{
    Eigen::Isometry3d getTransformEigen(const cv::Mat &c);
}
}
#endif
