#ifndef __MSCKF_TYPE__
#define __MSCKF_TYPE__
#include<opencv2/core/core.hpp>
#include<Eigen/Core>
#include"feature.hpp"
namespace msckf_vio
{
    
    class ImageFrame
    {
    public:
        cv::Mat image;
        double stamp;
        ImageFrame(double time,cv::Mat &img)
        {
            this->image = img;
            this->stamp = time;
        }
        ImageFrame()
        {
            this->image = cv::Mat();
            this->stamp = 0.000;
        }

        ImageFrame operator=(const ImageFrame &frame)
        {
            this->stamp = frame.stamp;
            this->image = frame.image;
            return *this;
        }
    };

    typedef struct IMU
    {
    public:
        double stamp;
        Eigen::Vector3d linear_acceleration;
        Eigen::Vector3d angular_velocity;
    }IMU;

    typedef struct Stereo_Feature_Pt
    {
      uint32_t id;
      float u0;
      float v0;
      float u1;
      float v1;
    }Stereo_Feature_Pt;
    typedef struct Frame_Feature
    {
       double stamp;
       std::vector<Stereo_Feature_Pt> features;
    } Frame_Feature;

    /*
     * @brief StateServer Store one IMU states and several
     *    camera states for constructing measurement
     *    model.
     */
    struct StateServer {
      IMUState imu_state;
      CamStateServer cam_states;

      // State covariance matrix
      Eigen::MatrixXd state_cov;
      Eigen::Matrix<double, 12, 12> continuous_noise_cov;
    };
}

#endif