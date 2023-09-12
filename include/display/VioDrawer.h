/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "imu_state.h"
#include "cam_state.h"
#include "type.h"
#include "math_utils.hpp"
#include <pangolin/pangolin.h>

#include<mutex>

namespace msckf_vio
{
class VioDrawer
{
public:
    VioDrawer();

    void DrawMapPoints();
    void DrawIMUFrames();
    void DrawCamFrames();
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Tcw);
    void DrawCurrentIMU(pangolin::OpenGlMatrix &Tiw);
    void SetCurrentFramePose(const Eigen::Matrix4d &Tcw,const Eigen::Matrix4d &Tiw);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    void GetCurrentOpenGLIMUMatrix(pangolin::OpenGlMatrix &M);
    void UpdateMap(const StateServer &stateServer,const MapServer &mapServer);
private:

    float mFrameSize;
    float mFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;
    bool  mbFirstPose;

    Eigen::Matrix4d mCurImuPose;//Tiw
    Eigen::Matrix4d mCurCamPose;//Tcw
    Eigen::Matrix4d mInitialPose;
    std::map<int,Eigen::Matrix4d> mmImu_Odom;
    std::map<int,Eigen::Matrix4d> mmCam_Odom;
    std::vector<Eigen::Vector3d> mvFeaturePts;

    std::mutex mMutexPose;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
