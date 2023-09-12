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

#include <pangolin/pangolin.h>
#include"VioDrawer.h"
#include <mutex>
using namespace std;
namespace msckf_vio
{

VioDrawer::VioDrawer()
{
    mFrameSize = 0.1;
    mFrameLineWidth = 1;
    mGraphLineWidth = 1;
    mPointSize = 2;
    mCameraSize = 0.15;
    mCameraLineWidth = 2;
    mbFirstPose = true;
}

void VioDrawer::DrawMapPoints()
{
    
    unique_lock<mutex> lock(mMutexPose);
    if(this->mvFeaturePts.empty())
    {
        return;
    }

    int nPointSize = this->mvFeaturePts.size();

    glPointSize(2);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t id = 0, iend = this->mvFeaturePts.size(); id < iend;id++)
    {
        Eigen::Vector3d &pos = this->mvFeaturePts[id];
        glVertex3f(pos(0),pos(1),pos(2));
        // std::cout<<"pt = "<<pos<<std::endl;
    }
    glEnd();    
}


void VioDrawer::DrawCamFrames()
{
    vector<Eigen::Matrix4d> vCamPose;
    int nFrameSize = 0;
    {
       unique_lock<mutex> lock(mMutexPose);
       nFrameSize = this->mmCam_Odom.size();
       vCamPose.reserve(nFrameSize);
       for(auto it = this->mmCam_Odom.begin();it != this->mmCam_Odom.end();it++)
       {
            vCamPose.push_back(it->second);
       }
    }

    
    if(nFrameSize < 1)
    {
        return;
    }

    //not last frame
    glLineWidth(mFrameLineWidth);
    glColor3f(0.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    Eigen::Vector3d prePt;
    for(int id = 0; id < nFrameSize;id++)
    {
        Eigen::Matrix4d &Tcw = vCamPose[id];
        Eigen::Vector3d pos = Tcw.block<3,1>(0,3);
        if(id == 0)
        {
            prePt = pos;
            continue;
        }

        glVertex3f(prePt(0),prePt(1),prePt(2));
        glVertex3f(pos(0),pos(1),pos(2));
        prePt = pos;
    }

    glEnd();

    Eigen::Matrix4d &Tcw = vCamPose[nFrameSize - 1];
    pangolin::OpenGlMatrix M;
    M.SetIdentity();
    GetCurrentOpenGLCameraMatrix(M);
    DrawCurrentCamera(M);
}

void VioDrawer::DrawIMUFrames()
{
    std::vector<Eigen::Matrix4d> vImuPos;
    int nFrameSize = 0;
    {
        unique_lock<mutex> lock(mMutexPose);
        nFrameSize = this->mmImu_Odom.size();
        vImuPos.reserve(nFrameSize);
        for(auto it = this->mmImu_Odom.begin();it != this->mmImu_Odom.end();it++)
        {
            vImuPos.push_back(it->second);
        }
    }
   
    if(nFrameSize < 1)
    {
        return;
    }
    

    //not last frame
    glLineWidth(3);
    glColor3f(0.0f,1.0f,1.0f);
    glBegin(GL_LINES);

    Eigen::Vector3d prePt;
    for(int id = 0; id < nFrameSize;id++)
    {
        Eigen::Matrix4d &Tiw = vImuPos[id];
        Eigen::Vector3d pos = Tiw.block<3,1>(0,3);
        if(id == 0)
        {
            prePt = pos;
            continue;
        }

        glVertex3f(prePt(0),prePt(1),prePt(2));
        glVertex3f(pos(0),pos(1),pos(2));
        prePt = pos;
    }
    glEnd();

    Eigen::Matrix4d &Tiw = vImuPos[nFrameSize - 1];
    pangolin::OpenGlMatrix M;
    M.SetIdentity();
    GetCurrentOpenGLIMUMatrix(M);
    DrawCurrentIMU(M);
}

void VioDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Tcw)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Tcw.m);
#else
        glMultMatrixd(Tcw.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void VioDrawer::DrawCurrentIMU(pangolin::OpenGlMatrix &Tiw)
{
    const float &w = 4;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Tiw.m);
#else
        glMultMatrixd(Tiw.m);
#endif
    float axisLen = 0.5;
    float arrowLen = 0.1;
    if(arrowLen > axisLen / 4)
    {
        arrowLen = axisLen / 4;
    }
    float arrowAngle = 10 * 3.1415926 / 180.0;
    float sinAngle = sin(arrowAngle);
    float arrowWidth = arrowLen * sinAngle;
    glLineWidth(5);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(axisLen,0,0);

    glVertex3f(axisLen,0,0);
    glVertex3f(axisLen - arrowLen,-arrowWidth,0);

    glVertex3f(axisLen,0,0);
    glVertex3f(axisLen - arrowLen,arrowWidth,0);


    glColor3f(0.0f,0.0f,1.0f);
    glVertex3f(0,0,0);
    glVertex3f(0,axisLen,0);

    glVertex3f(0,axisLen,0);
    glVertex3f(0,axisLen - arrowLen,arrowWidth);

    glVertex3f(0,axisLen,0);
    glVertex3f(0,axisLen - arrowLen,-arrowWidth);

    glColor3f(1.0f,0.0f,0.0f);
    glVertex3f(0,0,0);
    glVertex3f(0,0,axisLen);

    glVertex3f(0,0,axisLen);
    glVertex3f(arrowWidth,0,axisLen - arrowLen);

    glVertex3f(0,0,axisLen);
    glVertex3f(-arrowWidth,0,axisLen - arrowLen);

    glEnd();

    glPopMatrix();
}


void VioDrawer::SetCurrentFramePose(const Eigen::Matrix4d &Tcw,const Eigen::Matrix4d &Tiw)
{
    unique_lock<mutex> lock(mMutexPose);
    this->mCurCamPose = Tcw;
    this->mCurImuPose = Tiw;
}

void VioDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
   
    {
        Eigen::Matrix3d Rxw;
        Eigen::Vector3d txw;
        {
            unique_lock<mutex> lock(mMutexPose);
            Rxw = mCurCamPose.block<3,3>(0,0);
            txw = mCurCamPose.block<3,1>(0,3);
        }

        M.m[0] = Rxw(0,0);
        M.m[1] = Rxw(1,0);
        M.m[2] = Rxw(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rxw(0,1);
        M.m[5] = Rxw(1,1);
        M.m[6] = Rxw(2,1);
        M.m[7] = 0.0;

        M.m[8] = Rxw(0,2);
        M.m[9] = Rxw(1,2);
        M.m[10]= Rxw(2,2);
        M.m[11]= 0.0;

        M.m[12] = txw(0);
        M.m[13] = txw(1);
        M.m[14] = txw(2);
        M.m[15] = 1.0;
    }
    // else
    // {
    //     M.SetIdentity();
    // }

}

void VioDrawer::GetCurrentOpenGLIMUMatrix(pangolin::OpenGlMatrix &M)
{
   
    {
        Eigen::Matrix3d Rxw;
        Eigen::Vector3d txw;
        {
            unique_lock<mutex> lock(mMutexPose);
            Rxw = mCurImuPose.block<3,3>(0,0);
            txw = mCurImuPose.block<3,1>(0,3);
        }

        M.m[0] = Rxw(0,0);
        M.m[1] = Rxw(1,0);
        M.m[2] = Rxw(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rxw(0,1);
        M.m[5] = Rxw(1,1);
        M.m[6] = Rxw(2,1);
        M.m[7] = 0.0;

        M.m[8] = Rxw(0,2);
        M.m[9] = Rxw(1,2);
        M.m[10]= Rxw(2,2);
        M.m[11]= 0.0;

        M.m[12] = txw(0);
        M.m[13] = txw(1);
        M.m[14] = txw(2);
        M.m[15] = 1.0;
    }
    // else
    // {
    //     M.SetIdentity();
    // }

}

void VioDrawer::UpdateMap(const StateServer &stateServer,const MapServer &mapServer)
{
    unique_lock<mutex> lock(mMutexPose);

    //get IMU and camera pose in pStateServer
   const IMUState &imu_state = stateServer.imu_state;
   Eigen::Isometry3d T_i_w = Eigen::Isometry3d::Identity();
   T_i_w.linear() = quaternionToRotation(imu_state.orientation).transpose();
   T_i_w.translation() = imu_state.position;
//    Eigen::Isometry3d T_b_w = IMUState::T_imu_body * T_i_w *IMUState::T_imu_body.inverse();
//   std::cout<<"T_b_w.rotation = "<<std::endl<<T_b_w.linear()<<std::endl;
//   std::cout<<"T_b_w.translation = "<<std::endl<<T_b_w.translation()<<std::endl;
//    Eigen::Vector3d body_velocity = IMUState::T_imu_body.linear() * imu_state.velocity;
//    if(mbFirstPose)
//    {
//        mInitialPose.block<3,3>(0,0) = T_i_w.linear();
//        mInitialPose.block<3,1>(0,3) = T_i_w.translation();
//        mbFirstPose = false;
//    }
//    Eigen::Isometry3d T_b_v_gt;
//    T_b_v_gt.linear() = T_i_w.linear();
//    T_b_v_gt.translation() = imu_state.position;

//    Eigen::Isometry3d mocap_initial_frame;
//    mocap_initial_frame.linear()      = mInitialPose.block<3,3>(0,0);
//    mocap_initial_frame.translation() = mInitialPose.block<3,1>(0,3);

//    Eigen::Isometry3d T_b_w_gt = mocap_initial_frame.inverse() * T_b_v_gt;

//    this->mCurImuPose.block<3,3>(0,0) = T_b_w_gt.linear();
//    this->mCurImuPose.block<3,1>(0,3) = T_b_w_gt.translation();
   this->mCurImuPose.setIdentity();
   this->mCurImuPose.block<3,3>(0,0) = T_i_w.linear();
   this->mCurImuPose.block<3,1>(0,3) = T_i_w.translation();


   this->mmImu_Odom[imu_state.id] = this->mCurImuPose;

   const CamStateServer &cam_states = stateServer.cam_states;
   int latestCamId = 0;
   for(auto it = cam_states.begin();it != cam_states.end();it++)
   {
       long long int id = it->first;
       const CAMState &state = it->second;
       Eigen::Matrix4d Tcw;
       Tcw.block<3,3>(0,0) = quaternionToRotation(state.orientation).transpose();
       Tcw.block<3,1>(0,3) = state.position;
       this->mmCam_Odom[id] = Tcw;
       if(latestCamId < id)
       {
           latestCamId = id;
       }
   }
   this->mCurCamPose = this->mmCam_Odom[latestCamId];

    //get FeaturePoint in pMapServer
    int ptSize = mapServer.size();
    this->mvFeaturePts.clear();
    this->mvFeaturePts.reserve(ptSize);
    for(auto it = mapServer.begin();it != mapServer.end();it++)
    {
        long long int id = it->first;
        const Feature &feature = it->second;
        this->mvFeaturePts.push_back(feature.position);
    }
}

} //namespace ORB_SLAM
