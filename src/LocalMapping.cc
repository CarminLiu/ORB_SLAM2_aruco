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

#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include<mutex>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "Thirdparty/DBoW2/DUtils/Random.h"

namespace ORB_SLAM2
{

bool LocalMapping::mbIniT = true;

long unsigned int LocalMapping::nkfid = 0;

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
    mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{    mLastDoneScale=0; DoScale = false;
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
    // DoScale = pTracker->mbUseAruco;
}

void LocalMapping::Run()
{

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();

            // Triangulate new MapPoints
            CreateNewMapPoints();
            MapPointRelatedAruco();

            //* Create MapPoints about aurco
            CreateArucoMapPoints();

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

            mbAbortBA = false;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                if(mpMap->KeyFramesInMap()>2)
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap); //! Add Aruco in this function

                // Check redundant local Keyframes
                KeyFrameCulling();
            }

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                {
                    pMP->AddObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }    

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);

    nkfid++;
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    cv::Mat Tcw1(3,4,CV_32F);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        cv::Mat vBaseline = Ow2-Ow1;
        const float baseline = cv::norm(vBaseline);

        if(!mbMonocular)
        {
            if(baseline<pKF2->mb)
            continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

        cv::Mat Rcw2 = pKF2->GetRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->GetTranslation();
        cv::Mat Tcw2(3,4,CV_32F);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
            const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
            bool bStereo1 = kp1_ur>=0;

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = kp2_ur>=0;

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

            cv::Mat x3D;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,CV_32F);
                A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<float>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                x3D = pKF2->UnprojectStereo(idx2);
            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;
            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            /*if(fabs(ratioDist-ratioOctave)>ratioFactor)
                continue;*/
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);

            pMP->AddObservation(mpCurrentKeyFrame,idx1);            
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
}

void LocalMapping::MapPointRelatedAruco()
{
    for(size_t i=0; i<mpCurrentKeyFrame->NA; i++)
    {
        vector<size_t> vindices = mpCurrentKeyFrame->GetFeaturesInAruco(i);
        for(size_t j=0; j<vindices.size(); j++)
        {
            MapPoint* pMP = mpCurrentKeyFrame->GetMapPoint(vindices[j]);
            if(pMP) {
                if(!pMP->isBad()) {
                    pMP->forflag = 1;
                    pMP->mArucoID = mpCurrentKeyFrame->mvMarkers[i].id;
                }
            }
        }
    }
}

void LocalMapping::CreateArucoMapPoints()
{
    // step 1: 获取当前帧的Aruco数量
    const int NA = mpCurrentKeyFrame->NA;
    std::map<int, double>& ArucoMeanLength = mpMap->mArucoMeanLength;
    // vector<KeyFrame*> pvkf = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(3); // 获取3帧共视程度最好的帧
    // pvkf.push_back(mpCurrentKeyFrame); //再加上自己

    //保存所有检测比较好的Aruco的ID
    // vector<int> vA;
    // for(size_t i=0; i<pvkf.size(); i++){
    //     KeyFrame* kf = pvkf[i];
    //     for(size_t j=0; j<kf->NA; j++) {
    //         if(kf->mvbArucoGood[i] == false)
    //             continue;
    //         int iid = kf->mvMarkers[j].id;
    //         vector<int>::iterator vit = find(vA.begin(), vA.end(), iid);
    //         if(vit==vA.end()){
    //             vA.push_back(iid);
    //         }
    //     }
    // }
    // int numAruco = vA.size(); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    map<int, vector<cv::Point3f>> &mAruMPs=mpTracker->mmAruMPs; // 用于计算当前帧算的比较好的Aruco的四个空间点。
    // step 2: For each aruco:
    // 1) Find keypoints in aruco; 2)fit a plane; 
    //* 3)aruco 4 cornors in this plane 找4个空间点
    float maxLengthDiff;// = 0.015;
    for(size_t i=0; i<NA; i++)
    {
        if(mpCurrentKeyFrame->mvbArucoGood[i] == false)
            continue;
        vector<size_t> vindices = mpCurrentKeyFrame->GetFeaturesInAruco(i); //获取当前帧索引号为i的Aruco框内的特征点
        //假设要求Aruco内有5个点。
        if(vindices.size()<5) continue; // <5,则算下一个Aruco

        vector<Eigen::Vector3d> plane_pts;
        cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
        cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
        int cntMPok=0; //记录多少个点ok
        //* all pMPs are ok 并把框内的MapPoint做标记，1.forflag; 2.mArucoID
        for(size_t j=0; j<vindices.size(); j++)
        {
            MapPoint* pMP = mpCurrentKeyFrame->GetMapPoint(vindices[j]);
            if(pMP) {
                if(!pMP->isBad()) {
                    // pMP->forflag = 1;
                    cv::Mat pw = pMP->GetWorldPos();
                    cv::Mat pc = Rcw*pw + tcw; // 获得在当前关键帧下的坐标
                    plane_pts.push_back(Eigen::Vector3d(pc.at<float>(0), pc.at<float>(1), pc.at<float>(2)));
                    pMP->forflag = 1;
                    pMP->mArucoID = mpCurrentKeyFrame->mvMarkers[i].id;
                    cntMPok++;
                }
            }
        }
        if(plane_pts.size()<5) continue; // <5,则算下一个Aruco

        MapAruco* pMA = mpCurrentKeyFrame->GetMapAruco(i);
        cv::Mat axlez = pMA->GetAxleZInworld();

        int iterator = cntMPok/2;
        DUtils::Random::SeedRandOnce(0);
        Eigen::Vector4d plapramBest;
        Eigen::Vector3d centerBest;
        const float &cx1 = mpCurrentKeyFrame->cx;
        const float &cy1 = mpCurrentKeyFrame->cy;
        const float &invfx1 = mpCurrentKeyFrame->invfx;
        const float &invfy1 = mpCurrentKeyFrame->invfy;
        // float ErrMin = MAXFLOAT;
        
        int id = mpCurrentKeyFrame->mvMarkers[i].id;
        map<int,double>::iterator mit = maxLenDiff.find(id);
        if(mit == maxLenDiff.end())
            maxLengthDiff = 0.015;
        else 
            maxLengthDiff = maxLenDiff[id];
        for (size_t j = 0; j < iterator; j++)
        {
            vector<Eigen::Vector3d> planeRandomPoints;
            for(int m=0; m<5; m++){
                int randi = DUtils::Random::RandomInt(0,cntMPok-1);
                planeRandomPoints.push_back(plane_pts[randi]);
            }
            Eigen::Vector3d center;
            Eigen::Vector4d plapram = PlaneFitting(planeRandomPoints, center);
            double a=100*plapram[0], b=100*plapram[1], c=100*plapram[2], d=100*plapram[3];
            
            vector<cv::Point3f> vp3; //用于暂时保存Aruco的空间四点
        
            for(int k=0; k<4; k++) 
            {
                //注意要用去畸变的值。
                double um = mpCurrentKeyFrame->mvArucoUn[4*i+k].x;
                double vm = mpCurrentKeyFrame->mvArucoUn[4*i+k].y;
                double xzm = (um-cx1)*invfx1;
                double yzm = (vm-cy1)*invfy1;
                // 求 直线(0-二维码一角点)与拟合的二维码空间平面 角点，即要求lamda
                // x = 0+lamda*x0
                // y = 0+lamda*y0
                // z = 0+lamda*1
                // double a=100*plapram[0], b=100*plapram[1], c=100*plapram[2]; //乘上100，是为了避免vpt过小。
                double vpt = a*xzm+b*yzm+c;
                double lamda = (a*center[0] + b*center[1] + c*center[2])/vpt;
                cv::Mat posm=(cv::Mat_<float>(3,1)<< lamda*xzm, lamda*yzm, lamda); // in camera reference
                posm = Rcw.t()*posm - Rcw.t()*tcw; // in world reference
                //! 存储到mpTracker->mmAruMPs
                vp3.push_back(cv::Point3f(posm.at<float>(0), posm.at<float>(1), posm.at<float>(2) ));
            }

            
        
            cv::Point3f dd0 = vp3[1]-vp3[0];
            cv::Point3f dd1 = vp3[2]-vp3[1];
            cv::Point3f dd2 = vp3[3]-vp3[2];
            cv::Point3f dd3 = vp3[0]-vp3[3];
            double l0=cv::norm(dd0), l1=cv::norm(dd1), l2=cv::norm(dd2), l3=cv::norm(dd3); //四条边的边长
            double lmean = (l0+l1+l2+l3)/4; //平均边长
            double lm[4] = {abs(l0-lmean), abs(l1-lmean), abs(l2-lmean), abs(l3-lmean)}; //与平均边长的差值
            double maxLength = *max_element(lm,lm+4); //最大的差值
            if(maxLength < maxLengthDiff ){
                maxLengthDiff = maxLength;
                maxLenDiff[id] = maxLength;
                ArucoMeanLength[id] = lmean;
                cout<<"Aruco id = "<<id<<"; \033[32mMEAN Length = "<<lmean<<"\033[0m"<<endl;
                mAruMPs[id] = vp3;
                
                cv::Mat normalPlane = (cv::Mat_<float>(3,1)<<a/100,b/100,c/100);
                normalPlane = Rcw.t()*normalPlane - Rcw.t()*tcw;
                a=normalPlane.at<float>(0);
                b=normalPlane.at<float>(1);
                c=normalPlane.at<float>(2);
                // cout<<axlez<<"\n"<<normalPlane<<endl;
                double tmp = axlez.at<float>(0)*a + axlez.at<float>(1)*b + axlez.at<float>(2)*c;
                double theta = atan2( cv::norm(axlez.cross(normalPlane)),  tmp);
                double angle = theta*180/M_PI;
                cout<< "angle = "<< angle << endl;

                if(angle<15 || angle>165 && mpCurrentKeyFrame->isIdxArucoOld(i)==false) {
                    pMA->isWellComputed = true;
                }
                else if(angle>40 && angle<140 && mpCurrentKeyFrame->isIdxArucoOld(i)==false) {
                    // TODO: update MapAruco's params
                    pMA->nBadComputed++;
                    // cout<<"\033[31m pMA->nBadComputed++"<<pMA->nBadComputed<<"\033[0m"<<endl;
                }

            }
            
        
        }
        if(pMA->nBadComputed>=3 && pMA->isWellComputed==false && pMA->isBad()==false){
            // pMA->UpdateTcmByKF(mpCurrentKeyFrame, i);
            // pMA->SetRtwmByKeyFrame(mpCurrentKeyFrame->GetRotation().t(), mpCurrentKeyFrame->GetCameraCenter());
            pMA->SetBadFlag();
            cout<<"\033[31m pMA->SetBadFlag()========================\033[0m"<<endl;
        }       
    }
    // cout<<"****************************************"<<endl;

    if(mbIniT){
        DoScale = mpTracker->mbUseAruco;
        cout<<DoScale<<"\033[31m===========================------------doscale*************\033[0m"<<endl;
        mbIniT = false;
    }
    
    if(!DoScale)
    {
        if(maxLengthDiff < 0.015){ //* Do SCALE
        cout<<"\033[31m===========================------------doscale*************\033[0m"<<endl;
            int idx = -1;
            double maxl = 0.007;
            double maxll = 0.007;
            int cntaok=0;
            for(map<int, double>::iterator mit=maxLenDiff.begin(); mit!=maxLenDiff.end(); mit++){
                int i = mit->first;
                double d = mit->second;
                if(d<maxl){
                    // maxl = d;
                    if(d<maxll){
                        maxll = d;
                        idx=i;
                    }
                   
                    cntaok++;
                }
            }
            if(cntaok<3) 
                return ;
            double meanLen = ArucoMeanLength[idx];
            double arucoLen = Frame::mMarkerSize;
            float s = arucoLen/meanLen;

            {
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
            vector<MapPoint*> vpAllMP = mpMap->GetAllMapPoints();
            for(size_t iMP = 0; iMP<vpAllMP.size(); iMP++) {
                if(vpAllMP[iMP]){
                    MapPoint* pMP = vpAllMP[iMP];
                    pMP->SetWorldPos(pMP->GetWorldPos()*s);
                }
            }

            vector<KeyFrame*> vpAllKF = mpMap->GetAllKeyFrames();
            for(size_t iKF = 0; iKF<vpAllKF.size(); iKF++) {
                if(!vpAllKF[iKF]->isBad()) {
                    cv::Mat Tc2w = vpAllKF[iKF]->GetPose();
                    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*s;
                    vpAllKF[iKF]->SetPose(Tc2w);
                }
            }
            DoScale = true;
            Frame::mbUArucoIni = true;
            }

        }
    }

    /*double sumA=0; 
    int cnt=0;
    int cntsumA = 0;
    for(size_t i=0; i<numAruco; i++) {
        int iid = vA[i];
        map<int,double>::iterator mit = ArucoMeanLength.find(iid);
        if(mit!=ArucoMeanLength.end()) {
            sumA += ArucoMeanLength[iid];
            cntsumA++;
        }
    }
    sumA = sumA/cntsumA;
    for(size_t i=0; i<numAruco; i++) {
        int iid = vA[i];
        map<int,double>::iterator mit = ArucoMeanLength.find(iid);
        if(mit!=ArucoMeanLength.end()) {
            if(abs(ArucoMeanLength[iid]-sumA)<0.005)
                cnt++;
        }
    }
    if(cnt>=3 && (mLastDoneScale+15<nkfid ||mLastDoneScale==0) ){ //     !DoScale
        mLastDoneScale = nkfid;
        //TODO scale the map
        cout<<"\033[31m "<<sumA<<"\033[0m"<<endl;
        double scale = 0.165/sumA; //要放缩的比例        

        vector<KeyFrame*> vpAllKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();
        vpAllKeyFrames.push_back(mpCurrentKeyFrame);

        vector<MapPoint*> vpAllMapPoints;
        for(size_t iKF=0; iKF<vpAllKeyFrames.size(); iKF++)
        {
            set<MapPoint*> vpMPinCur = vpAllKeyFrames[iKF]->GetMapPoints();
            for(set<MapPoint*>::iterator sit=vpMPinCur.begin(), send=vpMPinCur.end(); sit!=send; sit++) {
                MapPoint* vpMP = *sit;
                if(vpMP->DoneScale == false) {
                    vpAllMapPoints.push_back(vpMP);
                    vpMP->DoneScale = true;
                }
            }
        }

        for(size_t iKF=0; iKF<vpAllKeyFrames.size(); iKF++)
        {
            if(vpAllKeyFrames[iKF]->DoneScale == true)
                continue;
            cv::Mat pCurPose = vpAllKeyFrames[iKF]->GetPose();
            pCurPose.col(3).rowRange(0,3) = pCurPose.col(3).rowRange(0,3)*scale;
            vpAllKeyFrames[iKF]->SetPose(pCurPose);
            vpAllKeyFrames[iKF]->DoneScale = true;
        }
        std::set<int> sAIdSave;
        for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
        {
            if(vpAllMapPoints[iMP])
            {
                MapPoint* pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos()*scale);
                int mpid=pMP->mArucoID;
                sAIdSave.insert(mpid);
            }
        }
        for(set<int>::iterator sit=sAIdSave.begin(), send=sAIdSave.end(); sit!=send; sit++)
        {
            int sid = *sit;
            set<int>::iterator sittmp = msDoneScaleArucoID.find(sid);
            if(sittmp == msDoneScaleArucoID.end()) {
                map<int,double>::iterator mitfind = ArucoMeanLength.find(sid);
                if(mitfind == ArucoMeanLength.end() )
                    continue;
                vector<cv::Point3f> &vp3 = mpTracker->mmAruMPs[sid];
                for(int i=0; i<4; i++){
                    vp3[i].x = scale * vp3[i].x;
                    vp3[i].y = scale * vp3[i].y;
                    vp3[i].z = scale * vp3[i].z;
                }
                ArucoMeanLength[sid] = ArucoMeanLength[sid]*scale;
                msDoneScaleArucoID.insert(sid);
            }
        }
        mpMap->UpdateAruco();

        for(map<int,double>::iterator mit=ArucoMeanLength.begin(), mend=ArucoMeanLength.end(); mit!=mend; mit++){
            cout<<"\033[31m Aruco id = "<<mit->first<<"; length = "<<mit->second<<"\033[0m"<<endl;
        }
        
        // DoScale = true;
        cout<<"\33[32m******* DO SCALE ***********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************\033[0m"<<endl;
    }
*/
    // cout<<"****************************************"<<endl;
}

Eigen::Vector4d LocalMapping::PlaneFitting(const vector<Eigen::Vector3d> &plane_pts, Eigen::Vector3d &center)
{
    center = Eigen::Vector3d::Zero();
    for (const auto & pt : plane_pts) 
        center += pt;
    center /= plane_pts.size();

    Eigen::MatrixXd A(plane_pts.size(), 3);
    for (int i = 0; i < plane_pts.size(); i++) {
        A(i, 0) = plane_pts[i][0] - center[0];
        A(i, 1) = plane_pts[i][1] - center[1];
        A(i, 2) = plane_pts[i][2] - center[2];
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
    const float a = svd.matrixV()(0, 2);
    const float b = svd.matrixV()(1, 2);
    const float c = svd.matrixV()(2, 2);
    const float d = -(a * center[0] + b * center[1] + c * center[2]);

    return Eigen::Vector4d(a, b, c, d);
}

void LocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=20;
    const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<KeyFrame*> vpTargetKFs;
    for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            KeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;

        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    vector<MapPoint*> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        KeyFrame* pKFi = *vitKF;

        vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPoint* pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        KeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFrame* pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  

        vector<MapAruco*> vpMapArucos = pKF->GetAllMapArucos();
        bool isArucoOK=true;
        for(auto ma: vpMapArucos)
        {
            MapAruco* pMA = ma;
            if(pMA)
            {
                if(!pMA->isBad())
                {
                    int obsAruco = pMA->Observations();
                    if(obsAruco<=5){
                        isArucoOK=false;
                        // cout<<"isArucoOK=false==============================================="<<endl;
                    }
                }
            }
        }

        if(nRedundantObservations>0.9*nMPs && isArucoOK)
            pKF->SetBadFlag();
    }
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
        mbIniT = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
        mbIniT = true;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

} //namespace ORB_SLAM
