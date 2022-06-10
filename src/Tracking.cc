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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>


using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0),
    IsInArucoReloc(false), mMayExistLoop(false)
{
    mbUseAruco = true;
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                // add TrackByAruco() !!! 这里忽略了很多条件，首先假设第一二帧都有marker
                // TODO 先做一个判断，判断是否有良好的Aruco可以提供初值
                // * 有的话，运行TrackByAruco()。
                // * 没有的话，依次选择TrackReferenceKeyFrame()、TrackWithMotionModel()
                bool isArucoGood = IsArucoWellTrack();
                if(isArucoGood)
                {
                    bOK = TrackByAruco();
                    // cout<<bOK<<"\t\033[33mTrackByAruco()\033[0m"<<endl;
                    if(!bOK && !mVelocity.empty()){
                        bOK = TrackWithMotionModel();
                        // cout<<bOK<<"\t\t\t\033[35mAfter TrackByMarker(), TrackWithMotionModel()\033[0m"<<endl; 
                    }
                    if(!bOK){
                        bOK = TrackReferenceKeyFrame();
                        // cout<<bOK<<"\t\t\033[34mAfter TrackByMarker(), TrackReferenceKeyFrame()\033[0m"<<endl;
                    }
                }
                else if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                    // cout<<bOK<<"\t\t\033[34mTrackReferenceKeyFrame()\033[0m"<<endl;
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    // cout<<bOK<<"\t\t\t\033[35mTrackWithMotionModel()\033[0m"<<endl;
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                if(mCurrentFrame.NA>0){
                    bOK = RelocalizationByAruco();
                    cout<<bOK<<" \033[34mRelocalizationByAruco();\033[0m"<<endl;
                    if(!bOK){
                        bOK = Relocalization();
                        cout<<bOK<<" \033[34m\tAfter-RelocalizationByAruco() ==> Relocalization();\033[0m"<<endl;
                    }
                        
                }
                else
                    bOK = Relocalization();
                // cout<<bOK<<"\t\t\t\033[32mRelocalization()\033[0m"<<endl;
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                if(mCurrentFrame.NA>0){
                    bOK = RelocalizationByAruco();
                    cout<<bOK<<" \033[34mRelocalizationByAruco();\033[0m"<<endl;
                    if(!bOK){
                        bOK = Relocalization();
                        cout<<bOK<<" \033[34m\tAfter-RelocalizationByAruco() ==> Relocalization();\033[0m"<<endl;
                    }
                }
                else
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map
                    bool isArucoGood = IsArucoWellTrack(); //IsArucoWellTrack();
                    if(isArucoGood)
                    {
                        bOK = TrackByAruco();
                        // cout<<bOK<<"\t\033[33mTrackByAruco()\033[0m"<<endl;
                        // if(!bOK && !mVelocity.empty()){
                        //     bOK = TrackWithMotionModel();
                        //     // cout<<bOK<<"\t\t\t\033[35mAfter TrackByMarker(), TrackWithMotionModel()\033[0m"<<endl; 
                        // }
                        if(!bOK){
                            bOK = TrackReferenceKeyFrame();
                            // cout<<bOK<<"\t\t\033[34mAfter TrackByMarker(), TrackReferenceKeyFrame()\033[0m"<<endl;
                        }
                    }
                    else if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                        // cout<<bOK<<"\t\t\033[34mTrackReferenceKeyFrame()\033[0m"<<endl;
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    if(mCurrentFrame.NA>0){
                        bOKReloc = RelocalizationByAruco();
                        cout<<bOKReloc<<" \033[34mRelocalizationByAruco();\033[0m"<<endl;
                        if(!bOKReloc){
                            bOKReloc = Relocalization();
                            cout<<bOKReloc<<" \033[34m\tAfter-RelocalizationByAruco() ==> Relocalization();\033[0m"<<endl;
                        }
                    }
                    else
                        bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc) 
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;  //!!
                // cout<<"mVelocity's ||t|| = "<<cv::norm(mVelocity.rowRange(0,3).col(3) )<<endl<<"++++++++++++++++++++"<<endl;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            cout<<mCurrentFrame.mTimeStamp<<endl;

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
        // vector<bool> vbTriAruco;

        bool bIniA = false;
        //TODO: use Aruco's POSE to initialize -- FIRST
        if(!bIniA)
        {
            vector<cv::Mat> vmR21A;
            vector<cv::Mat> vmt21A;
            vector<pair<int,int>> vp;
            for(int i=0; i<mInitialFrame.NA; i++){
                if(!mInitialFrame.mvbArucoGood[i]) //测得不准的直接跳过
                    continue;
                for(int j=0; j<mCurrentFrame.NA; j++){
                    if(mInitialFrame.mvMarkers[i].id == mCurrentFrame.mvMarkers[j].id){
                        if(mCurrentFrame.mvbArucoGood[j] == true)
                            vp.push_back(make_pair(i,j));
                        break;
                    }
                }
            }
            
            int numA = vp.size();
            float length = Frame::mMarkerSize;
            const float fx = mK.at<float>(0,0);
            const float fy = mK.at<float>(1,1);
            const float cx = mK.at<float>(0,2);
            const float cy = mK.at<float>(1,2);
    
            // TODO 存储1、2帧上对应二维码计算得到的位姿R和t
            for(int i=0; i<numA; i++)
            {
                int n1 = vp[i].first;
                int n2 = vp[i].second;
                aruco::Marker pam1 = mInitialFrame.mvMarkers[n1];
                aruco::Marker pam2 = mCurrentFrame.mvMarkers[n2];
                // cout<<i<<" id = "<<pam1.id<<endl;
                cv::Mat rr1,rr2;
                cv::Rodrigues(pam1.Rvec, rr1);
                cv::Rodrigues(pam2.Rvec, rr2);
                cv::Mat r21 = rr2*rr1.t();
                cv::Mat t21 = -rr2*rr1.t()*pam1.Tvec+pam2.Tvec;
                //* 保证间距
                if(cv::norm(t21)<0.1)
                    continue;
                
                //TODO: judge the accurracy
                cv::Mat c0=(cv::Mat_<float>(3,1)<<-length/2, length/2, 0);
                cv::Mat c1=(cv::Mat_<float>(3,1)<< length/2, length/2, 0);
                cv::Mat c2=(cv::Mat_<float>(3,1)<< length/2,-length/2, 0);
                cv::Mat c3=(cv::Mat_<float>(3,1)<<-length/2,-length/2, 0);
                vector<cv::Mat> a3d1;
                a3d1.resize(4);
                a3d1[0] = rr1*c0+pam1.Tvec;
                a3d1[1] = rr1*c1+pam1.Tvec;
                a3d1[2] = rr1*c2+pam1.Tvec;
                a3d1[3] = rr1*c3+pam1.Tvec;
                vector<cv::Mat> a2d1, a2d2;
                a2d1.resize(4);
                a2d2.resize(4);
                a2d1[0] = (cv::Mat_<float>(2,1)<<mInitialFrame.mvArucoUn[4*n1].x,   mInitialFrame.mvArucoUn[4*n1].y);
                a2d1[1] = (cv::Mat_<float>(2,1)<<mInitialFrame.mvArucoUn[4*n1+1].x, mInitialFrame.mvArucoUn[4*n1+1].y);
                a2d1[2] = (cv::Mat_<float>(2,1)<<mInitialFrame.mvArucoUn[4*n1+2].x, mInitialFrame.mvArucoUn[4*n1+2].y);
                a2d1[3] = (cv::Mat_<float>(2,1)<<mInitialFrame.mvArucoUn[4*n1+3].x, mInitialFrame.mvArucoUn[4*n1+3].y);

                a2d2[0] = (cv::Mat_<float>(2,1)<<mCurrentFrame.mvArucoUn[4*n2].x,   mCurrentFrame.mvArucoUn[4*n2].y);
                a2d2[1] = (cv::Mat_<float>(2,1)<<mCurrentFrame.mvArucoUn[4*n2+1].x, mCurrentFrame.mvArucoUn[4*n2+1].y);
                a2d2[2] = (cv::Mat_<float>(2,1)<<mCurrentFrame.mvArucoUn[4*n2+2].x, mCurrentFrame.mvArucoUn[4*n2+2].y);
                a2d2[3] = (cv::Mat_<float>(2,1)<<mCurrentFrame.mvArucoUn[4*n2+3].x, mCurrentFrame.mvArucoUn[4*n2+3].y);
                
                float err = 0;
                for(int k=0; k<4; k++){
                    float ex1 = ((a3d1[k].at<float>(0)/a3d1[k].at<float>(2))*fx+cx) - a2d1[k].at<float>(0);
                    float ey1 = ((a3d1[k].at<float>(1)/a3d1[k].at<float>(2))*fy+cy) - a2d1[k].at<float>(1);
                    cv::Mat a3d2 = r21*a3d1[k] + t21;
                    float ex2 = ((a3d2.at<float>(0)/a3d2.at<float>(2))*fx+cx) - a2d2[k].at<float>(0);
                    float ey2 = ((a3d2.at<float>(1)/a3d2.at<float>(2))*fy+cy) - a2d2[k].at<float>(1);
                    err += ex1*ex1 + ey1*ey1 + ex2*ex2 + ey2*ey2;
                }
                //! MODIFIED
                cout<<"err = "<<err<<endl;
                if(err>0.5)
                    continue;
                vmR21A.push_back(r21);
                vmt21A.push_back(t21);
            }
            
            int bestIdA=-1;
            bIniA = mpInitializer->InitializeUseAruco(mCurrentFrame,mvIniMatches, vmR21A, vmt21A,mvIniP3D, vbTriangulated, bestIdA);
            // bool bIniA = mpInitializer->InitializeUseAruco(mCurrentFrame,mvIniMatches, vAky1, vAky2, vM12, vmR21A, vmt21A,mvIniP3D, vbTriangulated, bestIdA, mvIniP3AD, vbTriAruco);

            if(bIniA)
            {
                for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
                {
                    if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                    {
                        mvIniMatches[i]=-1;
                        nmatches--;
                    }
                }
                mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
                cout<<"bestIdA = "<<bestIdA<<endl;
                vmR21A[bestIdA].copyTo(Tcw.rowRange(0,3).colRange(0,3));
                vmt21A[bestIdA].copyTo(Tcw.rowRange(0,3).col(3));
                mCurrentFrame.SetPose(Tcw);
                cout<<"------------------------vmt21A[bestIdA]'s norm = "<<cv::norm(vmt21A[bestIdA])<<endl;
                cout<<mCurrentFrame.mTimeStamp<<endl; //!
                
                mbUseAruco = true;
                Frame::mbUArucoIni = true;
                CreateInitialMapMonocular();
                return ;
            }
        }
        //TODO: use ORB_SLAM method to initialize -- SECOND
        else //if(!bIniA)
        {
            bool bIni = mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated);
            if(bIni)
            {
                cout<<"-----------------------!!!!!!!!!!!!!!!1"<<endl;
                for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
                {
                    if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                    {
                        mvIniMatches[i]=-1;
                        nmatches--;
                    }
                }

                // Set Frame Poses
                mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
                Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(Tcw.rowRange(0,3).col(3));
                mCurrentFrame.SetPose(Tcw);
                mbUseAruco = false;
                Frame::mbUArucoIni = false;
                CreateInitialMapMonocular();
            }
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
    //* 两个kf都有pose

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    // 先全局优化之后加上Aruco的信息
    // 1.pKFini; 2.pKFcur.
    vector<int> viId;
    float s = 0.165;
    for(size_t i=0; i<mInitialFrame.NA; i++)
    {
        if(mInitialFrame.mvbArucoGood[i] == false) 
            continue;
        aruco::Marker xM = mInitialFrame.mvMarkers[i];
        MapAruco* pNewMA = new MapAruco(xM, pKFini, mpMap, s);
        viId.push_back(xM.id);
        pNewMA->SetRtwm(cv::Mat::eye(3,3,CV_32F), cv::Mat::zeros(3,1,CV_32F) );

        pNewMA->AddObservation(pKFini,i);
        pKFini->AddMapAruco(pNewMA,i);
        mpMap->AddMapAruco(pNewMA);
        mInitialFrame.mvpMapArucos[i]=pNewMA;
        
        // associate aruco with the second kf
        for(size_t j=0; j<mCurrentFrame.NA; j++)
        {
            int jd = mCurrentFrame.mvMarkers[j].id;
            if(jd == xM.id) {
                pNewMA->AddObservation(pKFcur,j);
                pKFcur->AddMapAruco(pNewMA,j);
                mCurrentFrame.mvpMapArucos[j]=pNewMA;
            }
        }

    }
    // 2.pKFcur可能会有重复的Aruco id号
    // cout << "out...  1.pKFini;"<<endl;
    for(size_t i=0; i<mCurrentFrame.NA; i++)
    {
        if(mCurrentFrame.mvbArucoGood[i] == false)
            continue;
        int cid = mCurrentFrame.mvMarkers[i].id;
        // cout << "In second for"<<endl;
        if( find(viId.begin(), viId.end(), cid)==viId.end() )
        {
            // cout << "In CreateInitialMapMonocular()... 2.pKFcur."<<endl;
            // 即Aruco的id没有与mInitialFrame中重复，则add
            aruco::Marker xM = mCurrentFrame.mvMarkers[i];
            MapAruco* pNewMA = new MapAruco(xM, pKFcur, mpMap, s);
            pNewMA->SetRtwm(pKFcur->GetRotation().t(), pKFcur->GetCameraCenter());

            pNewMA->AddObservation(pKFcur,i);
            pKFcur->AddMapAruco(pNewMA,i);
            mpMap->AddMapAruco(pNewMA); 
            mCurrentFrame.mvpMapArucos[i]=pNewMA;

            
        }
    }

    cout<<"before GBA"<<endl;
    Optimizer::GlobalBundleAdjustemnt(mpMap,20); //* 没想到这里突然来了个全局BA   

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);
    cout<<"pKFcur->GetPose()-- t's norm = "<<cv::norm(pKFcur->GetCameraCenter())<<endl;
    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::Triangulate(cv::Point2f &kp1, cv::Point2f &kp2, cv::Mat &P1, cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.y*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

// 给TrackReferenceKeyFrame中的F->mvpMapArucos[i]赋值
// 只要是地图中有的Aruco ID号，都会赋到mvpMapArucos[i]中去，所以存在可能会形成回环的Aruco。
//// 现在改成只从 上一帧or参考帧赋值
void Tracking::CheckArucoID()
{
    // mMayExistLoop = false;
    vector<MapAruco*> vpMA = mpMap->GetAllMapArucos();
    for(size_t k=0; k<vpMA.size(); k++)
    {
        MapAruco* pMA = vpMA[k];
        int aid = pMA->GetMapArucoID();
        for(size_t i=0; i<mCurrentFrame.mvMarkers.size(); i++)
        {
            int iid = mCurrentFrame.mvMarkers[i].id;
            if(aid == iid)
            {
                mCurrentFrame.mvpMapArucos[i] = pMA;
            }
        }
    }
    
    //判断是否有形成回环的Aruco
    KeyFrame* pKFRef = mpReferenceKF;
    set<KeyFrame*> spConnectedKeyFrames = pKFRef->GetConnectedKeyFrames();
    spConnectedKeyFrames.insert(pKFRef); //从而剔除新来的Aruco
    long unsigned int KFIdMin = INT32_MAX;
    // long unsigned int KFIdMax = 1; 找到一个最小的kf id，用于判断出现的Aruco是否有
    for(set<KeyFrame*>::iterator sit=spConnectedKeyFrames.begin(), send=spConnectedKeyFrames.end();sit!=send;sit++){
        KeyFrame* pKF = *sit;
        long unsigned int id = pKF->mnId;
        if(id<KFIdMin)
            KFIdMin = id;
    }

    // 遍历当前帧所有Aruco，保存每个aruco不在spConnectedKeyFrames的KF
    for(size_t i=0; i<mCurrentFrame.NA; i++){
        int ida = mCurrentFrame.mvMarkers[i].id;
        MapAruco* pMA = mpMap->GetMapAruco(ida);
        if(pMA && !pMA->isBad()) {
            map<KeyFrame*, size_t> mAO = pMA->GetObservations();
            for(map<KeyFrame*, size_t>::iterator mit=mAO.begin(), mend=mAO.end(); mit!=mend; mit++)
            {
                KeyFrame* pKFi= mit->first;
                if(pKFi->mnId > KFIdMin) 
                    continue;
                if(pKFi->isBad())
                    continue;
                if(!spConnectedKeyFrames.count(pKFi)){
                    mCurrentFrame.mvbOldAruco[i] = true;
                    // mMayExistLoop = true;
                }
            }
        }
    }

}

bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
    // cout<<"In TrackRefernceKeyFrame(), nmarches = "<<nmatches<<endl;

    // Add by liujiamin
    // TODO: make frame.mvpMapArucos[idx] have value
    CheckArucoID();

    if(nmatches<10) // 15
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;

    //! Because Tracking::TrackReferenceKeyFrame() also use for relocalization by Aruco
    if(IsInArucoReloc){
        mCurrentFrame.SetPose(mTcwTemp );//!
    }
    else 
        mCurrentFrame.SetPose(mLastFrame.mTcw);

    // if(!mbOnlyTracking)
    Optimizer::PoseOptimizationByAruco(&mCurrentFrame);
    // else 
    // Optimizer::PoseOptimization(&mCurrentFrame);

    

    // Discard outliers
    int nmatchesMap = 0;
    int nmvpMP=0;
    int nout=0;
    int nNull=0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            nmvpMP++;
            if(mCurrentFrame.mvbOutlier[i])
            {
                nout++;
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0){
                nmatchesMap++;
            } else
            {
                nNull++;
            }
            
        }
    }

    // cout<<"in TrackReferenceKeyFrame nmvpMP = "<<nmvpMP<<endl;
    // cout<<"in TrackReferenceKeyFrame nout = "<<nout<<endl;
    // cout<<"in TrackReferenceKeyFrame nNull = "<<nNull<<endl;
    // cout<<"in TrackReferenceKeyFrame nmatchesMap = "<<nmatchesMap<<endl;
    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    return;
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();
    // Add by liujiamin
    // TODO: make frame.mvpMapArucos[idx] have value
    CheckArucoID();
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    // if(!mbOnlyTracking)
    Optimizer::PoseOptimizationByAruco(&mCurrentFrame);
    // else 
    // Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }


    return nmatchesMap>=10;
}

bool Tracking::IsArucoWellTrack()
{

    // TODO get best R,t by Aruco
    // step 1: find Aruco in both last and current frame 并且二维码投影误差小，即mvbArucoGood[...] == true
    vector<pair<int,int>> vp;
    for(size_t i=0; i<mLastFrame.NA; i++) {
        if(!mLastFrame.mvbArucoGood[i])
            continue;
        for(size_t j=0; j<mCurrentFrame.NA; j++) {
            if(mLastFrame.mvMarkers[i].id == mCurrentFrame.mvMarkers[j].id) {
                if(mCurrentFrame.mvbArucoGood[j] == true)
                    vp.push_back(make_pair(i,j));
                break;
            }
        }
    }
    if(vp.size() == 0)
        return false;
    
    mR21A= cv::Mat::eye(3,3,CV_32F); 
    mt21A= cv::Mat::zeros(3,1,CV_32F);
    int numA = vp.size();
    vector<cv::Mat> a3d1;
    a3d1.resize(4);
    vector<cv::Mat> a2d1, a2d2;
    a2d1.resize(4);
    a2d2.resize(4);
    // vector<cv::Mat> vr1;
    float length = 0.165;
    const float fx = mK.at<float>(0,0);
    const float fy = mK.at<float>(1,1);
    const float cx = mK.at<float>(0,2);
    const float cy = mK.at<float>(1,2);
    float errMin=2; //!
    int bestA = -1;

    for(int i=0; i<numA; i++)
    {
        int n1 = vp[i].first;  // mLastFrame
        int n2 = vp[i].second; // mCurrentFrame

        aruco::Marker pam1 = mLastFrame.mvMarkers[n1];
        aruco::Marker pam2 = mCurrentFrame.mvMarkers[n2];
        
        cv::Mat rr1,rr2;
        cv::Rodrigues(pam1.Rvec, rr1);
        cv::Rodrigues(pam2.Rvec, rr2);

        cv::Mat r21 = rr2*rr1.t();
        cv::Mat t21 = -rr2*rr1.t()*pam1.Tvec+pam2.Tvec;
        // cout<<"||t21|| = "<<cv::norm(t21)<<endl;

        cv::Mat c0=(cv::Mat_<float>(3,1)<<-length/2, length/2, 0);
        cv::Mat c1=(cv::Mat_<float>(3,1)<< length/2, length/2, 0);
        cv::Mat c2=(cv::Mat_<float>(3,1)<< length/2,-length/2, 0);
        cv::Mat c3=(cv::Mat_<float>(3,1)<<-length/2,-length/2, 0);

        a3d1[0] = rr1*c0+pam1.Tvec;
        a3d1[1] = rr1*c1+pam1.Tvec;
        a3d1[2] = rr1*c2+pam1.Tvec;
        a3d1[3] = rr1*c3+pam1.Tvec;

        a2d1[0] = (cv::Mat_<float>(2,1)<<mLastFrame.mvArucoUn[4*n1].x,   mLastFrame.mvArucoUn[4*n1].y);
        a2d1[1] = (cv::Mat_<float>(2,1)<<mLastFrame.mvArucoUn[4*n1+1].x, mLastFrame.mvArucoUn[4*n1+1].y);
        a2d1[2] = (cv::Mat_<float>(2,1)<<mLastFrame.mvArucoUn[4*n1+2].x, mLastFrame.mvArucoUn[4*n1+2].y);
        a2d1[3] = (cv::Mat_<float>(2,1)<<mLastFrame.mvArucoUn[4*n1+3].x, mLastFrame.mvArucoUn[4*n1+3].y);

        a2d2[0] = (cv::Mat_<float>(2,1)<<mCurrentFrame.mvArucoUn[4*n2].x,   mCurrentFrame.mvArucoUn[4*n2].y);
        a2d2[1] = (cv::Mat_<float>(2,1)<<mCurrentFrame.mvArucoUn[4*n2+1].x, mCurrentFrame.mvArucoUn[4*n2+1].y);
        a2d2[2] = (cv::Mat_<float>(2,1)<<mCurrentFrame.mvArucoUn[4*n2+2].x, mCurrentFrame.mvArucoUn[4*n2+2].y);
        a2d2[3] = (cv::Mat_<float>(2,1)<<mCurrentFrame.mvArucoUn[4*n2+3].x, mCurrentFrame.mvArucoUn[4*n2+3].y);
        
        float err = 0;
        for(int k=0; k<4; k++){
            float ex1 = ((a3d1[k].at<float>(0)/a3d1[k].at<float>(2))*fx+cx) - a2d1[k].at<float>(0);
            float ey1 = ((a3d1[k].at<float>(1)/a3d1[k].at<float>(2))*fy+cy) - a2d1[k].at<float>(1);
            cv::Mat a3d2 = r21*a3d1[k] + t21;
            float ex2 = ((a3d2.at<float>(0)/a3d2.at<float>(2))*fx+cx) - a2d2[k].at<float>(0);
            float ey2 = ((a3d2.at<float>(1)/a3d2.at<float>(2))*fy+cy) - a2d2[k].at<float>(1); 
            err += ex1*ex1 + ey1*ey1 + ex2*ex2 + ey2*ey2; 
        }
        if(err<errMin)
        {
            errMin = err;
            mR21A = r21;
            mt21A = t21;
            // cout << "mt21A's ||t|| = "<<cv::norm(mt21A)<<endl;
            bestA = i;
            // cout<<"track by id = "<<pam2.id<<endl;
        }
    }

    if(bestA == -1){
        
        return false;
    } 
    else {
        if(cv::norm(mt21A)>0.3) {
            // cout<<"\033[31m cv::norm(mt21A)>0.3 \033[0m"<<endl;
            return false;
        }

        return true;
    }
        
}

bool Tracking::TrackByAruco()
{
    ORBmatcher matcher(0.9,true);
    UpdateLastFrame();
    CheckArucoID();
    
    cv::Mat T21 = cv::Mat::eye(4,4,CV_32F);
    mR21A.copyTo(T21.rowRange(0,3).colRange(0,3) );
    mt21A.copyTo(T21.rowRange(0,3).col(3) );
    // cout << "Aruco's ||t|| = "<<cv::norm(mt21A)<<endl;
    mCurrentFrame.SetPose( T21 * mLastFrame.mTcw );

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
    int th =10;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    // 如果跟踪的点少，则扩大搜索半径再来一次
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR); // 2*th
    }

    if(nmatches<20){
        cout<<"nmatches<20"<<endl;
        return false;
    }

    // if(!mbOnlyTracking)
    Optimizer::PoseOptimizationByAruco(&mCurrentFrame);
    // else 
    // Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    int nout = 0;
    int nmvpMP = 0;
    int nNull = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            nmvpMP++;
            if(mCurrentFrame.mvbOutlier[i])
            {
                nout++;
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
            else
                nNull++;
        }
    }


    // cout<<"in TrackByAruco nmvpMP = "<<nmvpMP<<endl;
    // cout<<"in TrackByAruco nout = "<<nout<<endl;
    // cout<<"in TrackByAruco nNull = "<<nNull<<endl;
    // cout<<"in TrackByAruco nmatchesMap = "<<nmatchesMap<<endl;

    // cout<< (nmatchesMap>=10) <<"\t\tnmatchesMap>=10"<<endl;
    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();//在局部地图中查找在当前帧视野范围内的点，将视野范围内的点和当前帧的特征点进行投影匹配

    //* 上两步是给mCurrentFrame.mvpMapPoints重新处理了一下
    //* 此处再来一个PoseOptimization主要是调整MapPoint的
    //* 所以可能上两个函数对Aruco的影响不大
    // Optimize Pose
    // if(!mbOnlyTracking)
    Optimizer::PoseOptimizationByAruco(&mCurrentFrame);
    // else 
    // Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }


    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    // TODO: If see new Aruco, return true
    CheckArucoID();
    for(size_t i=0; i<mCurrentFrame.NA; i++)
    {
        if(mCurrentFrame.mvbArucoGood[i] == false) continue;
        MapAruco* pMA = mCurrentFrame.mvpMapArucos[i];
        if(!pMA){
            cout<<"Need new key frame, because of new aruco."<<endl;
            return true;
        }
    }

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    // if(mSensor==System::MONOCULAR)
    //     thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    // Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;
    // CheckArucoID();

    // //* it is about adding Aruco to the map
    // // ! 改到LocalMapping线程中加入
    // TODO: adding Aruco to the map --- MONOCULAR
    // // cout<<"in create a new key frame ===================="<<endl;
    if(mSensor==System::MONOCULAR)
    {
        int na = mCurrentFrame.NA;
        for(size_t i=0; i<na; i++)
        {
            bool bCreateNewMapAruco = false;
            if(mCurrentFrame.mvbArucoGood[i] == false)
                continue;
            MapAruco* pMA = mCurrentFrame.mvpMapArucos[i];
            //* 详细注解见STEREO部分
            if(!pMA) 
            {
                cout<<"pMA is NULL"<<endl;
                bCreateNewMapAruco = true;
            }
            else if(pMA->Observations()<1)
            {
                cout<<"pMA->Observations() < 1"<<endl;
                bCreateNewMapAruco = true;
            }
            else{
                pMA->AddObservation(pKF, i);
            }
            aruco::Marker xM = mCurrentFrame.mvMarkers[i];
            // float s=0.165; // SPM Datasets is 0.165
            if(bCreateNewMapAruco)
            {
                MapAruco* pNewMA = new MapAruco(xM, pKF, mpMap, Frame::mMarkerSize);
                cv::Mat r=pKF->GetRotation().t();
                cv::Mat t=pKF->GetCameraCenter();
                // cout<<r<<endl;
                // cout<<t<<endl;
                pNewMA->SetRtwm(r, t);
                // cv::Mat T_get = pNewMA->GetTwm();
                // cout<<"T_get = "<<T_get<<endl;
                pNewMA->AddObservation(pKF, i);
                pKF->AddMapAruco(pNewMA, i);
                mpMap->AddMapAruco(pNewMA);
                mCurrentFrame.mvpMapArucos[i]=pNewMA;
                cout<<"-----ckf-------New Aruco's ID: "<<xM.id<<endl;

            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

/**
 * @brief 对 Local MapPoints 进行跟踪
 * 
 * 在局部地图中查找在当前帧视野范围内的点，将视野范围内的点和当前帧的特征点进行投影匹配
 */
void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::RelocalizationByAruco()
{
    cout<< mCurrentFrame.mTimeStamp <<"\t\033[31m In RelocalizationByAruco\033[0m"<<endl;
    vector<int> vAllId = mpMap->GetAllMapArucoID();
    // vector<int> vIdInFrame;
    bool isOk = false;
    mCurrentFrame.ComputeBoW(); //!!!!!!!!!!!!!!!!!!!!!!!!!

    IsInArucoReloc = true;

    for(int i=0; i<mCurrentFrame.NA; i++)
    {
        if(!mCurrentFrame.mvbArucoGood[i])
            continue;
        int ida = mCurrentFrame.mvMarkers[i].id;
        vector<int>::iterator vit = find(vAllId.begin(), vAllId.end(), ida);
        if(vit == vAllId.end()) //没有此二维码
            continue;
        
        // have this aruco in map
        MapAruco* pMA = mpMap->GetMapAruco(ida);
        if(pMA->isBad())
            continue;
        cout<<"!pMA->isBad()"<<endl;

        cv::Mat Rcm;
        cv::Rodrigues(mCurrentFrame.mvMarkers[i].Rvec, Rcm);
        cout<<"Rodrigues"<<endl;

        cv::Mat Rwm = pMA->GetRw2m();
        cv::Mat twm = pMA->Gettw2m();

        cv::Mat Rcw = Rcm * Rwm.t();
        cv::Mat tcw = - Rcm*Rwm.t()*twm + mCurrentFrame.mvMarkers[i].Tvec;

        cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
        Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(Tcw.rowRange(0,3).col(3)       );
        Tcw.copyTo(mCurrentFrame.mTcw);
        cout<<"Tcw.copyTo(mCurrentFrame.mTcw);"<<endl;

        map<KeyFrame*, size_t> mapObs = pMA->GetObservations();

        vector<MapPoint*> vpMapPointMatches;
        cout<<"mapObs.size() = "<<mapObs.size()<<endl;
        // vvpMapPointMatches.resize(mapObs.size());
        for(map<KeyFrame*, size_t>::const_iterator mit=mapObs.begin(), mend=mapObs.end(); mit!=mend; mit++)
        {
            KeyFrame* pKF = mit->first;
            mpReferenceKF = pKF;
            mTcwTemp = Tcw;
            isOk = TrackReferenceKeyFrame();
            cout<<"TrackReferenceKeyFrame() = " <<isOk<<endl;
            if(isOk)
            {
                mnLastRelocFrameId = mCurrentFrame.mnId;
                IsInArucoReloc = false;
                return true;
            }   

        }
        
    }
    
    IsInArucoReloc = false;
    if(!isOk)
        return false;
    else {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }


    
}

bool Tracking::Relocalization()
{
    cout<< mCurrentFrame.mTimeStamp <<"\t\033[33m In Relocalization\033[0m"<<endl;
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            cout<<"nmatches = "<<nmatches<<endl;
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }
                int nGood;
                // if(!mbOnlyTracking)
                    nGood = Optimizer::PoseOptimizationByAruco(&mCurrentFrame);
                // else 
                // nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                cout<<"nGood = "<<nGood<<endl;
                
                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        // if(!mbOnlyTracking)
                            nGood = Optimizer::PoseOptimizationByAruco(&mCurrentFrame);
                        // else 
                        // nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                // if(!mbOnlyTracking)
                                    nGood = Optimizer::PoseOptimizationByAruco(&mCurrentFrame);
                                // else 
                                // nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mbUseAruco = false;
    Frame::mbUArucoIni = true;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
