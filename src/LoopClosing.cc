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

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>


namespace ORB_SLAM2
{
int LoopClosing::countForErase=0;

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0)
{
    mnCovisibilityConsistencyTh = 3;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            if(DetectLoop())
            {
                if(mbExistLoopByAruco){
                    bool b = ComputeSim3ByAruco();
                    if(b)
                        CorrectLoopByAruco();
                    else{
                        if(ComputeSim3())
                            CorrectLoopByAruco();
                    }
                }
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
            //    if(ComputeSim3())
            //    {
            // //        // Perform loop fusion and pose graph optimization
            //        CorrectLoop();
            //    }
            }
        }       

        ResetIfRequested();

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::DetectLoopByAruco()
{
    int cnt = 0;
    for(size_t i=0; i<mpCurrentKF->NA; i++)
    {
        if(mpCurrentKF->isIdxArucoOld(i) == false)
            cnt++;
    }
    if(cnt == mpCurrentKF->NA)
        return false;

    set<KeyFrame*> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();
    spConnectedKeyFrames.insert(mpCurrentKF); //从而剔除新来的Aruco
    long unsigned int KFIdMin = INT32_MAX;
    // long unsigned int KFIdMax = 1; 找到一个最小的kf id，用于判断出现的Aruco是否有
    for(set<KeyFrame*>::iterator sit=spConnectedKeyFrames.begin(), send=spConnectedKeyFrames.end();sit!=send;sit++){
        KeyFrame* pKF = *sit;
        long unsigned int id = pKF->mnId;
        if(id<KFIdMin)
            KFIdMin = id;
    }

    // 遍历当前帧所有Aruco，保存每个aruco不在spConnectedKeyFrames的KF
    map<int, set<KeyFrame*>> mIandK;
    for(size_t i=0; i<mpCurrentKF->NA; i++){
        int ida = mpCurrentKF->mvMarkers[i].id;
        MapAruco* pMA = mpCurrentKF->GetMapAruco(i);
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
                    mIandK[ida].insert(pKFi);
                }
            }
        }
    }
    if(mIandK.empty()){
        // * 是否此处有要clear的项
        mmIDandKFs.clear();
        return false;
    }

    // !注意区分是新的Aruco。== 第二行解决
    // 在mmIDandKFs内保存与闭环相关的
    // vector<int> vAllId = mpMap->GetAllMapArucoID();
    for(map<int, set<KeyFrame*>>::iterator mit=mIandK.begin(), mend=mIandK.end(); mit!=mend; mit++)
    {
        int ida = mit->first;
        
        if(mmIDandKFs.count(ida) ){
            for(set<KeyFrame*>::iterator sit=mit->second.begin(), send=mit->second.end(); sit!=send; sit++ )
            {
                KeyFrame* pKFs = *sit;
                if(pKFs->isBad())
                    continue;
                // if(!mmIDandKFs[ida].count(pKFs)){
                    mmIDandKFs[ida].insert(pKFs);
                // }
            }
        }
        else{
            mmIDandKFs[ida]=mIandK[ida];
        }
    }

    // cout<<"mmIDandKFs is OK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\t"<<mmIDandKFs.size()<<endl;
    return true;
}

bool LoopClosing::DetectLoop()
{
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }
    
    if(mpCurrentKF->mnId<10){
        for(size_t i=0; i<mpCurrentKF->NA; i++)
        {
            msiArucoAllID.insert(mpCurrentKF->mvMarkers[i].id);
        }
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();

        // TODO 

        return false;
    }

    //* 在此处检查是否可通过Aruco判断有回环形成
    mbExistLoopByAruco = false;
    //  没有Aruco ==> 依旧可以通过BoW判断是否有回环
    //  有Aruco，但是没有很有前曾经见过的Aruco ==> 代表没有回环，可不必再通过BoW检测。
    //? 如果bExistLoopByAruco==false，则是不是可以代表不用进行之后的操作了？
    //* 但此处复杂的问题是：此函数后期涉及的一些变量，在校正回环时依旧会使用
    if(mpCurrentKF->NA != 0) {
        mbExistLoopByAruco = DetectLoopByAruco();             
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;

    // Add Current Keyframe to database
    mpKeyFrameDB->add(mpCurrentKF);
    //! Add Aruco Information
    //! msiArucoTrackID.insert();
    // mbExistLoopByAruco = false;
    if(mbExistLoopByAruco){
        cout<<"DetectLoopByAruco====="<<endl;
        return true;
    }
    else if(mpCurrentKF->NA!=0 && !mbExistLoopByAruco){
        // 此步是为了留多一点关键帧
        countForErase++;
        if(countForErase>5){
            countForErase = 0;
        }
        else 
            mpCurrentKF->SetErase();
        return false;
    }
    else{

        if(mvpEnoughConsistentCandidates.empty())
        {
            countForErase++;
            if(countForErase>5){
                countForErase = 0;
            }
            else 
                mpCurrentKF->SetErase();
            return false;
        }
        else
        {
            return true;
        }

        mpCurrentKF->SetErase();
        return false;
    }
}

bool LoopClosing::ComputeSim3ByAruco()
{
    for(unordered_map<int, set<KeyFrame*>>::iterator mapit=mmIDandKFs.begin(), mapend=mmIDandKFs.end();
        mapit!=mapend; mapit++)
    {
        int idx = mapit->first;
        for(set<KeyFrame*>::iterator sit=mapit->second.begin(), send=mapit->second.end(); sit!=send; sit++)
        {
            KeyFrame* pKF = *sit; // 与当前关键帧算Sim3的候选帧
            MapAruco* pMA = mpMap->GetMapAruco(idx);
            if(pKF->isBad()) continue;
            if( !(pMA && !pMA->isBad()) ) continue;

            int idx1 = pKF->mmArucoIdandIdx[idx];           // idx in candidate
            int idx2 = mpCurrentKF->mmArucoIdandIdx[idx];    // idx in cur

            cv::Mat T_can_w = pKF->GetPose();
            cv::Mat Rvec_can_m = pKF->mvMarkers[idx1].Rvec;
            cv::Mat tvec_can_m = pKF->mvMarkers[idx1].Tvec;
            cv::Mat R_can_m;
            cv::Rodrigues(Rvec_can_m, R_can_m);
            cv::Mat R_m_can = R_can_m.t();
            cv::Mat tvec_m_can = - R_can_m.t() * tvec_can_m;

            cv::Mat Rvec_cur_m = mpCurrentKF->mvMarkers[idx2].Rvec;
            cv::Mat tvec_cur_m = mpCurrentKF->mvMarkers[idx2].Tvec;
            cv::Mat R_cur_m;
            cv::Rodrigues(Rvec_cur_m, R_cur_m);

            g2o::Sim3 g_cur_can(Converter::toMatrix3d(R_cur_m*R_m_can),Converter::toVector3d(R_cur_m*tvec_m_can+tvec_cur_m),1.0);
            g2o::Sim3 g_can_w(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
            mg2oScw = g_cur_can * g_can_w;

            mScw = Converter::toCvMat(mg2oScw);
            mpMatchedKF = pKF;

            ORBmatcher matcher(0.75,true);
            vector<MapPoint*> vvpMapPointMatches;
            int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches);
    
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches,true);
            pSolver->SetRansacParameters(0.99,20,300);
            vector<bool> vbInliers;
            int nIn;
            bool bNoMore;
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nIn);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                continue;
            }
            // cout<<"bNoMore = 0"<<endl;
            vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches.size(), static_cast<MapPoint*>(NULL));
            for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
            {
                if(vbInliers[j])
                    vpMapPointMatches[j]=vvpMapPointMatches[j];
            }

            cv::Mat R = R_cur_m*R_m_can ;
            cv::Mat t = R_cur_m*tvec_m_can+tvec_cur_m;
            matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,1,R,t,7.5);
            const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, g_cur_can, 10, true);
            // cout<<"nInliers>=15 ? = " <<(nInliers>=15)<<endl;
            if(nInliers>=15)
            {
                mpMatchedKF = pKF;
                mScw = Converter::toCvMat(mg2oScw);
                mvpCurrentMatchedPoints = vpMapPointMatches;
            }
            else {
                continue;
            }
            // Retrieve MapPoints seen in Loop Keyframe and neighbors
            vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
            vpLoopConnectedKFs.push_back(mpMatchedKF);
            mvpLoopMapPoints.clear();
            for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
            {
                KeyFrame* pKF = *vit;
                vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
                for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
                {
                    MapPoint* pMP = vpMapPoints[i];
                    if(pMP)
                    {
                        if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                        {
                            mvpLoopMapPoints.push_back(pMP);
                            pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                        }
                    }
                }
            }
            // cout<<"SearchByProjection in computeSim3"<<endl;
            matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

            // If enough matches accept Loop
            int nTotalMatches = 0;
            for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
            {
                if(mvpCurrentMatchedPoints[i])
                    nTotalMatches++;
            }
            // cout<<"nTotalMatches>=30 ? = " <<(nTotalMatches>=30)<<endl;
            if(nTotalMatches>=30)
            {
                // cout<<"computesim3 SUCCESS! "<<endl;
                return true;
            }
            else
            {
                continue;
            }
        }
    }

    // cout<<"computesim3 FAILED: nTotalMatches < 30"<<endl;
    mpCurrentKF->SetErase();
    return false;
}

bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                // If optimization is succesful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=40)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentKF->SetErase();
        return false;
    }

}

void LoopClosing::CorrectLoopByAruco()
{
    cout << "Loop detected!" << endl;
    mpLocalMapper->RequestStop();
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    mpCurrentKF->UpdateConnections();

    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();
    // cout<<"before get map mutex"<<endl;
    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        // cout<<"get map mutex"<<endl;

        // 保存要校正的帧：即将修改的T，和原有的T
        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;
            // cout<<"pKFi"<<endl;
            if(pKFi->isBad()){
                // cout<<"pKFi->isBad()"<<endl;
                continue;
            }

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }
        // cout<<"save KF poses"<<endl;
        set<int> sAId;
        set<int> sModified;
        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            if(pKFi->isBad()){
                cout<<"pKFi->isBad()"<<endl;
                continue;
            }
            for(size_t i=0; i<pKFi->NA; i++)
            {
                pKFi->SetIdxArucoOldFalse(i);
            }
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            sAId.clear();
            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw)); // 从漂移后的位置 =转到=> 正确的在world下的位置

                if(pMPi->forflag) { //保存在Aruco内地图点对应的Aruco ID号
                    sAId.insert(pMPi->mArucoID);
                }

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            // double s = g2oCorrectedSiw.scale();

            // eigt *=(1./s); //[R t/s;0 1] //! 此处的s应该就是 =1

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            cout<<"pKFi->SetPose(correctedTiw);"<<endl;
            pKFi->SetPose(correctedTiw);
            
            //! 此处应该要修正Aruco的位置
            cout<<"sAId.size() = "<<sAId.size()<<endl;
            for(set<int>::iterator sit=sAId.begin(); sit!=sAId.end(); sit++)
            {
                // cout<<"修正Aruco的位置"<<endl;
                int aid = *sit;
                set<int>::iterator sfind = sModified.find(aid);
                if(sfind!=sModified.end())
                    continue;
                cout<<"idx = pKFi->mmArucoIdandIdx[aid]"<<endl;
                bool hasAruco = pKFi->mmArucoIdandIdx.count(aid);
                if(!hasAruco)
                    continue;
                int idx = pKFi->mmArucoIdandIdx[aid];
                cout<<"pMA = pKFi->GetMapAruco(idx)"<<endl;
                MapAruco* pMA = pKFi->GetMapAruco(idx);
                cout<<"pMA is ok?"<<endl;
                if(pMA)
                {
                    if( !pMA->isBad()){
                        pMA->UpdateTcmByKF(pKFi, idx);
                        pMA->SetRtwmByKeyFrame(pKFi->GetRotation().t(), pKFi->GetCameraCenter() );
                        sModified.insert(aid);
                    }
                }
                
            }
            // vector<MapAruco*> vpMAsi = pKFi->GetAllMapArucos();
            // for(size_t iMA =0, endMAi=vpMAsi.size(); iMA<endMAi; iMA++)
            // {
            //     if(pKFi->mvbArucoGood[iMA]==false)
            //         continue;
            //     MapAruco* pMAi = vpMAsi[iMA];
            //     if(pMAi->mnCorrectedByKF==mpCurrentKF->mnId)
            //         continue;
                
            //     // cv::Mat TMAw = pMAi->GetTwm();
            //     // 真Twi <= Ti <= Tw
            //     pMAi->UpdateTcmByKF(pKFi, iMA);
            //     pMAi->SetRtwmByKeyFrame(pKFi->GetRotation().t(), pKFi->GetCameraCenter() ); //真Twi
            //     pMAi->mnCorrectedByKF = mpCurrentKF->mnId;
            // }

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }
        cout<<"correct KPs and ARUCOs"<<endl;
        // mpMap->UpdateAruco();

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)  //!!!!!! in computesim3()
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }

    }
    cout << "before SearchAndFuse(CorrectedSim3)"<<endl;
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    cout<<"before OptimizeEssentialGraph"<<endl;
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, true);

    mpMap->InformNewBigChange();

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    mLastLoopKFid = mpCurrentKF->mnId;   
    cout<<"end of correctloopbyaruco"<<endl;
}

void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();


    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }

    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    mpMap->InformNewBigChange();

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    mLastLoopKFid = mpCurrentKF->mnId;   
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);
            }
        }
    }
}


void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;

    int idx =  mnFullBAIdx;
    Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }            

            mpMap->InformNewBigChange();

            mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
