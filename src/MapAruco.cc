/**
 * Add by liujiamin
 * TODO: to manage Aruco's corner
 */

#include "MapAruco.h"

namespace ORB_SLAM2
{

mutex MapAruco::mGlobalMutex;

MapAruco::MapAruco(const aruco::Marker& pM, KeyFrame* pRefKF, Map* pMap, double length):
nObs(0), mbAddLocalBA(false), mnCorrectedByKF(0), 
mbBad(false), mpMap(pMap), nBadComputed(0), isWellComputed(false)
{
    // cout<<"It's in the constuction..."<<endl;
    mAruco = pM; // 此mAruco会保存他在当前帧上的像素位置
    mLength = length;
    cv::Mat rvec = mAruco.Rvec;
    mRcm = cv::Mat::zeros(3,3,CV_32F);
    cv::Rodrigues(rvec, mRcm);
    mtcm = mAruco.Tvec;
    isTwm = false;

    mTwm = cv::Mat::eye(4,4,CV_32F);

    // TODO: Initial the position in the Tag Reference
    // (-s/2,s/2,0), (s/2,s/2,0), (s/2,-s/2,0), (-s/2,-s/2,0)
    cv::Mat c0=(cv::Mat_<float>(3,1)<<-mLength/2, mLength/2, 0);
    cv::Mat c1=(cv::Mat_<float>(3,1)<< mLength/2, mLength/2, 0);
    cv::Mat c2=(cv::Mat_<float>(3,1)<< mLength/2,-mLength/2, 0);
    cv::Mat c3=(cv::Mat_<float>(3,1)<<-mLength/2,-mLength/2, 0);
    mvPosInTag.push_back(c0);
    mvPosInTag.push_back(c1);
    mvPosInTag.push_back(c2);
    mvPosInTag.push_back(c3);

    // TODO: get rotation and transform from tag
    // 当一个新的Aruco出现的时候，先保存它与当前帧之间的R、t
    // 等当前帧位姿优化之后，再附上与世界坐标之间的关系，即 Rwm=Rwc*Rcm, twm=twc+tcm
    // 这样好像在 mAruco 里面已经包含了 Rvec, Tvec. 

    // TODO: Record FirstKFid | first Frame id
    mnFirstKFid = pRefKF->mnId;
    mnFirstFrame = pRefKF->mnFrameId;

    mvMapPoint = vector<MapPoint*>(4,static_cast<MapPoint*>(NULL));
}

cv::Point3f MapAruco::get3DPointsLocalRefSystem(size_t i)
{
    if(i==0) return cv::Point3f( -mLength/2.,  mLength/2., 0 );
    if(i==1) return cv::Point3f(  mLength/2.,  mLength/2., 0 );
    if(i==2) return cv::Point3f(  mLength/2., -mLength/2., 0 );
    if(i==3) return cv::Point3f( -mLength/2., -mLength/2., 0 );
}

// @param ini 
void MapAruco::SetRtwm(const cv::Mat &Rwc, const cv::Mat &twc) // first time: Rwc, twc
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    // cout<<"It's in the SetRTwm..."<<endl;

    if(!isTwm)
    {
        mRwm = Rwc * mRcm;
        mtwm = Rwc*mtcm + twc;
        isTwm = true;
    }
    else // 不是第一次赋值了，直接改变 marker->world
    {
        mRwm = Rwc;
        mtwm = twc;
    }
    
    mRwm.copyTo( mTwm.rowRange(0,3).colRange(0,3) );
    mtwm.copyTo( mTwm.rowRange(0,3).col(3) );

    SetPosInWorld();
}

void MapAruco::SetRtwmByKeyFrame(const cv::Mat &Rwc, const cv::Mat &twc)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    mRwm = Rwc * mRcm;
    mtwm = Rwc * mtcm + twc;
    SetPosInWorld();
}

void MapAruco::UpdateTcmByKF(KeyFrame* pKF, size_t idx)
{
    mnCurKFidForTcm = pKF->mnId;
    cv::Mat rvec = pKF->mvMarkers[idx].Rvec;
    cv::Rodrigues(rvec, mRcm);
    mtcm = pKF->mvMarkers[idx].Tvec;
    // SetPosInWorld();

}

cv::Mat MapAruco::GetTwm()
{
    unique_lock<mutex> lock(mMutexPos);
    return mTwm;
}

cv::Mat MapAruco::GetRw2m()
{
    unique_lock<mutex> lock(mMutexPos);
    return mRwm;
}

cv::Mat MapAruco::Gettw2m()
{
    unique_lock<mutex> lock(mMutexPos);
    return mtwm;
}

void MapAruco::SetPosInWorld()
{
    // unique_lock<mutex> lock(mMutexFeatures);
    // cout<<"It's in the GetPosInWorld..."<<endl;
    mvPosInWorld.clear();
    mvPosInWorld.push_back(mRwm * mvPosInTag[0] + mtwm);
    mvPosInWorld.push_back(mRwm * mvPosInTag[1] + mtwm);
    mvPosInWorld.push_back(mRwm * mvPosInTag[2] + mtwm);
    mvPosInWorld.push_back(mRwm * mvPosInTag[3] + mtwm);

    // cout<<"m1-m0="<<cv::norm(mvPosInWorld[1]-mvPosInWorld[0])<<endl;
    // cout<<"m2-01="<<cv::norm(mvPosInWorld[2]-mvPosInWorld[1])<<endl;
    // cout<<"m3-m2="<<cv::norm(mvPosInWorld[3]-mvPosInWorld[2])<<endl;
    // cout<<"m0-m3="<<cv::norm(mvPosInWorld[0]-mvPosInWorld[3])<<endl;

}

cv::Mat MapAruco::GetPosInWorld(const size_t & idx)
{
    unique_lock<mutex> lock(mMutexPos);
    return mvPosInWorld[idx];
}

MapPoint* MapAruco::GetMapPoint(const size_t & idx)
{
    unique_lock<mutex> lock(mMutexPos);
    return mvMapPoint[idx];
}

// void MapAruco::SetPosInWorld(const size_t & idx, const cv::Mat & p)
// {
//     unique_lock<mutex> lock(mMutexFeatures);
//     mvPosInWorld[idx] = p;
// }

void MapAruco::AddObservation(KeyFrame* pKF,size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    // cout<<"It's in the AddObservation..."<<endl;
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;
    mObsKFid.insert(pKF->mnId); //添加观测到的KF->mnId信息
    
    // pKF->mvuRightAruco[0/1/2/3] 的值都大于0，而且使用的是双目

    nObs++;
}

std::vector<int> MapAruco::GetAllObsKFsId()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return vector<int>(mObsKFid.begin(), mObsKFid.end());
}

int MapAruco::GetMapArucoID()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mAruco.id;
}

aruco::Marker MapAruco::GetAruco()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mAruco;
}

int MapAruco::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

map<KeyFrame*, size_t> MapAruco::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

double MapAruco::GetArucoLength()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mLength;
}

void MapAruco::SetCorrelateMapPoint(size_t idx, MapPoint* pMP)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvMapPoint[idx] = pMP;
}

long unsigned int MapAruco::GetFirstFrameid()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mnFirstFrame;
}

cv::Mat MapAruco::GetAxleZInworld()
{
    cv::Mat axleZ=(cv::Mat_<float>(3,1)<<0, 0, 1);
    mAxleZ = mRwm * axleZ + mtwm;
    return mAxleZ;
}

void MapAruco::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            // int idx=mObservations[pKF];
            mObservations.erase(pKF);
            if(nObs<=1)
                bBad=true;
        }
    }
    if(bBad) SetBadFlag();
}

void MapAruco::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs=mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF=mit->first;
        pKF->EraseMapArucoMatch(mit->second); 
    }
    mpMap->EraseMapAruco(this); 
}

bool MapAruco::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

}