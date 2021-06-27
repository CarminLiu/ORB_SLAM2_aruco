/**
 * Add by liujiamin
 * TODO: to manage Planes
 */

#include "MapPlane.h"

namespace ORB_SLAM2
{
int MapPlane::mNextID=0;

MapPlane::MapPlane(const cv::Mat& Twm, Map* pMap ):mpMap(pMap), mbBad(false)
{
    cv::Mat normal = Twm.rowRange(0,3).col(2);
    cv::Mat center = Twm.rowRange(0,3).col(3);
    mNormal=Eigen::Vector3d(normal.at<float>(0,0), normal.at<float>(1,0), normal.at<float>(2,0));
    mPoint=Eigen::Vector3d(center.at<float>(0,0), center.at<float>(1,0), center.at<float>(2,0));
    mDis = mNormal.dot(mPoint);

    mCoeffs = Eigen::Vector4d(mNormal[0], mNormal[1], mNormal[2], -mDis);

    mID = mNextID++;
}

void MapPlane::AddMapAruco(MapAruco* pMA)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mspMapArucos.insert(pMA);
}

void MapPlane::SetCoeffs(const Eigen::Vector4d &coe)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mCoeffs = coe;
    mNormal[0] = coe[0];
    mNormal[1] = coe[1];
    mNormal[2] = coe[2];
    mDis = -coe[3];
    mPoint[2] = -(mPoint[0]*coe[0]+mPoint[1]*coe[1]+coe[3])/coe[2]; //这样好像稍微有点问题
}

void MapPlane::SetCoeffsByAruco(const cv::Mat& Twm)
{
    cv::Mat normal = Twm.rowRange(0,3).col(2);
    cv::Mat center = Twm.rowRange(0,3).col(3);
    mNormal=Eigen::Vector3d(normal.at<float>(0,0), normal.at<float>(1,0), normal.at<float>(2,0));
    mPoint=Eigen::Vector3d(center.at<float>(0,0), center.at<float>(1,0), center.at<float>(2,0));
    mDis = mNormal.dot(mPoint);
    mCoeffs = Eigen::Vector4d(mNormal[0], mNormal[1], mNormal[2], -mDis);
}

Eigen::Vector4d MapPlane::GetCoeffs()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mCoeffs;
}

Eigen::Vector3d MapPlane::GetNormal()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mNormal;
}

double MapPlane::GetDis()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDis;
}

int MapPlane::GetID()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mID;
}

Eigen::Vector3d MapPlane::GetOnePoint()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mPoint;
}

int MapPlane::GetMapArucoNums()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mspMapArucos.size();
}

vector<MapAruco*> MapPlane::GetMapAruco()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return vector<MapAruco*>(mspMapArucos.begin(),mspMapArucos.end());
}

double MapPlane::GetMapArucoDis(const size_t& idx)
{
    vector<MapAruco*> vpMA(mspMapArucos.begin(),mspMapArucos.end());
    MapAruco* pMA = vpMA[idx];
    cv::Mat tw = pMA->Gettw2m();
    Eigen::Vector3d pointM(tw.at<float>(0,0),tw.at<float>(1,0),tw.at<float>(2,0));
    Eigen::Vector3d pP2M = pointM - mPoint;
    double d = abs(pP2M.dot(mNormal));
    return d;
}

void MapPlane::SetParPlanes(MapPlane* pMPL)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mspParPlanes.insert(pMPL);
}

void MapPlane::SetVerPlanes(MapPlane* pMPL)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mspVerPlanes.insert(pMPL);
}

void MapPlane::SetParArucos(MapAruco* pMA)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mspParArucos.insert(pMA);
}

void MapPlane::SetVerArucos(MapAruco* pMA)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mspVerArucos.insert(pMA);
}

vector<MapPlane*> MapPlane::GetParPlanes()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return vector<MapPlane*>(mspParPlanes.begin(),mspParPlanes.end());
}

bool MapPlane::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mbBad;
}

void MapPlane::SetBadFlag()
{
    
}

}