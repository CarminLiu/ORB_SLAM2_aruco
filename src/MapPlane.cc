/**
 * Add by liujiamin
 * TODO: to manage Planes
 */

#include "MapPlane.h"

namespace ORB_SLAM2
{
int MapPlane::mNextID=0;

MapPlane::MapPlane(const cv::Mat& Twm, Map* pMap ):mpMap(pMap)
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

Eigen::Vector3d MapPlane::GetOnePoint()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mPoint;
}

}