/**
 * Add by liujiamin
 * TODO: to manage Planes
 */

#ifndef MAPPLANE_H
#define MAPPLANE_H

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Thirdparty/aruco/aruco/aruco.h"
#include "MapAruco.h"
#include <opencv2/core/core.hpp>
#include <mutex>
#include <set>
#include <Eigen/Core>

namespace ORB_SLAM2
{

class Frame;
class KeyFrame;
class Map;
class MapPoint;
class MapAruco;

class MapPlane
{
public:
    MapPlane(const cv::Mat& Twm, Map* pMap );

    void AddMapAruco(MapAruco* pMA);

    Eigen::Vector3d GetNormal();
    Eigen::Vector3d GetOnePoint();
    double          GetDis();
    int             GetID();
    Eigen::Vector4d GetCoeffs();
    void            SetCoeffs(const Eigen::Vector4d &coe);
    void            SetCoeffsByAruco(const cv::Mat& Twm);
    int             GetMapArucoNums();
    vector<MapAruco*> GetMapAruco();
    double          GetMapArucoDis(const size_t& idx);

    void            SetParPlanes(MapPlane* pMPL);
    void            SetVerPlanes(MapPlane* pMPL);
    void            SetParArucos(MapAruco* pMA);
    void            SetVerArucos(MapAruco* pMA);
    vector<MapPlane*> GetParPlanes();
    
    bool  isBad();
    void  SetBadFlag();

protected:
    Eigen::Vector4d mCoeffs;
    Eigen::Vector3d mNormal;
    double mDis;
    Eigen::Vector3d mPoint;

    static int mNextID;
    int mID;

    Map* mpMap;
    bool mbBad;

    set<MapPlane*> mspParPlanes;
    set<MapPlane*> mspVerPlanes;
    set<MapAruco*> mspParArucos;
    set<MapAruco*> mspVerArucos;

    std::set<MapAruco*> mspMapArucos;

    std::mutex mMutexFeatures;

};


}


#endif //MAPPLANE_H