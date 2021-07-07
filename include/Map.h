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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "MapAruco.h"
#include <set>
#include "Converter.h"
#include "SystemSetting.h"
#include "InitKeyFrame.h"
#include "MapPlane.h"

#include <mutex>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class MapAruco;
class SystemSetting;
class InitKeyFrame;
class MapPlane;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    // add by liujiamin
    void AddMapAruco(MapAruco* pMA);
    std::vector<MapAruco*> GetAllMapArucos();
    void UpdateAruco();
    std::vector<int> GetAllMapArucoID();
    void EraseMapAruco(MapAruco* pMA);
    bool HasMapAruco(const int &idx);
    MapAruco* GetMapAruco(const int &idx);
    // void AddArucoMapPointsID(int id);
    // std::vector<int> GetAllArucoMapPointID();
    // void AddArucoMapPoint(MapPoint* pAMP);
    void AddMapPlane(MapPlane* pMPL);
    std::vector<MapPlane*> GetAllMapPlanes();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
    std::mutex mMutexArucoCreation;

    //* SAVE and LOAD map
    void Save(const string &filename);
    void Load(const string &filename, SystemSetting* mySystemSetting);
    
    std::map<int, double> mArucoMeanLength;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    //* add by liujiamin
    std::set<MapAruco*> mspMapArucos;
    std::set<int>       msMAid;
    unordered_map<int, MapAruco*> mmIdAndAruco;
    // std::set<int> msAMPsid;
    // std::set<MapPoint*> mspArucoMapPoints;
    std::set<MapPlane*> mspMapPlanes;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;

    //* 保存地图点和关键帧
    void SaveMapPoint(ofstream &f, MapPoint* mp);
    void SaveKeyFrame(ofstream &f, KeyFrame* kf);
    std::map<MapPoint*, unsigned long int> mmpnMapPointsIdx;
    void GetMapPointsIdx();
    MapPoint* LoadMapPoint(ifstream &f);
    KeyFrame* LoadKeyFrame(ifstream &f, SystemSetting* mySystemSetting);
};

} //namespace ORB_SLAM

#endif // MAP_H
