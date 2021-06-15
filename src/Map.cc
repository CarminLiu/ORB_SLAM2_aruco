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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

// Add by liujiamin
void Map::AddMapAruco(MapAruco* pMA)
{
    unique_lock<mutex> lock(mMutexMap);
    // cout<<"Now is add MapAruco"<<endl;
    mspMapArucos.insert(pMA);
    msMAid.insert(pMA->GetMapArucoID());
    mmIdAndAruco[pMA->GetMapArucoID()] = pMA;
}

void Map::AddMapPlane(MapPlane* pMPL)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPlanes.insert(pMPL);
}

std::vector<int> Map::GetAllMapArucoID()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<int>(msMAid.begin(), msMAid.end());
}

bool Map::HasMapAruco(const int &idx)
{
    unique_lock<mutex> lock(mMutexMap);
    bool has = mmIdAndAruco.count(idx);
    return has;
}

MapAruco* Map::GetMapAruco(const int &idx)
{
    unique_lock<mutex> lock(mMutexMap);
    // set<int>::iterator sit = msMAid.find(idx);
    // if(sit!=)
    return mmIdAndAruco[idx];
}

// void Map::AddArucoMapPoint(MapPoint* pAMP)
// {
//     unique_lock<mutex> lock(mMutexMap);
//     mspArucoMapPoints.insert(pAMP);
// }

// void Map::AddArucoMapPointsID(int id)
// {
//     unique_lock<mutex> lock(mMutexMap);
//     msAMPsid.insert(id);
// }

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseMapAruco(MapAruco *pMA)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapArucos.erase(pMA);
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

std::vector<MapAruco*> Map::GetAllMapArucos()
{
    unique_lock<mutex> lock(mMutexMap);
    // cout<<"in Map::GetAllMapArucos()"<<mspMapArucos.size()<<endl;
    return vector<MapAruco*>(mspMapArucos.begin(),mspMapArucos.end());
}

vector<MapPlane*> Map::GetAllMapPlanes()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPlane*>(mspMapPlanes.begin(),mspMapPlanes.end());
}

// std::vector<int> Map::GetAllArucoMapPointID()
// {
//     unique_lock<mutex> lock(mMutexMap);
//     return vector<int>(msAMPsid.begin(), msAMPsid.end());
// }

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    //! 应该包括MapAruco的删除
    for(set<MapAruco*>::iterator sit=mspMapArucos.begin(), send=mspMapArucos.end(); sit!=send; sit++)
        delete *sit;

    for(set<MapPlane*>::iterator sit=mspMapPlanes.begin(), send=mspMapPlanes.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mspMapArucos.clear();
    mspMapPlanes.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::UpdateAruco()
{
    unique_lock<mutex> lock(mMutexMap);
    // vector<int> vKFid;
    // vector<KeyFrame*> vkf = this->GetAllKeyFrames();
    // long unsigned int numkf = vkf.size();
    // for(size_t i=0; i<numkf; i++)
    // {
    //     KeyFrame* pk = vkf[i];
    //     int id = pk->mnId;
    //     vKFid.push_back(id);
    // }
    for(set<MapAruco*>::iterator sit=mspMapArucos.begin(), send=mspMapArucos.end(); sit!=send; sit++)
    {
        MapAruco* pA = *sit;
        cout<<"pa->id = "<<pA->GetMapArucoID();
        map<KeyFrame*, size_t> mks = pA->GetObservations();
        for(map<KeyFrame*, size_t>::iterator mit=mks.begin(),mend=mks.end(); mit!=mend; mit++) {
            KeyFrame* pKF=mit->first;
            if(!pKF->isBad())
            {
                size_t idx=mit->second;
                pA->UpdateTcmByKF(pKF, idx);
                pA->SetRtwmByKeyFrame(pKF->GetRotation().t(), pKF->GetCameraCenter());
                cout<<"; update by kf: "<<pKF->mnId;
                // break;
            }
        }
        cout<<endl;
        // long unsigned int refkfid = pA->GetFirstKFid();
        // for(long unsigned int i=0; i<numkf; i++) {
        //     if(vKFid[i] == refkfid) {
        //         pA->SetRtwmByKeyFrame(vkf[i]->GetRotation().t(), vkf[i]->GetCameraCenter());
                
        //         break;
        //     }
        // }
    }
    // cout<<"Update Aruco"<<endl;
}

void Map::Save(const string &filename)
{
    cerr << "Map saving to:" << filename <<endl;
    ofstream f;
    f.open(filename.c_str(), ios_base::out|ios::binary);

    unsigned long int nMapPoints =mspMapPoints.size();
    f.write((char*)&nMapPoints, sizeof(nMapPoints));
    //Save MapPoint sequentially
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++) {
        MapPoint* pMP = *sit;
        SaveMapPoint(f, pMP);
    }
    cerr << "The number of MapPoints is :"<<mspMapPoints.size()<<endl;

    //Grab the index of each MapPoint, count from 0, in which we initialized mmpnMapPointsIdx 
    GetMapPointsIdx();

    unsigned long int nKeyFrames = mspKeyFrames.size();
    f.write((char*)&nKeyFrames, sizeof(nKeyFrames));
    for (auto kf: mspKeyFrames)
        SaveKeyFrame(f, kf);
    cerr << "The number of KeyFrames:"<<mspKeyFrames.size()<<endl;

    for (auto kf:mspKeyFrames)
    {
        //Get parent of current KeyFrame and save the ID of this parent
        KeyFrame* parent = kf->GetParent();
        unsigned long int parent_id = ULONG_MAX;
        if ( parent )
            parent_id = parent->mnId;
        f.write((char*)&parent_id, sizeof(parent_id));

        //Get the size of the Connected KeyFrames of the current KeyFrames
        //and then save the ID and weight of the Connected KeyFrames
        unsigned long int nb_con = kf->GetConnectedKeyFrames().size();
        f.write((char*)&nb_con, sizeof(nb_con));
        for ( auto ckf: kf->GetConnectedKeyFrames())
        {
            int weight = kf->GetWeight(ckf);
            f.write((char*)&ckf->mnId, sizeof(ckf->mnId));
            f.write((char*)&weight, sizeof(weight));
        }
    }
    f.close();
    cerr<<"Map.cc :: Map Saving Finished!"<<endl;
}

void Map::SaveMapPoint(ofstream &f, MapPoint* mp)
{
    //Save ID and the x,y,z coordinates of the current MapPoint
    f.write((char*)&mp->mnId, sizeof(mp->mnId));
    cv::Mat mpWorldPos = mp->GetWorldPos();
    f.write((char*)& mpWorldPos.at<float>(0),sizeof(float));
    f.write((char*)& mpWorldPos.at<float>(1),sizeof(float));
    f.write((char*)& mpWorldPos.at<float>(2),sizeof(float));
}

void Map::SaveKeyFrame(ofstream &f, KeyFrame* kf)
{
    //Save the ID and timesteps of current KeyFrame
    f.write((char*)&kf->mnId, sizeof(kf->mnId));
    // cout << "saving kf->mnId = " << kf->mnId <<endl;
    f.write((char*)&kf->mTimeStamp, sizeof(kf->mTimeStamp));
    //Save the Pose Matrix of current KeyFrame
    cv::Mat Tcw = kf->GetPose();

     //Save the rotation matrix in Quaternion
    std::vector<float> Quat = Converter::toQuaternion(Tcw);
    for ( int i = 0; i < 4; i ++ )
        f.write((char*)&Quat[i],sizeof(float));
    //Save the translation matrix
    for ( int i = 0; i < 3; i ++ )
        f.write((char*)&Tcw.at<float>(i,3),sizeof(float));

     //Save the size of the ORB features current KeyFrame
    //cerr<<"kf->N:"<<kf->N<<endl;
    f.write((char*)&kf->N, sizeof(kf->N));
    //Save each ORB features
    for( int i = 0; i < kf->N; i ++ )
    {
        cv::KeyPoint kp = kf->mvKeys[i];
        f.write((char*)&kp.pt.x, sizeof(kp.pt.x));
        f.write((char*)&kp.pt.y, sizeof(kp.pt.y));
        f.write((char*)&kp.size, sizeof(kp.size));
        f.write((char*)&kp.angle,sizeof(kp.angle));
        f.write((char*)&kp.response, sizeof(kp.response));
        f.write((char*)&kp.octave, sizeof(kp.octave));

        //Save the Descriptors of current ORB features
        f.write((char*)&kf->mDescriptors.cols, sizeof(kf->mDescriptors.cols)); //kf->mDescriptors.cols is always 32 here.
        for (int j = 0; j < kf->mDescriptors.cols; j ++ )
            f.write((char*)&kf->mDescriptors.at<unsigned char>(i,j), sizeof(char));

        //Save the index of MapPoints that corresponds to current ORB features
        unsigned long int mnIdx;
        MapPoint* mp = kf->GetMapPoint(i);
        if (mp == NULL  )
            mnIdx = ULONG_MAX;
        else
            mnIdx = mmpnMapPointsIdx[mp];

        f.write((char*)&mnIdx, sizeof(mnIdx));
    }

    // Save BoW for relocalization.
    // f.write((char*)&kf->mBowVec, sizeof(kf->mBowVec));
}

void Map::GetMapPointsIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    unsigned long int i = 0;
    for(auto mp: mspMapPoints)
    {
        mmpnMapPointsIdx[mp] = i;
        i += 1;
    }
}

void Map::Load(const string &filename, SystemSetting* mySystemSetting)
{
    cerr << "Map reading from:"<<filename<<endl;
    ifstream f;
    f.open( filename.c_str() );

    // Same as the sequence that we save the file, we first read the number of MapPoints.
    unsigned long int nMapPoints;
    f.read((char*)&nMapPoints, sizeof(nMapPoints));

    // Then read MapPoints one after another, and add them into the map
    cerr<<"The number of MapPoints:"<<nMapPoints<<endl;
    for ( unsigned int i = 0; i < nMapPoints; i ++ )
    {
        MapPoint* mp = LoadMapPoint(f);
        AddMapPoint(mp);
    }

    // Get all MapPoints
    std::vector<MapPoint*> vmp = GetAllMapPoints();

    // Read the number of KeyFrames
    unsigned long int nKeyFrames;
    f.read((char*)&nKeyFrames, sizeof(nKeyFrames));
    cerr<<"The number of KeyFrames:"<<nKeyFrames<<endl;

    // Then read KeyFrames one after another, and add them into the map
    vector<KeyFrame*>kf_by_order;
    for( unsigned int i = 0; i < nKeyFrames; i ++ )
    {
        KeyFrame* kf = LoadKeyFrame(f, mySystemSetting);
        AddKeyFrame(kf);
        kf_by_order.push_back(kf);
    }

    cerr<<"Max KeyFrame ID is: " << mnMaxKFid << ", and I set mnId to this number" <<endl;
    
    cerr<<"KeyFrame Load OVER!"<<endl;

    // Read Spanning Tree(open loop trajectory)
    map<unsigned long int, KeyFrame*> kf_by_id;
    for ( auto kf: mspKeyFrames )
        kf_by_id[kf->mnId] = kf;
    cerr<<"Start Load The Parent!"<<endl;
    for( auto kf: kf_by_order )
    {
        // Read parent_id of current KeyFrame.
        unsigned long int parent_id;
        f.read((char*)&parent_id, sizeof(parent_id));

        // Add parent KeyFrame to current KeyFrame.
        // cout<<"Map::Load : Add parent KeyFrame to current KeyFrame"<<endl;
        if ( parent_id != ULONG_MAX )
            kf->ChangeParent(kf_by_id[parent_id]);

        // Read covisibility graphs.
        // Read the number of Connected KeyFrames of current KeyFrame.
        unsigned long int nb_con;
        f.read((char*)&nb_con, sizeof(nb_con));
        // Read id and weight of Connected KeyFrames of current KeyFrame, 
        // and add Connected KeyFrames into covisibility graph.
        // cout<<"Map::Load : Read id and weight of Connected KeyFrames"<<endl;
        for ( unsigned long int i = 0; i < nb_con; i ++ )
        {
            unsigned long int id;
            int weight;
            f.read((char*)&id, sizeof(id));
            f.read((char*)&weight, sizeof(weight));
            kf->AddConnection(kf_by_id[id],weight);
        }
   }
   cerr<<"Parent Load OVER!"<<endl;
   for ( auto mp: vmp )
   {
       // cout << "Now mp = "<< mp << endl;
       if(mp)
       {
            // cout << "compute for mp = "<< mp << endl;
            mp->ComputeDistinctiveDescriptors();
            // cout << "Computed Distinctive Descriptors." << endl;
            mp->UpdateNormalAndDepth();
            // cout << "Updated Normal And Depth." << endl;
        }
   }
    f.close();
    cerr<<"Load IS OVER!"<<endl;
    return;
}

MapPoint* Map::LoadMapPoint( ifstream &f )
{
    // Position and Orientation of the MapPoints.
    cv::Mat Position(3,1,CV_32F);
    long unsigned int id;
    f.read((char*)&id, sizeof(id));

    f.read((char*)&Position.at<float>(0), sizeof(float));
    f.read((char*)&Position.at<float>(1), sizeof(float));
    f.read((char*)&Position.at<float>(2), sizeof(float));

    // Initialize a MapPoint, and set its id and Position.
    MapPoint* mp = new MapPoint(Position, this );
    mp->mnId = id;
    mp->SetWorldPos( Position );

    return mp;
}

KeyFrame* Map::LoadKeyFrame( ifstream &f, SystemSetting* mySystemSetting )
{
    InitKeyFrame initkf(*mySystemSetting);

    // Read ID and TimeStamp of each KeyFrame.
    f.read((char*)&initkf.nId, sizeof(initkf.nId));
    f.read((char*)&initkf.TimeStamp, sizeof(double));

    // Read position and quaternion
    cv::Mat T = cv::Mat::zeros(4,4,CV_32F);
    std::vector<float> Quat(4);
    //Quat.reserve(4);
    for ( int i = 0; i < 4; i ++ )
        f.read((char*)&Quat[i],sizeof(float));
    cv::Mat R = Converter::toCvMat(Quat);
    for ( int i = 0; i < 3; i ++ )
        f.read((char*)&T.at<float>(i,3),sizeof(float));
    for ( int i = 0; i < 3; i ++ )
        for ( int j = 0; j < 3; j ++ )
            T.at<float>(i,j) = R.at<float>(i,j);
    T.at<float>(3,3) = 1;
    
    // Read feature point number of current Key Frame
    f.read((char*)&initkf.N, sizeof(initkf.N));
    initkf.vKps.reserve(initkf.N);
    initkf.Descriptors.create(initkf.N, 32, CV_8UC1);
    vector<float>KeypointDepth;

    std::vector<MapPoint*> vpMapPoints;
    vpMapPoints = vector<MapPoint*>(initkf.N,static_cast<MapPoint*>(NULL));
    // Read Keypoints and descriptors of current KeyFrame
    std::vector<MapPoint*> vmp = GetAllMapPoints();
    for(int i = 0; i < initkf.N; i ++ )
    {
        cv::KeyPoint kp;
        f.read((char*)&kp.pt.x, sizeof(kp.pt.x));
        f.read((char*)&kp.pt.y, sizeof(kp.pt.y));
        f.read((char*)&kp.size, sizeof(kp.size));
        f.read((char*)&kp.angle,sizeof(kp.angle));
        f.read((char*)&kp.response, sizeof(kp.response));
        f.read((char*)&kp.octave, sizeof(kp.octave));

        initkf.vKps.push_back(kp);
        
        // Read descriptors of keypoints
        f.read((char*)&initkf.Descriptors.cols, sizeof(initkf.Descriptors.cols));
        // for ( int j = 0; j < 32; j ++ ) // Since initkf.Descriptors.cols is always 32, for loop may also write like this.
        for ( int j = 0; j < initkf.Descriptors.cols; j ++ )
            f.read((char*)&initkf.Descriptors.at<unsigned char>(i,j),sizeof(char));

        // Read the mapping from keypoints to MapPoints.
        unsigned long int mpidx;
        f.read((char*)&mpidx, sizeof(mpidx));

        // Look up from vmp, which contains all MapPoints, MapPoint of current KeyFrame, and then insert in vpMapPoints.
        if( mpidx == ULONG_MAX )
                vpMapPoints[i] = NULL;
        else
                vpMapPoints[i] = vmp[mpidx];
    }

  
    initkf.vRight = vector<float>(initkf.N,-1);
    initkf.vDepth = vector<float>(initkf.N,-1);
    //initkf.vDepth = KeypointDepth;
    initkf.UndistortKeyPoints();
    initkf.AssignFeaturesToGrid();

    // Use initkf to initialize a KeyFrame and set parameters
    KeyFrame* kf = new KeyFrame( initkf, this, NULL, vpMapPoints );
    kf->mnId = initkf.nId;
    kf->SetPose(T);
    kf->ComputeBoW();

    for ( int i = 0; i < initkf.N; i ++ )
    {
        if ( vpMapPoints[i] )
        {
            vpMapPoints[i]->AddObservation(kf,i);
            if( !vpMapPoints[i]->GetReferenceKeyFrame())
                vpMapPoints[i]->SetReferenceKeyFrame(kf);
        }
    }
    return kf;
}

} //namespace ORB_SLAM
