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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints(std::map<int, vector<cv::Point3f>> mmAruMPs)
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();
    

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    // glPointSize(mPointSize);
    // glBegin(GL_POINTS);
    // glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        // if(vpMPs[i]->forflag == 1) {
        //     glPointSize(mPointSize*2);
        //     glBegin(GL_POINTS);
        //     glColor3f(0.0, 1.0, 0.0);
        //     glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        //     glEnd();
        // }
        // else{
            glPointSize(mPointSize);
            glBegin(GL_POINTS);
            glColor3f(0.0,0.0,0.0);
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
            glEnd();
        // }
        // glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    // glEnd();

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        // if((*sit)->forflag == 1) {
        //     glPointSize(mPointSize*2);
        //     glBegin(GL_POINTS);
        //     glColor3f(0.0, 1.0, 0.0);
        //     glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        //     glEnd();
        // } else {
            glPointSize(mPointSize);
            glBegin(GL_POINTS);
            glColor3f(1.0,0.0,0.0);
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
            glEnd();
        // }
        
    }
    
    int n = mmAruMPs.size();
    for(map<int, vector<cv::Point3f>>::iterator it=mmAruMPs.begin(); it!=mmAruMPs.end(); it++ )
    {
        vector<cv::Point3f> vpAruco = it->second;
        glLineWidth(mKeyFrameLineWidth*1.3);
        glColor4f(0.0f,0.0f,0.9f, 0.5f);
        glBegin(GL_POLYGON);
        glVertex3f(vpAruco[0].x,vpAruco[0].y,vpAruco[0].z);
        glVertex3f(vpAruco[1].x,vpAruco[1].y,vpAruco[1].z);
        glVertex3f(vpAruco[2].x,vpAruco[2].y,vpAruco[2].z);
        glVertex3f(vpAruco[3].x,vpAruco[3].y,vpAruco[3].z);
        glEnd();
    }
}

void MapDrawer::DrawAruco()
{
    const vector<MapAruco*> &vpMAs = mpMap->GetAllMapArucos();
    // cout<<"Draw Aruco"<<endl;
    for(size_t i=0; i<vpMAs.size(); i++)
    {
        if(vpMAs[i])
        {
            int plId = vpMAs[i]->GetPlane()->GetID();
            
            cv::Vec3b v3 = GetColor(plId); 
            glLineWidth(mKeyFrameLineWidth);
            float a =(float)v3(0)/255;
            float b =(float)v3(1)/255;
            float c =(float)v3(2)/255;

            glColor3f(a,b,c);
            glBegin(GL_POLYGON);

            cv::Mat m0 = vpMAs[i]->GetPosInWorld(0);
            cv::Mat m1 = vpMAs[i]->GetPosInWorld(1);
            cv::Mat m2 = vpMAs[i]->GetPosInWorld(2);
            cv::Mat m3 = vpMAs[i]->GetPosInWorld(3);

            glVertex3f(m0.at<float>(0),m0.at<float>(1),m0.at<float>(2));
            glVertex3f(m1.at<float>(0),m1.at<float>(1),m1.at<float>(2));
            glVertex3f(m2.at<float>(0),m2.at<float>(1),m2.at<float>(2));
            glVertex3f(m3.at<float>(0),m3.at<float>(1),m3.at<float>(2));
        

            glEnd();

            glColor3f(0.0f, 0.5f, 0.0f);
            glBegin(GL_LINES);
            glVertex3f(m0.at<float>(0),m0.at<float>(1),m0.at<float>(2));
            glVertex3f(m1.at<float>(0),m1.at<float>(1),m1.at<float>(2));

            glVertex3f(m1.at<float>(0),m1.at<float>(1),m1.at<float>(2));
            glVertex3f(m2.at<float>(0),m2.at<float>(1),m2.at<float>(2));

            glVertex3f(m2.at<float>(0),m2.at<float>(1),m2.at<float>(2));
            glVertex3f(m3.at<float>(0),m3.at<float>(1),m3.at<float>(2));
        
            glVertex3f(m3.at<float>(0),m3.at<float>(1),m3.at<float>(2));
            glVertex3f(m0.at<float>(0),m0.at<float>(1),m0.at<float>(2));
            glEnd();
        }
        
    }
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
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
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
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


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

cv::Vec3b MapDrawer::GetColor(int i)
{
    int tmp = i%49;
    switch (tmp)
    {
    case 0:
        return cv::Vec3b(0,255,255);
    case 1:
        return cv::Vec3b(0,0,255);  // red
    case 2:
        return cv::Vec3b(255,0,0);  // blue
    case 3:
        return cv::Vec3b(255,255,0); // cyan
    case 4:
        return cv::Vec3b(47,255,173); // green yellow
    case 5:
        return cv::Vec3b(128, 0, 128);
    case 6:
        return cv::Vec3b(203,192,255);
    case 7:
        return cv::Vec3b(196,228,255);
    case 8:
        return cv::Vec3b(42,42,165);
    // case 9:
    //     return cv::Vec3b(255,255,255);
    // case 10:
    //     return cv::Vec3b(245,245,245); // whitesmoke
    case 11:
        return cv::Vec3b(0,165,255); // orange
    case 12:
        return cv::Vec3b(230,216,173); // lightblue   
    case 13:
        return cv::Vec3b(128,128,128); // grey  
    case 14:
        return cv::Vec3b(0,215,255); // gold 
    case 15:
        return cv::Vec3b(30,105,210); // chocolate
    case 16:
        return cv::Vec3b(0,255,0);  // green
    case 17:
        return cv::Vec3b(34, 34, 178);  // firebrick
    case 18:
        return cv::Vec3b(240, 255, 240);  // honeydew
    case 19:
        return cv::Vec3b(250, 206, 135);  // lightskyblue
    case 20:
        return cv::Vec3b(238, 104, 123);  // mediumslateblue
    case 21:
        return cv::Vec3b(225, 228, 255);  // mistyrose
    case 22:
        return cv::Vec3b(128, 0, 0);  // navy
    case 23:
        return cv::Vec3b(35, 142, 107);  // olivedrab
    case 24:
        return cv::Vec3b(45, 82, 160);  // sienna
    case 25:
        return cv::Vec3b(0, 255, 127); // chartreuse
    case 26:
        return cv::Vec3b(139, 0, 0);  // darkblue
    case 27:
        return cv::Vec3b(60, 20, 220);  // crimson
    case 28:
        return cv::Vec3b(0, 0, 139);  // darkred
    case 29:
        return cv::Vec3b(211, 0, 148);  // darkviolet
    case 30:
        return cv::Vec3b(255, 144, 30);  // dodgerblue
    case 31:
        return cv::Vec3b(105, 105, 105);  // dimgray
    case 32:
        return cv::Vec3b(180, 105, 255);  // hotpink
    case 33:
        return cv::Vec3b(204, 209, 72);  // mediumturquoise
    case 34:
        return cv::Vec3b(173, 222, 255);  // navajowhite
    case 35:
        return cv::Vec3b(143, 143, 188); // rosybrown
    case 36:
        return cv::Vec3b(50, 205, 50);  // limegreen
    case 37:
        return cv::Vec3b(34, 34, 178);  // firebrick
    case 38:
        return cv::Vec3b(240, 255, 240);  // honeydew
    case 39:
        return cv::Vec3b(250, 206, 135);  // lightskyblue
    case 40:
        return cv::Vec3b(238, 104, 123);  // mediumslateblue
    case 41:
        return cv::Vec3b(225, 228, 255);  // mistyrose
    case 42:
        return cv::Vec3b(128, 0, 0);  // navy
    case 43:
        return cv::Vec3b(35, 142, 107);  // olivedrab
    case 44:
        return cv::Vec3b(45, 82, 160);  // sienna
    case 45:
        return cv::Vec3b(30,105,210); // chocolate
    case 46:
        return cv::Vec3b(0,255,0);  // green
    case 47:
        return cv::Vec3b(34, 34, 178);  // firebrick
    case 48:
        return cv::Vec3b(240, 255, 240);  // honeydew
    
    case 9:
        return cv::Vec3b(250, 206, 135);  // lightskyblue
    
    case 10:
        return cv::Vec3b(238, 104, 123);  // mediumslateblue    
    }
}

} //namespace ORB_SLAM
