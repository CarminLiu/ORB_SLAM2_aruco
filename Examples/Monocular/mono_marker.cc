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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
// #include <Timer.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void createYMLfromOpencvCamera(string opencv_camerafile,string orb2configfile  ,cv::Size outputSize=cv::Size(-1,-1)){

    cv::FileStorage infile;
    infile.open(opencv_camerafile,cv::FileStorage::READ);
    if (!infile.isOpened())throw std::runtime_error("Could not open "+opencv_camerafile);
    cv::Mat cameraMatrix,distCoeff;
    cv::Mat cameraMatrix32,distCoeff32;
    infile["camera_matrix"]>>cameraMatrix;
    infile["distortion_coefficients"]>>distCoeff;

    int img_width,img_height;
    infile["image_width"]>>img_width;
    infile["image_height"]>>img_height;



    cameraMatrix.convertTo(cameraMatrix32, CV_32F);
    distCoeff.convertTo(distCoeff32, CV_32F);



    //change params if size modified
    if (outputSize.width>0){
        float scalex=float(outputSize.width)/float(img_width);
        float scaley=float(outputSize.height)/float(img_height);
        cout<<scalex<<" "<<scaley<<endl;
        cameraMatrix32.at<float>(0,0)*=scalex;
        cameraMatrix32.at<float>(0,2)*=scalex;
        cameraMatrix32.at<float>(1,1)*=scaley;
        cameraMatrix32.at<float>(1,2)*=scaley;
    }

    ofstream ofile(orb2configfile);
    if(!ofile)  throw std::runtime_error("Could not open "+orb2configfile);
    ofile<<"%YAML:1.0"<<endl;
    ofile<<"Camera.fx:"<<cameraMatrix32.at<float>(0,0)<<endl;
    ofile<<"Camera.fy:"<<cameraMatrix32.at<float>(1,1)<<endl;
    ofile<<"Camera.cx:"<<cameraMatrix32.at<float>(0,2)<<endl;
    ofile<<"Camera.cy:"<<cameraMatrix32.at<float>(1,2)<<endl;
    ofile<<"Camera.k1:"<<distCoeff32.ptr<float>(0)[0]<<endl;
    ofile<<"Camera.k2:"<<distCoeff32.ptr<float>(0)[1]<<endl;
    ofile<<"Camera.p1:"<<distCoeff32.ptr<float>(0)[2]<<endl;
    ofile<<"Camera.p2:"<<distCoeff32.ptr<float>(0)[3]<<endl;
    ofile<<"Camera.k3:"<<distCoeff32.ptr<float>(0)[4]<<endl;

    ofile<<"Camera.fps: 30.0"<<endl;
    ofile<<"Camera.RGB: 1"<<endl;
    ofile<<"ORBextractor.nFeatures: 2000"<<endl; //! 1000 2000 3000 4000
    ofile<<"ORBextractor.scaleFactor: 1.2"<<endl;
    ofile<<"ORBextractor.nLevels: 8"<<endl;
    ofile<<"ORBextractor.iniThFAST: 20"<<endl;
    ofile<<"ORBextractor.minThFAST: 7"<<endl;

    ofile<<"Viewer.KeyFrameSize: 0.05"<<endl;
    ofile<<"Viewer.KeyFrameLineWidth: 1"<<endl;
    ofile<<"Viewer.GraphLineWidth: 0.9"<<endl;
    ofile<<"Viewer.PointSize:2"<<endl;
    ofile<<"Viewer.CameraSize: 0.08"<<endl;
    ofile<<"Viewer.CameraLineWidth: 3"<<endl;
    ofile<<"Viewer.ViewpointX: 0"<<endl;
    ofile<<"Viewer.ViewpointY: -0.7"<<endl;
    ofile<<"Viewer.ViewpointZ: -1.8"<<endl;
    ofile<<"Viewer.ViewpointF: 500"<<endl;

}

void  getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,double &qx,double &qy,double &qz,double &qw,double &tx,double &ty,double &tz){
    //get the 3d part of matrix and get quaternion
    assert(M_in.total()==16);
    cv::Mat M;
    M_in.convertTo(M,CV_64F);
    cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
    //use now eigen
    Eigen::Matrix3f e_r33;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            e_r33(i,j)=M.at<double>(i,j);

    //now, move to a angle axis
    Eigen::Quaternionf q(e_r33);
    qx=q.x();
    qy=q.y();
    qz=q.z();
    qw=q.w();


    tx=M.at<double>(0,3);
    ty=M.at<double>(1,3);
    tz=M.at<double>(2,3);
}


cv::Mat  getMatrix(double qx,double qy, double qz,double qw,double tx,double ty ,double tz){


    double qx2 = qx*qx;
    double qy2 = qy*qy;
    double qz2 = qz*qz;


    cv::Mat m=cv::Mat::eye(4,4,CV_32F);

    m.at<float>(0,0)=1 - 2*qy2 - 2*qz2;
    m.at<float>(0,1)=2*qx*qy - 2*qz*qw;
    m.at<float>(0,2)=2*qx*qz + 2*qy*qw;
    m.at<float>(0,3)=tx;

    m.at<float>(1,0)=2*qx*qy + 2*qz*qw;
    m.at<float>(1,1)=1 - 2*qx2 - 2*qz2;
    m.at<float>(1,2)=2*qy*qz - 2*qx*qw;
    m.at<float>(1,3)=ty;

    m.at<float>(2,0)=2*qx*qz - 2*qy*qw	;
    m.at<float>(2,1)=2*qy*qz + 2*qx*qw	;
    m.at<float>(2,2)=1 - 2*qx2 - 2*qy2;
    m.at<float>(2,3)=tz;
    return m;
}

void savePosesToFile(string filename, const std::map<int, cv::Mat>& fmp)
{

//    vector<cv::Vec4f> points1,points2;

    std::ofstream file(filename);
    double qx, qy, qz, qw, tx, ty, tz;
    for (auto frame : fmp)
    {
        if (!frame.second.empty())
        {

//            cv::Mat p32;
//            frame.second.convertTo(p32,CV_32F);
//            points1.push_back( cv::Vec4f(p32.at<float>(0,3),p32.at<float>(1,3),p32.at<float>(2,3),12));
//            p32=p32.inv();
//            points2.push_back( cv::Vec4f(p32.at<float>(0,3),p32.at<float>(1,3),p32.at<float>(2,3),34987));
            getQuaternionAndTranslationfromMatrix44(frame.second, qx, qy, qz, qw, tx, ty, tz);
            file << frame.first << " " << tx << " " << ty << " " << tz << " " << qx << " " << qy << " " << qz << " "
                 << qw << endl;



        }
    }

//    savePcd("points1.pcd",points1);
//    savePcd("points2.pcd",points2);


}

int main(int argc, char **argv)
{
    if(argc != 7)
    {
        cerr << endl << "Usage: ./mono_marker path_to_vocabulary  path_to_image_folder path_to_times_file cameraparams.yml outposes(no '.txt') ARUCO_DIC" << endl;
        return 1;
    }
    cv::Size outSize(1280, 720);
    createYMLfromOpencvCamera(argv[4],"orbfile.yml",outSize); //!!!!!!!!!!!!!!

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[2]), string(argv[3]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    if(nImages<=0)
    {
        cerr << "ERROR: Failed to load images" << endl;
        return 1;
    }

    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],"orbfile.yml",ORB_SLAM2::System::MONOCULAR,true, argv[6]);

    // SLAM.LoadMap("/home/gzy/ORB_SLAM2_aruco/map.bin");//load the map 

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    // ORB_SLAM2::Timer::StartTimer(nImages);

    string outposes = argv[5];

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        // double T=0;
        // if(ni<nImages-1)
        //     T = vTimestamps[ni+1]-tframe;
        // else if(ni>0)
        //     T = tframe-vTimestamps[ni-1];

        // if(ttrack<T)
        //     usleep((T-ttrack)*1e6);

        // SLAM.ComputeArucoInMap();
    }

    sort(vTimesTrack.begin(),vTimesTrack.begin()+nImages);
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "----------------------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    SLAM.ActivateLocalizationMode();

    std::map<int,cv::Mat> poses;
    vTimesTrack.clear();

    for(int ni=0; ni<nImages; ni++)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // SLAM.ComputeArucoInMap();
    }

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    // SLAM.SaveTrajectoryTUM("EveryFrameTrajectory.txt");

    // SLAM.SaveMap("map.bin");

    // Stop all threads
    SLAM.Shutdown();
    string poseFile = outposes+".txt";
    savePosesToFile(poseFile,poses);

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e6);

        }
    }
}
