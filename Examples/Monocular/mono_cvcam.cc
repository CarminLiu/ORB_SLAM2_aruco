/**

*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <csignal>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                                    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };

bool stopCapture=false;
void ctrl_c_signal(int s){
    stopCapture=true;

}
void savePosesToFile(string filename, const std::map<int, cv::Mat>& fmp);

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
    ofile<<"ORBextractor.nFeatures: 1000"<<endl;
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

cv::Size parseSize(string str){
    for(auto &s:str)if (s==':')s=' ';
    stringstream sstr;
    sstr<<str;
    cv::Size  wh;
    sstr>>wh.width>>wh.height;
    return wh;
}

cv::Mat resizeInput(cv::Mat &in,cv::Size outsize){
    if (outsize.width<=0)return in;
    else{
        cv::Mat ret;
        cv::resize(in,ret,outsize);
        return ret;
    }

}
int main(int argc, char **argv)
{
    try
    {
        if(argc != 6)
        {
            cerr << endl << "Usage: ./mono_tum path_to_vocabulary    videofile  cameraparams.yml outposes ARUCO_DIC " << endl;
            return 1;
        }
        cv::Size outSize(960,540);
        createYMLfromOpencvCamera(argv[3],"orbfile.yml",outSize);
        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        ORB_SLAM2::System SLAM(argv[1],"orbfile.yml",ORB_SLAM2::System::MONOCULAR,true,argv[5]);
        cv::VideoCapture camera;
        //grab until first frame acquired
        cv::Mat im;
        // Main loop
        signal(SIGINT,ctrl_c_signal);

         uint32_t frameIndx=0;

        camera.open( argv[2]);

        if (!camera.isOpened()) throw std::runtime_error("Could not open source ");
        cout<<"Video 1 opened"<<endl;
        
        while(camera.grab()){

             std::this_thread::sleep_for(std::chrono::milliseconds(20));
            camera.retrieve(im);
            im=resizeInput(im,outSize);
            // Pass the image to the SLAM system
            SLAM.TrackMonocular(im,frameIndx++);
        }
        camera.release();

        // Save camera trajectory
        SLAM.ActivateLocalizationMode();

        camera.open(argv[2]);
        if (!camera.isOpened()) throw std::runtime_error("Could not open source ");
        std::map<int,cv::Mat> poses;
        frameIndx=0;
        
        while(camera.grab()){
            camera.retrieve(im);
            im=resizeInput(im,outSize);
            // Pass the image to the SLAM system
            cv::Mat pose=SLAM.TrackMonocular(im,frameIndx);
            if (frameIndx==0)//do not ask me why
                pose=SLAM.TrackMonocular(im,frameIndx);
            if (!pose.empty())
                poses.insert({frameIndx,pose.inv()});
            frameIndx++;
            
        }



        // Stop all threads
        SLAM.Shutdown();
        savePosesToFile(argv[4],poses);
        return 0;
    }catch(std::exception &ex){
        cout<<ex.what()<<endl;
    }
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
