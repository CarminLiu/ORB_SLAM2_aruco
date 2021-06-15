/**
 * Learn From blog
 */

#ifndef SYSTEMSETTING_H
#define SYSTEMSETTING_H

#include<string>
#include"ORBVocabulary.h"
#include<opencv2/opencv.hpp>


namespace ORB_SLAM2 {

class SystemSetting{
public:
    SystemSetting(ORBVocabulary* pVoc);

    bool LoadSystemSetting(const std::string strSettingPath);

public:
    ORBVocabulary* pVocavulary;

    //相机参数
    float width;
    float height;
    float fx;
    float fy;
    float cx;
    float cy;
    float invfx;
    float invfy;
    float bf;
    float b;
    float fps;
    cv::Mat K;
    cv::Mat DistCoef;
    bool initialized;
    //相机 RGB 参数
    int nRGB;

    //ORB特征参数
    int nFeatures;
    float fScaleFactor;
    int nLevels;
    float fIniThFAST;
    float fMinThFAST;

    //其他参数
    float ThDepth = -1;
    float DepthMapFactor = -1;

};
    
}//namespace ORB_SLAM2

#endif //SystemSetting