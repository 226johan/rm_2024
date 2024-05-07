#ifndef CAMERA_MINDVISION_RUNNING_H
#define CAMERA_MINDVISION_RUNNING_H

#include "CameraApi.h"  //相机SDK的API头文件

#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core/core.hpp"
#include "opencv4/opencv2/highgui/highgui.hpp"
#include <opencv4/opencv2/imgproc/imgproc_c.h>
#include <opencv4/opencv2/imgproc/types_c.h>
#include <stdio.h>
#include"../debug.h"


using namespace cv;

class MindvisionCamera
{
public:
    int CameraParamInit(int iLanguageSel);  //初始化

    bool runCamera();             //运行相机
    bool getImage(cv::Mat& src);  //获取图像
    bool Do();
    bool CameraParamSet();

    void                SetParam();
    int                 iCameraCounts = 1;
    int                 iStatus       = -1;
    tSdkCameraDevInfo   tCameraEnumList;
    int                 hCamera;      //相机的句柄
    tSdkCameraCapbility tCapability;  //设备描述信息
    tSdkFrameHead       sFrameInfo;
    BYTE*               pbyBuffer;
    unsigned char*      g_pRgbBuffer;  //处理后数据缓存区
    IplImage*           iplImage = NULL;
    int                 channel  = 3;
    cv::Mat             src_;
    BOOL                bAeState;  //相机曝光模式
    tSdkImageResolution tRes;      //相机帧率
    int                 iFrameSpeed;
    bool                SetExposureTime(double fExposureTime);        //设置曝光时间
    bool                SetGain(int iRGain, int iGGain, int iBGain);  //设置图像增益
    bool                SetGamma(int camera_gamma);                   //设置伽马
    bool                SetSize();
    bool                SetFrameSpeed();
    //    bool                VideoStart();  //视频录制模式
    //    bool                VideoRecord();
    //    bool                VideoStop();
    int   iFormat    = 1;                                                        //录像的格式 0:不压缩；1:MSCV方式压缩。
    char* pcSavePath = "~/rm_2023_05_10/NCIST_RM2023_Sentry/camera_mindvision";  //录像文件保存的路径
    BOOL  b2GLimit   = false;                                                    //如果为TRUE,则文件大于2G时自动分割。
    DWORD dwQuality  = 35;                                                       //录像的质量因子，越大，则质量越好。范围1到100.
    int   iFrameRate = 170;                                                      //录像的帧率。建议设定的比实际采集帧率大，

private:
    double exposure_time;  // 曝光时间
    int    camera_gamma;   //相机伽马
    int    iRGain;         // R gain
    int    iGGain;         // G gain
    int    iBGain;         // B Gain
};

#endif  // CAMERA_MINDVISION_RUNNING_H
