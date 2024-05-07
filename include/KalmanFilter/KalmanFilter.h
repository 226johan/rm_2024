//
// Created by johan on 2024/5/2.
//

#ifndef RM_2024_KALMANFILTER_H
#define RM_2024_KALMANFILTER_H
#include "../Resolver/Resolver.h"
#include <iostream>
#include <opencv2/opencv.hpp>

class Kalmanfilter {
public:
    Kalmanfilter();
    ~Kalmanfilter();

    void InitKalman();
    void UpDate(float dT);
    void KalmanPredict(float bullet_flight_time);

    cv::KalmanFilter KF;
    cv::Mat statement;
    cv::Mat measurement;

    float bullet_flighy_time;       // 子弹飞行时间   时间*角速度
    float dT;                       // 帧时间差   计算角速度

    float v_yaw;
    float v_pitch;

    float a_yaw;
    float a_pitch;

    float gain_yaw;
    float gain_pitch;

    float send_yaw;
    float send_pitch;


};


#endif //RM_2024_KALMANFILTER_H
