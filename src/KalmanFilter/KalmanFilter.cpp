//
// Created by johan on 2024/5/2.
//

#include "../../include/KalmanFilter/KalmanFilter.h"


Kalmanfilter::Kalmanfilter()
{
    const int stateNum = 6;             // 状态矩阵6*1(yaw,pitch,v_yaw,v_pitch,a_yaw,a_pitch)
    const int measureNum = 2;           // 测量矩阵2*1(yaw,pitch)
    const int controlNum = 0;           // 控制值

    KF = cv::KalmanFilter(stateNum, measureNum, controlNum, CV_32F);

    statement   = Mat::zeros(stateNum,1,CV_32F);
    measurement = Mat::zeros(measureNum, 1, CV_32F);


    setIdentity(KF.transitionMatrix, Scalar(1));

    KF.measurementMatrix = Mat::zeros(measureNum, stateNum, CV_32F);


    KF.measurementMatrix = (cv::Mat_<float>(2, 6) <<1,0,0,0,0,0,0,1,0,0,0,0);  //测量矩阵H


    KF.processNoiseCov.at<float>(0)  = 0.7f;
    KF.processNoiseCov.at<float>(7)  = 0.7f;
    KF.processNoiseCov.at<float>(14) = 5.0f;
    KF.processNoiseCov.at<float>(21) = 5.0f;
    KF.processNoiseCov.at<float>(28) = 10.0f;
    KF.processNoiseCov.at<float>(35) = 10.0f;

    setIdentity(KF.measurementNoiseCov, Scalar(100));  //测量噪声方差矩阵R

    setIdentity(KF.errorCovPost, Scalar::all(1));    //后验错误估计协方差矩阵P
}

Kalmanfilter::~Kalmanfilter() {}

void Kalmanfilter::InitKalman(){

    setIdentity(KF.errorCovPre, Scalar(1));  //  6*6矩阵

    // 初始化 状态向量
    statement.at<float>(0) = gain_yaw + send_yaw;
    statement.at<float>(1) = gain_pitch + send_pitch;
    statement.at<float>(2) = 0;
    statement.at<float>(3) = 0;
    statement.at<float>(4) = 0;
    statement.at<float>(5) = 0;

    // 设定系统初始状态
    KF.statePost = statement;
}

void Kalmanfilter::UpDate(float delta_time){

    dT = delta_time;

    //1.更新状态转移矩阵A
    KF.transitionMatrix.at<float>(2)  = dT;
    KF.transitionMatrix.at<float>(9)  = dT;
    KF.transitionMatrix.at<float>(16) = dT;
    KF.transitionMatrix.at<float>(23) = dT;
    KF.transitionMatrix.at<float>(4)  = 1 / 2 * dT * dT;
    KF.transitionMatrix.at<float>(11) = 1 / 2 * dT * dT;

    //2.kalman prediction
    KF.predict();

    //3.update measurement
    measurement.at<float>(0) = send_yaw + gain_yaw;
    measurement.at<float>(1) = send_pitch + gain_pitch;

    //4.update
    KF.correct(measurement);
}

void Kalmanfilter::KalmanPredict(float bullet_flight_time){

    v_yaw   = KF.statePost.at<float>(2)*0.35 ;
    v_pitch = KF.statePost.at<float>(3) ;

    a_yaw   = KF.statePost.at<float>(4)*0.65 ;
    a_pitch = KF.statePost.at<float>(5) ;

    send_yaw   -= (v_yaw   * bullet_flight_time  + 1/2 * a_yaw * bullet_flight_time * bullet_flight_time * 1000.0f);
    send_pitch += (v_pitch * bullet_flight_time * 0.86f + 1/2 * a_pitch * bullet_flight_time * bullet_flight_time * 1000.0f);
}