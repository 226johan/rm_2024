//
// Created by johan on 2024/5/2.
//

#include "../../include/Tracker/Tracker.h"

void Trackers::TrackJudge(cv::Mat &src) {
    tracker_count++;
    armor_drop_count=5;
    // 连续5帧捕捉目标进入追踪模式
    if(tracker_count<5)
    {
        if_track = false;
    }
    else
    {
        if_track = true;
        putText(src, "Tracking Mode", cv::Point2f(500, 55), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
}