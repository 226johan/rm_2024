//
// Created by johan on 2024/4/29.
//

#ifndef DESIGN_ARMOR_H
#define DESIGN_ARMOR_H

#include"../debug.h"
#include "opencv2/opencv.hpp"
#include"eigen3/Eigen/Eigen"
using namespace cv;

struct Light : public cv::RotatedRect
{
    Light() = default;
    explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
    {
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
        top    = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width  = (cv::norm(p[0] - p[1])+cv::norm(p[2]-p[3])) / 2;

        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = tilt_angle / CV_PI * 180;
    }

    cv::Point2f p[4];
    cv::Point2f top, bottom;
    double      length, width;
    float       tilt_angle;
};

struct Armor : public Light {
    Armor()=default;
    Armor(const Light& l1,const Light& l2)
    {
        if(l1.center.x<l2.center.y)
        {
            left_light = l1;
            right_light = l2;
        }
        else
        {
            left_light = l2;
            right_light = l1;
        }
        center = (left_light.center + right_light.center) /2;

        // 顶点顺序：左下角为0，顺时针旋转
        armor_points.push_back(left_light.bottom);
        armor_points.push_back(left_light.top);
        armor_points.push_back(right_light.top);
        armor_points.push_back(right_light.bottom);

        //顶点顺序：左上角为0，顺时针旋转
//        armor_points.push_back(left_light.top);
//        armor_points.push_back(right_light.top);
//        armor_points.push_back(right_light.bottom);
//        armor_points.push_back(left_light.bottom);


        // 延长灯条
        left_light_= extendLight(left_light);
        right_light_= extendLight(right_light);

        cv::Point2f left_light_points[4],right_light_points[4];
        left_light_.points(left_light_points);
        right_light_.points(right_light_points);
        std::vector<cv::Point2f> left_light_points_,right_light_points_;
        sortLightPoints(left_light_points,left_light_points_);
        sortLightPoints(right_light_points,right_light_points_);

        //最后状态： 左上角为0 按顺时针排序
        armor_points_.push_back(left_light_points_[1]);
        armor_points_.push_back(right_light_points_[0]);
        armor_points_.push_back(right_light_points_[3]);
        armor_points_.push_back(left_light_points_[2]);


    }

    Light extendLight(const Light & light);
    void sortLightPoints(Point2f points[4],std::vector<cv::Point2f>& new_points);
    // before
    Light                    left_light, right_light;
    cv::Point2f              center;
    std::vector<cv::Point2f> armor_points;
    // final
    Light left_light_,right_light_;
    std::vector<cv::Point2f> armor_points_;

    std::string number;      // 数字结果
    float confidence;        // 置信度
    std::string classfication_result;
    Mat   number_img;       // 48*48的二值化数字图像
    ArmorType type;

    // resolver
    Eigen::Affine3d pose;
    double distance;
    double distance_to_image_center;

    int score;      // 击打评分
};


#endif //DESIGN_ARMOR_H
