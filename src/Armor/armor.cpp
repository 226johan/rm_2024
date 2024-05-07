//
// Created by johan on 2024/4/29.
//

#include "../../include/Armor/armor.h"

Light Armor::extendLight(const Light & light)
{
    Light new_light;
    new_light.top        = light.top;
    new_light.bottom     = light.bottom;
    new_light.length     = light.length;
    new_light.width      = light.width;
    new_light.tilt_angle = light.tilt_angle;

    new_light.angle  = light.angle;
    new_light.center = light.center;

    if (light.size.width < light.size.height)
    {
        new_light.size.height = 2 * light.size.height;
        new_light.size.width  = light.size.width;
    }
    else
    {
        new_light.size.width  = 2 * light.size.width;
        new_light.size.height = light.size.height;
    }
    return new_light;
}


// 排序完成后状态： 左上角为0 顺时针排序
void Armor::sortLightPoints(Point2f points[4],std::vector<cv::Point2f>& new_points)
{
    cv::Point2f temp_point;
    for (size_t i = 0; i < 4; i++)
    {
        for (size_t j = i + 1; j < 4; j++)
        {
            if (points[i].y > points[j].y)
            {
                temp_point = points[i];
                points[i]  = points[j];
                points[j]  = temp_point;
            }
        }
    }
    if (points[0].x > points[1].x)
    {
        temp_point = points[0];
        points[0]  = points[1];
        points[1]  = temp_point;
    }
    if (points[2].x < points[3].x)
    {
        temp_point = points[2];
        points[2]  = points[3];
        points[3]  = temp_point;
    }
    for (size_t i = 0; i < 4; i++)
    {
        new_points.push_back(points[i]);
    }
}