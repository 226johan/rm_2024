//
// Created by johan on 2024/4/30.
//

#include "../../include/Armor/ArmorDetetion.h"


ArmorDetetion::ArmorDetetion()
{
    net_ = cv::dnn::readNetFromONNX(onnx_path);
    std::ifstream label_file(label_path);
    std::string   line;
    while (std::getline(label_file, line))
    {
        class_names_.push_back(line);
    }
}

bool ArmorDetetion::FindLight(Mat &src)
{
    Mat src_=src.clone();
    // 每次处理的是一帧中的图像，每次寻找前注意清空
    lights_.clear();

    std::vector<std::vector<cv::Point2i>> contours;
    std::vector<cv::Vec4i>                hierarchy;
    findContours(src_, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto contour : contours)
    {
        cv::RotatedRect rrect = minAreaRect(contour);
        auto            light = Light(rrect);
//        灯条筛选
        if (IsLight(light))
        {
            lights_.push_back(light);
        }
    }

    if(lights_.size() == 0 )
    {
        std::cout<<"lights empty"<<std::endl;
        return false;
    }
    else
    {
        for(auto light_ : lights_)
        {
            Point2f light_points[4];
            light_.points(light_points);
            for(size_t i=0;i<4;i++)
            {
                line(this->src_,light_points[i],light_points[(i+1) % 4],Scalar(0,255,0),2,8);
            }
            imshow("light",this->src_);
            std::cout<<"light numbers: "<<lights_.size()<<std::endl;

        }
        return true;
    }
}

bool ArmorDetetion::IsLight(const Light& light)
{

    float ratio = light.width / light.length;
    // std::cout<<"ratio="<<ratio<<std::endl;
    // 基准比例 5.1/9=0.56
    bool ratio_ok = 0.05f < ratio && ratio < 0.65f;

    bool angle_ok = light.tilt_angle < 40.0f;
    // std::cout<<"light.tilt_angle="<<light.tilt_angle<<std::endl;
    bool is_light = ratio_ok && angle_ok;
    //    std::cout<<"______________________________________"<<std::endl;

    return is_light;
}

bool ArmorDetetion::FindArmor(Mat &src)
{
    // 每次处理的是一帧中的图像，每次寻找前注意清空
    armors_.clear();

    for (auto light_1 = lights_.begin(); light_1 != lights_.end(); light_1++)
    {
        for (auto light_2 = light_1 + 1; light_2 != lights_.end(); light_2++)
        {
            auto armor = Armor(*light_1, *light_2);
            if (IsArmor(armor))
            {
                armors_.emplace_back(armor);
            }
        }
    }

    if(armors_.size()==0)
    {
        std::cout<<"not find armor "<<std::endl;
        return false;
    }
    else
    {
        for (size_t i = 0; i < armors_.size(); i++)
        {
            for (uint8_t j = 0; j < 4; j++)
            {
                line(this->src_, armors_[i].armor_points[j], armors_[i].armor_points[(j + 1) % 4], cv::Scalar(0, 255, 0), 1, 8);
            }
            imshow("armor", this->src_);
        }
        return true;
    }

}


bool ArmorDetetion::IsArmor(Armor & armor)
{
    Light light_1 = armor.left_light_;
    Light light_2 = armor.right_light_;

    float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length : light_2.length / light_1.length;
     std::cout<<"light_length_ratio ="<<light_length_ratio<<std::endl;

    // 0.8
    bool light_ratio_ok = light_length_ratio > 0.7 /*a.min_light_ratio*/;
    std::cout << "light_ratio " << light_length_ratio << std::endl;
    float avg_light_length = (light_1.length + light_2.length) / 2;

    //     std::cout<<"avg_light_length ="<<avg_light_length<<std::endl;

    float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;

    //     std::cout<<"center_distance = "<<center_distance<<std::endl;

    // 0.5 4.0
    bool center_distance_ok = (/*a.min_small_center_distance*/ 0.5 < center_distance && center_distance < 4.5 /*a.max_small_center_distance*/); /*||*/
    //                            (/*a.min_large_center_distance*/3.2 < center_distance /*&&
    //                             center_distance <4.0 *//*a.max_large_center_distance*/);

    cv::Point2f diff  = light_1.center - light_2.center;
    float       angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;

    // 8.0
    bool angle_ok = angle < /*a.max_angle*/ 8.0;
    //     std::cout<<"angle="<<angle<<std::endl;

    bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;
    // std::cout<<"is_amor =="<<is_armor <<std::endl;

    //根据装甲板长宽比判断装甲板类型
    armor.type = center_distance > /*a.min_large_center_distance*/ 3.2 ? BIG : SMALL;
    //    std::cout<<"center_distence: "<<center_distance<<std::endl;
    while (0)
    {
        cv::Mat armor_frame(640, 512, CV_8UC3, cv::Scalar(0, 0, 0));
        putText(armor_frame, "light_length_ratio :" + std::to_string(light_length_ratio), cv::Point(4, 15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        putText(armor_frame, "center_distance :" + std::to_string(center_distance), cv::Point(4, 35), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        putText(armor_frame, "angle :" + std::to_string(angle), cv::Point(4, 55), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    }
    return is_armor;
}

void ArmorDetetion::Digital_recognition(const cv::Mat& src, std::vector<Armor> armors, std::vector<Armor>& target_armors)
{
    extractNumbers(src, armors_);
    classify(armors_, target_armors);
}

// 压根用不到src
void ArmorDetetion::extractNumbers(const cv::Mat& src, std::vector<Armor>& armors)
{
    cv::Mat src_show = src.clone();

    for (auto& armor : armors)
    {
        cv::Point2f affinePoints[4];
        std::cout<<"armor type: "<<armor.type<<std::endl;
        if(armor.type==BIG)
        {
            affinePoints[0]=cv::Point2f(armor.armor_points_[0].x + 120, armor.armor_points_[0].y);
            affinePoints[1]=cv::Point2f(armor.armor_points_[1].x - 100, armor.armor_points_[1].y);
            affinePoints[2]=cv::Point2f(armor.armor_points_[2].x - 100, armor.armor_points_[2].y);
            affinePoints[3]=cv::Point2f(armor.armor_points_[3].x + 120, armor.armor_points_[3].y);
        }

        if(armor.type==SMALL)
        {
            affinePoints[0]=cv::Point2f(armor.armor_points_[0].x + 15, armor.armor_points_[0].y);
            affinePoints[1]=cv::Point2f(armor.armor_points_[1].x - 10, armor.armor_points_[1].y);
            affinePoints[2]=cv::Point2f(armor.armor_points_[2].x - 10, armor.armor_points_[2].y);
            affinePoints[3]=cv::Point2f(armor.armor_points_[3].x + 15, armor.armor_points_[3].y);
        }

        cv::Point2f affinePoints2[4] = { cv::Point2f(0, 0), cv::Point2f(48, 0), cv::Point2f(48, 48), cv::Point2f(0, 48) };

        cv::Mat affine_trans;
        //透视变换
        affine_trans = getPerspectiveTransform(affinePoints, affinePoints2);
        cv::Mat number_image;
        //仿射变换
        cv::warpPerspective(src, number_image, affine_trans, cv::Size(48, 48), cv::INTER_CUBIC);
        // Binarize
        cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
        cv::threshold(number_image, number_image, 50, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        //        cv::threshold(number_image, number_image, 80, 240, cv::THRESH_BINARY);
        armor.number_img = number_image;
    }
}

void ArmorDetetion::classify(std::vector<Armor>& armors, std::vector<Armor>& target_armors)
{
    for (auto& armor : armors)
    {
        target_armors.clear();
        cv::Mat image = armor.number_img.clone();
        // 先进行归一化
        image = image / 255.0;

        cv::Mat blob;
        // 20,28
        cv::dnn::blobFromImage(image, blob, 1.0, cv::Size(20, 28));

        // 输入
        net_.setInput(blob);
        // 分类预测
        cv::Mat outputs = net_.forward();

        float   max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
        cv::Mat softmax_prob;
        cv::exp(outputs - max_prob, softmax_prob);
        float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
        softmax_prob /= sum;

        double    confidence;
        cv::Point class_id_point;
        // 概率最高值
        minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
        int label_id = class_id_point.x;

        armor.confidence = confidence;
        armor.number     = class_names_[label_id];
        std::stringstream result_ss;
        result_ss << armor.number << ": " << std::fixed << std::setprecision(1) << armor.confidence * 100.0 << "%";
        armor.classfication_result = result_ss.str();

        std::cout<<armor.classfication_result<<std::endl;

        cv::imshow("11", armor.number_img);

        if (confidence * 100.0 > 10 && class_names_[label_id] != "negative")
        {
            target_armors.push_back(armor);
            cv::imshow("1", armor.number_img);
        }
    }
}

bool ArmorDetetion::Target_Confirm(cv::Mat& src, std::vector<Armor>& target_armors)
{
    if(target_armors.size()==0)
    {
        std::cout<<"don't have armor with number"<<std::endl;
        return false;
    }
    else if(target_armors.size()==1)
    {
        this->confirm_armor=target_armors[0];

        putText(this->src_, confirm_armor.classfication_result, cv::Point2f(confirm_armor.armor_points[0].x, confirm_armor.armor_points[0].y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
        if (confirm_armor.type == SMALL)
        {
            putText(this->src_, "small_armor", cv::Point2f(confirm_armor.armor_points[0].x, confirm_armor.armor_points[0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
            for (size_t k = 0; k < 4; k++)
            {
                line(this->src_, confirm_armor.armor_points[k], confirm_armor.armor_points[(k + 1) % 4], cv::Scalar(0, 255, 255), 2, 8);
            }
        }
        else if (confirm_armor.type == BIG)
        {
            putText(this->src_, "big_armor", cv::Point2f(confirm_armor.armor_points[0].x, confirm_armor.armor_points[0].y - 30), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(165, 155, 238), 1);
            for (size_t k = 0; k < 4; k++)
            {
                line(this->src_, confirm_armor.armor_points[k], confirm_armor.armor_points[(k + 1) % 4], cv::Scalar(0, 255, 255), 2, 8);
            }
        }
        imshow("confirm_armor",this->src_);
        return true;
    }
    else
    {
        for(size_t i=0;i<target_armors.size();i++)
        {

        }
        return true;
    }
}