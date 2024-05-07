//
// Created by johan on 2024/4/30.
//

#ifndef DESIGN_ARMORDETETION_H
#define DESIGN_ARMORDETETION_H
#include"armor.h"

class ArmorDetetion {
public:
    ArmorDetetion();
    void Image_init(Mat& src){src_=src.clone();};
    // 寻找灯条
    bool FindLight(Mat &src);
    bool IsLight(const Light& light);
    // 寻找装甲板
    bool FindArmor(Mat &src);
    bool IsArmor(Armor & armor);

    // 数字识别
    void Digital_recognition(const cv::Mat& src, std::vector<Armor> armors, std::vector<Armor>& target_armors);
    void extractNumbers(const cv::Mat& src, std::vector<Armor>& armors);
    void classify(std::vector<Armor>& armors, std::vector<Armor>& target_armors);

    // 确定目标装甲板
    bool Target_Confirm(cv::Mat& src, std::vector<Armor>& target_armors);
    // 识别到的灯条
    std::vector<Light> lights_;
    // 识别到的所有装甲板（未经过数字识别判断）
    std::vector<Armor> armors_;
    // 确定是的装甲板
    std::vector<Armor> target_armors;
    // 目标装甲板
    Armor confirm_armor;

    // 图像识别属性
    EnemyColor enemy_color;

    // 原始图像
    Mat src_;

private:
    // 数字识别属性
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    std::vector<std::string> ignore_classes_;

};


#endif //DESIGN_ARMORDETETION_H
