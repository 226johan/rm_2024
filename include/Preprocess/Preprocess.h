//
// Created by johan on 2024/4/29.
//
#include"opencv4/opencv2/opencv.hpp"
#include"../Armor/armor.h"
using namespace cv;

void FillHole(cv::Mat srcBw, cv::Mat& dstBw);

Mat Preprocess(Mat & src,int enemy_color)
{
    Mat dst;
    Mat src_show=src.clone();
    Mat element = getStructuringElement(MORPH_ELLIPSE,cv::Size(5,5));
    if(enemy_color==RED)
    {
        Mat gray_threshold;
        std::vector<Mat> splited_channels;
        split(src_show,splited_channels);
        cvtColor(src_show,gray_threshold,COLOR_BGR2GRAY);
        threshold(gray_threshold,gray_threshold,96,255,THRESH_BINARY);

        Mat subtract_threshold;
        subtract(splited_channels[2],splited_channels[0],subtract_threshold);
        threshold(subtract_threshold,subtract_threshold,112,255,THRESH_BINARY);

        dilate(subtract_threshold,subtract_threshold,element);
        dst = subtract_threshold & gray_threshold;

        FillHole(dst,dst);

//        std::cout<<"red mode"<<std::endl;
    }
    else if (enemy_color==BLUE)
    {
        Mat gray_threshold;
        std::vector<Mat> splited_channels;
        split(src_show,splited_channels);
        cvtColor(src_show,gray_threshold,COLOR_BGR2GRAY);
        threshold(gray_threshold,gray_threshold,170,255,THRESH_BINARY);

        Mat subtract_threshold;
        subtract(splited_channels[0],splited_channels[2],subtract_threshold);
        threshold(subtract_threshold,subtract_threshold,110,255,THRESH_BINARY);

        dilate(subtract_threshold,subtract_threshold,element);
        dst = subtract_threshold & gray_threshold;

        FillHole(dst,dst);
//        std::cout<<"blue mode"<<std::endl;
    }
    else if(enemy_color==PURPLE)
    {
//        std::cout<<"purple mode"<<std::endl;
    }
    return dst;
}

void FillHole(cv::Mat srcBw, cv::Mat& dstBw)
{
    cv::Mat broedMat = cv::Mat::zeros(srcBw.rows + 2, srcBw.cols + 2, CV_8UC1);
    srcBw.copyTo(broedMat(cv::Range(1, srcBw.rows + 1), cv::Range(1, srcBw.cols + 1)));
    cv::floodFill(broedMat, cv::Point(0, 0), cv::Scalar(255));
    cv::Mat cutMat;
    broedMat(cv::Range(1, srcBw.rows + 1), cv::Range(1, srcBw.cols + 1)).copyTo(cutMat);
    dstBw = srcBw | (~cutMat);
}