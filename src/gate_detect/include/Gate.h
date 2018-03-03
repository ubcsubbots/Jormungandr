//
// Created by da-cam on 20/02/18.
//

#ifndef PROJECT_GATE_H
#define PROJECT_GATE_H

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

class Gate{
    int maskWidth;
    cv::Mat cdst, dst, matout;
    int lowThreshold, lowVertThresh, lowHorThresh, poles;

public:
    Gate(const cv::Mat mat);

    Gate();

private:
    cv::Mat initialize(const cv::Mat mat);

    bool checkMat();

    std::vector<cv::Vec4i> filterHorLines(std::vector<cv::Vec4i> allLines);

    std::vector<cv::Vec4i> filterVertLines(std::vector<cv::Vec4i> allLines);

};

#endif //PROJECT_GATE_H
