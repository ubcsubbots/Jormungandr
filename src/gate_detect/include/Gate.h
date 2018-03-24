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
    int lowThreshold, lowVertThresh, lowHorThresh, poles, cannyLow, cannyHigh, poleLow, counter;
    std::set<int> vertLines,vertPoles, horLines, horPoles;

public:
    Gate();

    /*
     *
     *
     */

    void initialize(const cv::Mat mat);

    bool checkMat();

private:

    /*
     *
     *
     */

    std::set<int> filterHorLines(std::vector<cv::Vec4i> allLines);

    std::set<int> filterVertLines(std::vector<cv::Vec4i> allLines);

};

#endif //PROJECT_GATE_H
