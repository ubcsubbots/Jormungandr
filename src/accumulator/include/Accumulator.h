/*
 * Created By: Viral Galaiya
 * Created On: November 16th, 2018
 * Description:
 */
#ifndef PROJECT_ACCUMULATOR_H
#define PROJECT_ACCUMULATOR_H

#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

class Accumulator {


public:
    Accumulator();
    cv::Mat acc;
    cv::Mat gray;
    int flag;
    void mask(const cv::Mat& video);


private:


};



#endif //PROJECT_ACCUMULATOR_H

