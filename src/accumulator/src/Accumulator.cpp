/*
 * Created By: Viral Galaiya
 * Created On: July 23, 2018
 * Description:
 */
//https://stackoverflow.com/questions/18748775/c-opencv-accumulateweighted-strange-behaviour

#include <Accumulator.h>


Accumulator::Accumulator(){


    acc = cv::Mat::zeros(cv::Size(1,49), CV_32FC1); //CV_32FC2 means a 2-channel (complex) floating-point array
}

void Accumulator::mask(const cv::Mat& frame){


    accumulateWeighted(frame, acc, 0.05);


}