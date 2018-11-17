/*
 * Created By: Reid Oliveira
 * Created On: November 18th, 2017
 * Description:
 */

//https://stackoverflow.com/questions/18748775/c-opencv-accumulateweighted-strange-behaviour

#include <Accumulator.h>
//why does he use a const in HSVfilter.cpp

Accumulator::Accumulator(const cv::mat& video){

    acc = Mat::zeros(frame1.size(), CV_32FC1); //CV_32FC2 means a 2-channel (complex) floating-point array
}

void mask(const cv::mat& video, int size){


    accumulateWeighted(frame, accumulator, 0.05);


}