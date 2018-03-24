//
// Created by da-cam on 20/02/18.
//

#ifndef PROJECT_GATE_H
#define PROJECT_GATE_H

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

class Gate {
    cv::Mat dst;
    int lowThreshold, lowVertThresh, lowHorThresh, cannyLow, cannyHigh, poleMax, counter;
    std::set<int> vertLines, vertPoles, horLines, horPoles;
    std::vector<int> poleVector;

public:
    Gate();

    /*
     * Function to be called on cv::Mat to find vertical poles and horizontal poles
     *
     * mat : cv::Mat potentially containing a gate
     *
     */

    std::vector<int>  initialize(const cv::Mat mat);

    /*
     * Function to check if Gate object has been initialized
     */

    bool checkMat();

private:

    /*
     * Function to filter through vector of cv::Vector4i objects and filter out the horizontal lines
     *
     * allLines : Vector of cv::Vector4i objects in which the Vector4i represents a lines such that [xo , yo , xf, yf]
     *
     * xo = x coordinate of start of line
     *
     * yo = y coordinate of start of line
     *
     * xf = end coordinate of end of line
     *
     * yf = end coordinate of end of line
     *
     */

    std::set<int> filterHorLines(std::vector<cv::Vec4i> allLines);

    /*
     * Function to filter through vector of cv::Vector4i objects and filter out the Vertical lines
     *
     * allLines : Vector of cv::Vector4i objects in which the Vector4i represents a lines such that [xo , yo , xf, yf]
     *
     * xo = x coordinate of start of line
     *
     * yo = y coordinate of start of line
     *
     * xf = end coordinate of end of line
     *
     * yf = end coordinate of end of line
     *
     */

    std::set<int> filterVertLines(std::vector<cv::Vec4i> allLines);

    /*
    * Function to filter through set of horizontal lines to determine which lines constitute a pole
    *
    * int poleMax : maximum distance between two horizontal lines that function will call a pole
    *
    */

    std::set<int> findHorPoles(std::set<int> horLines);

    /*
     * Function to filter through set of vertical lines to determine which lines constitute a pole
     *
     * int poleMax : maximum distance between two vertical lines that function will call a pole
     *
     */

    std::set<int> findVertPoles(std::set<int> vertLines);

    std::vector<int> gateVector(std::set<int> vertPoles, std::set<int> horPoles);


};

#endif //PROJECT_GATE_H
