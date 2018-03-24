#include "Gate.h"


Gate::Gate() {
    lowThreshold = 200;
    lowVertThresh = 5;
    lowHorThresh = 5;
    cannyLow = 65;
    cannyHigh = 250;
    poleLow = 40;
    counter = 0;
}

void Gate::initialize(const cv::Mat matin) {
    ROS_INFO("Initialize");
    cv::Canny(matin, dst, cannyLow, cannyHigh, 3);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dst, lines, 1, 0.5, 0, 10);

    if (counter == 0) {

        imshow("dst",dst);
        cvWaitKey(0);

        vertLines = filterVertLines(lines);
        vertPoles.clear();

        for (std::set<int>::iterator it1 = vertLines.begin(); it1 != vertLines.end(); it1++) {
            cv::line(matin, cv::Point(*it1, 0), cv::Point(*it1, matin.rows), CV_RGB(255, 255, 255));
        }

        cv::imshow("Mat with lines", matin);
        cvWaitKey(0);


        //ROS_INFO("Rows %i Colums %i ", matin.rows , matin.cols);

        for (std::set<int>::iterator it1 = vertLines.begin(); it1 != vertLines.end(); it1++) {

            ROS_INFO("Iterator 1 %i", *it1);

            for (std::set<int>::iterator it2 = vertLines.begin(); it2 != vertLines.end(); it2++) {

                if ((*it1 != *it2) && ((*it1 - *it2) < poleLow) && (*it1 < matin.cols) && (*it2 < matin.cols)) {

                    vertPoles.insert((*it1 + *it2) / 2);

                }
            }
        }
        /*
        for (std::set<int>::iterator it1 = vertPoles.begin(); it1 != vertPoles.end(); it1++) {
            ROS_INFO("Pole at %i", *it1);
            cv::line(matin, cv::Point(*it1, 0),cv::Point(*it1,matin.rows),CV_RGB(255,255,255));
        }
        */
        cv::imshow("Mat with lines", matin);
        cvWaitKey(0);


        horLines = filterHorLines(lines);
        counter = 0;
    } else counter++;

}

bool Gate::checkMat() {
    return (!dst.empty());
}

std::set<int> Gate::filterVertLines(std::vector<cv::Vec4i> allLines) {
    std::set<int> vertLines;

    for (int i = 0; i < (allLines.size() - 1); i++) {

        cv::Vec4i newLine = allLines[i];

        if (abs(newLine[0] - newLine[2]) < lowVertThresh) {
            ROS_INFO("Vertical Line at %i", newLine[0]);
            vertLines.insert(newLine[i]);
        }
    }

    return vertLines;
}

std::set<int> Gate::filterHorLines(std::vector<cv::Vec4i> allLines) {

    std::set<int> horLines;

    for (int i = 0; i < (allLines.size()); i++) {

        cv::Vec4i newLine = allLines[i];

        if (abs(newLine[1] - newLine[3]) < lowVertThresh) {
            horLines.insert(newLine[i]);
        }
    }

    return horLines;
}