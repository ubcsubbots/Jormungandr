#include "Gate.h"


Gate::Gate() {
    lowThreshold = 200;
    lowVertThresh = 5; //Sets the minimum deviation from start to finish that will constitute a vertical line
    lowHorThresh = 5; //Sets the minimum deviation from start to finish that will constitute a horizontal line
    cannyLow = 30; //Sets the low threshold for the canny filter, adjust if dst is not displaying all lines
    cannyHigh = 250; //Sets the high threshold for the canny filter, adjust if dst not displaying all lines
    poleMax = 60; //Sets the minimum threshold for two lines to be called a pole
    counter = 0; //Resets counter
}

std::vector<int> Gate::initialize(const cv::Mat matin) {
    ROS_INFO("Initialize");
    cv::Canny(matin, dst, cannyLow, cannyHigh, 3);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dst, lines, 1, 1, 50, 50, 10);

    if (counter == 10) {

        vertLines = filterVertLines(lines);

        vertPoles = findVertPoles(vertLines);

        horLines = filterHorLines(lines);

        horPoles = findHorPoles(horLines);

        counter = 0;

    } else counter++;



    return poleVector;

}

bool Gate::checkMat() {
    return (!dst.empty());
}

std::set<int> Gate::filterVertLines(std::vector<cv::Vec4i> allLines) {
    std::set<int> vertLines;
    vertLines.clear();

    for (int i = 0; i < (allLines.size() - 1); i++) {

        cv::Vec4i newLine = allLines[i];

        if (abs(newLine[0] - newLine[2]) < lowVertThresh) {
            vertLines.insert((newLine[0] + newLine[2])/2);
        }
    }

    return vertLines;
}

std::set<int> Gate::filterHorLines(std::vector<cv::Vec4i> allLines) {
    std::set<int> horLines;
    horLines.clear();

    for (int i = 0; i < (allLines.size()); i++) {

        cv::Vec4i newLine = allLines[i];

        if (abs(newLine[1] - newLine[3]) < lowHorThresh) {
            horLines.insert(newLine[i]);
        }
    }

    return horLines;
}

std::set<int> Gate::findVertPoles(std::set<int> vertLines){
    std::set<int> vertPoles;
    vertPoles.clear();

    for (std::set<int>::iterator it1 = vertLines.begin(); it1 != vertLines.end(); it1++) {

        for (std::set<int>::iterator it2 = vertLines.begin(); it2 != vertLines.end(); it2++) {

            if ((*it1 != *it2) && (abs(*it1 - *it2) < poleMax)) {

                vertPoles.insert((*it1 + *it2) / 2);

            }
        }
    }

    return vertPoles;

}

std::set<int> Gate::findHorPoles(std::set<int> horLines){
    std::set<int> horPoles;
    horPoles.clear();

    for (std::set<int>::iterator it1 = horLines.begin(); it1 != horLines.end(); it1++) {

        for (std::set<int>::iterator it2 = horLines.begin(); it2 != horLines.end(); it2++) {

            if ((*it1 != *it2) && (abs(*it1 - *it2) < poleMax)) {

                horPoles.insert((*it1 + *it2) / 2);

            }
        }
    }

    return  horPoles;

}

std::vector<int> gateVector(std::set<int> vertPoles, std::set<int> horPoles){

    std::vector<int> gateCoord(9);

    if (horPoles.size() > 1){

        std::set::iterator it = horPoles.begin();
        gateCoord[6] = *it;

        for(*it ; *it != horPoles.end(); it++){

            if (*it > gateCoord[6]){
                gateCoord[6] = *it;
                gateCoord[7] = 0;
                gateCoord[8] = 0;
            }
        }
    }
    else if (horPoles.size() ==0) gateCoord[6] =0, gateCoord[7] = 9, gateCoord[8] = 0;
    else gateCoord[6] = *horPoles.begin(), gateCoord[7] = 9, gateCoord[8] = 0;

    if (vertPoles.size() > 2){

        std::set::iterator it = vertPoles.begin();
        gateCoord[6] = *it;

        for(*it ; *it != vertPoles.end(); it++){

            if (*it > gateCoord[6]){
                gateCoord[6] = *it;
            }
        }
    }
    else if (horPoles.size() ==0) gateCoord[0] =0, gateCoord[1] = 9, gateCoord[2] = 0, gateCoord[3] =0, gateCoord[4] = 9, gateCoord[5] = 0;
    else gateCoord[6] = *horPoles.begin(), gateCoord[7] = 9, gateCoord[8] = 0;

    return gateCoord;

}


/*for (std::set<int>::iterator it1 = vertPoles.begin(); it1 != vertPoles.end(); it1++) {
            cv::line(matin, cv::Point(*it1, 0),cv::Point(*it1,matin.rows),CV_RGB(255,255,255));
        }

        cv::imshow("Mat with lines", matin);
        cvWaitKey(0);*/