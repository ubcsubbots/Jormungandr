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

std::vector<float> Gate::initialize(const cv::Mat matin) {
    ROS_INFO("Initialize");
    cv::Canny(matin, dst, cannyLow, cannyHigh, 3);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dst, lines, 1, 1, 50, 50, 10);

    if (counter == 10) {

        vertLines = filterVertLines(lines);

        vertPoles = findVertPoles(vertLines);

        horLines = filterHorLines(lines);

        horPoles = findHorPoles(horLines);

        poleVector = gateVector(vertPoles , horPoles);

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

std::set<std::vector<float> > Gate::findVertPoles(const std::set<int> vertLines){
    std::set<std::vector<float> > vertPoles;
    vertPoles.clear();

    for (std::set<int>::iterator it1 = vertLines.begin(); it1 != vertLines.end(); it1++) {

        for (std::set<int>::iterator it2 = vertLines.begin(); it2 != vertLines.end(); it2++) {

            if ((*it1 != *it2) && (abs(*it1 - *it2) < poleMax)) {
                float topDist = (abs(*it1 - *it2) / (fiveMetreWidthofPole) * 5);
                float topAngle = atan(abs((*it1 - *it2) - (dst.rows/2))/ topDist);
                float a = (( *it1 +  *it2) /  2);
                float array[] = { a , topAngle , topDist};
                std::vector<float> newVec(*array,3);
                vertPoles.insert(newVec);
            }
        }
    }

    return vertPoles;

}

std::set<std::vector<float> > Gate::findHorPoles(const std::set<int> horLines){
    std::set<std::vector<float> > horPoles;
    horPoles.clear();

    for (std::set<int>::iterator it1 = horLines.begin(); it1 != horLines.end(); it1++) {

        for (std::set<int>::iterator it2 = horLines.begin(); it2 != horLines.end(); it2++) {

            if ((*it1 != *it2) && (abs(*it1 - *it2) < poleMax)) {
                float topDist = (abs(*it1 - *it2) / (fiveMetreWidthofPole) * 5);
                float topAngle = (float) atan(abs((*it1 - *it2) - (dst.rows/2.0f))/ topDist);
                float a = (( *it1 +  *it2) /  2.0f);
                float array[] = { a , topAngle , topDist};
                std::vector<float> newVec(*array,3);
                horPoles.insert(newVec);
            }
        }
    }

    return  horPoles;

}

std::vector<float> Gate::gateVector(std::set<std::vector<float> > vertPoles, std::set<std::vector<float> > horPoles){

    std::vector<float> gateCoord(9);

    if (horPoles.size() >= 1){

        std::set<std::vector<float> >::iterator it = horPoles.begin();

        std::vector<float> newVec = *it;

        gateCoord[6] = newVec[0];
        gateCoord[7] = newVec[1];
        gateCoord[8] = newVec[2];

        for(*it ; it != horPoles.end(); it++){

            std::vector<float> newVec1 = *it;

            if (newVec1[0] > gateCoord[6]){
                gateCoord[6] = newVec1[0];
                gateCoord[7] = newVec1[1];
                gateCoord[8] = newVec1[2];
            }
        }
    }
    else if (horPoles.empty()) gateCoord[6] =0, gateCoord[7] = 0, gateCoord[8] = 0;


    if (vertPoles.size() >= 2){

        std::set<std::vector<float> >::iterator it = horPoles.begin();

        std::vector<float> newVec = *it;

        gateCoord[0] = newVec[0];
        gateCoord[1] = newVec[1];
        gateCoord[2] = newVec[2];

        std::advance(it,1);

        std::vector<float> newVec0 = *it;

        gateCoord[3] = newVec0[0];
        gateCoord[4] = newVec0[1];
        gateCoord[5] = newVec0[2];

        for(*it ; it != horPoles.end(); it++){

            std::vector<float> newVec1 = *it;

            if (newVec1[0] > gateCoord[3]){
                gateCoord[3] = newVec1[0];
                gateCoord[4] = newVec1[1];
                gateCoord[5] = newVec1[2];
            }
            else if( newVec1[0] < gateCoord[0]){
                gateCoord[3] = newVec1[0];
                gateCoord[4] = newVec1[1];
                gateCoord[5] = newVec1[2];
            }
        }
    }
    else if(horPoles.size() == 1){


    }
    else if (horPoles.empty()) gateCoord[6] =0, gateCoord[7] = 0, gateCoord[8] = 0;

    return gateCoord;

}


/*for (std::set<int>::iterator it1 = vertPoles.begin(); it1 != vertPoles.end(); it1++) {
            cv::line(matin, cv::Point(*it1, 0),cv::Point(*it1,matin.rows),CV_RGB(255,255,255));
        }

        cv::imshow("Mat with lines", matin);
        cvWaitKey(0);*/