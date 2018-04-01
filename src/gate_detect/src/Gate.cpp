#include "Gate.h"


Gate::Gate() {
    lowThreshold = 200;
    lowVertThresh = 5; //Sets the minimum deviation from start to finish that will constitute a vertical line
    lowHorThresh = 5; //Sets the minimum deviation from start to finish that will constitute a horizontal line
    cannyLow = 80; //Sets the low threshold for the canny filter, adjust if dst is not displaying all lines
    cannyHigh = 200; //Sets the high threshold for the canny filter, adjust if dst not displaying all lines
    poleMax = 60; //Sets the minimum threshold for two lines to be called a pole
    counter = 0; //Resets counter
    fiveMetreWidthofPole = 10.0f;
}

std::vector<float> Gate::initialize(const cv::Mat matin) {
    cv::Canny(matin, dst, cannyLow, cannyHigh, 3);

    std::vector<cv::Vec4i> lines;

    cv::HoughLinesP(dst, lines, 1, 1, 50, 50, 10);

    vertLines = filterVertLines(lines);

    vertPoles = findVertPoles(vertLines);

    horLines = filterHorLines(lines);

    horPoles = findHorPoles(horLines);

    poleVector = gateVector(vertPoles, horPoles);

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
        if ((abs(newLine[0] - newLine[2]) < lowVertThresh)) {
            vertLines.insert((newLine[0] + newLine[2]) / 2);
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

std::set<std::vector<float> > Gate::findVertPoles(const std::set<int> vertLines) {
    std::set<std::vector<float> > vertPoles;
    vertPoles.clear();



    for (auto it1 = vertLines.begin(); it1 != vertLines.end(); it1++) {
        ROS_INFO("First Line %i" ,*it1);
        for (auto it2 = vertLines.begin(); it2 != vertLines.end(); it2++) {
            ROS_INFO("Second Line %i" ,*it2);
            if ((*it1 != *it2) && (abs(*it1 - *it2) < poleMax) && (5 < abs(*it1 - *it2))) {
                ROS_INFO("Line is a Pole at %f" ,(float) ((*it1 + *it2) / 2) );
                float topDist = (abs(*it1 - *it2) / (fiveMetreWidthofPole)) * 5;
                float topAngle = atan(abs((*it1 - *it2) - (dst.cols / 2)) / topDist);
                float a = ((*it1 + *it2) / 2);
                float array[] = {a, topAngle, topDist};
                std::vector<float> newVec(array, array + sizeof(array) / sizeof(array[0]));
                vertPoles.insert(newVec);
            }
        }
    }



    return vertPoles;

}

std::set<std::vector<float> > Gate::findHorPoles(const std::set<int> horLines) {
    std::set<std::vector<float> > horPoles;
    horPoles.clear();

    for (auto it1 = horLines.begin(); it1 != horLines.end(); it1++) {


        for (auto it2 = horLines.begin(); it2 != horLines.end(); it2++) {
            if ((*it1 != *it2) && (abs(*it1 - *it2) < poleMax) && (5 < abs(*it1 - *it2))) {
                float topDist = (float) std::abs(*it1 - *it2) / (fiveMetreWidthofPole) * 5.0f;
                float topAngle = atan(std::abs((*it1 - *it2) - (dst.rows / 2)) / topDist);
                float a = ((*it1 + *it2) / 2);
                float array[] = {a, topAngle, topDist};
                std::vector<float> newVec(array, array + sizeof(array) / sizeof(array[0]));
                horPoles.insert(newVec);
            }
        }
    }



    return horPoles;

}

std::vector<float> Gate::gateVector(std::set<std::vector<float> > vertPoles, std::set<std::vector<float> > horPoles) {
    std::vector<float> gateCoord(9, 0.0f);
    for (std::vector<float> newVec : horPoles) {
        if (gateCoord[6] == 0.0f || newVec[0] > gateCoord[6]) {
            gateCoord[6] = newVec[0];
            gateCoord[7] = newVec[1];
            gateCoord[8] = newVec[2];
        }
    }
    for (std::vector<float> newVec : vertPoles) {
        if (gateCoord[0] == 0.0f || newVec[0] <= gateCoord[0]) {
            gateCoord[0] = newVec[0];
            gateCoord[1] = newVec[1];
            gateCoord[2] = newVec[2];
        }
        if (newVec[0] >= gateCoord[3]) {
            gateCoord[3] = newVec[0];
            gateCoord[4] = newVec[1];
            gateCoord[5] = newVec[2];
        }
    }


    cv::line(dst, cv::Point(gateCoord[0], 0),cv::Point(gateCoord[0],dst.rows),CV_RGB(255,255,255));
    cv::line(dst, cv::Point(gateCoord[3], 0),cv::Point(gateCoord[3],dst.rows),CV_RGB(255,255,255));

    cv::imshow("Mat with lines", dst);
    cvWaitKey(0);

    return gateCoord;
}


