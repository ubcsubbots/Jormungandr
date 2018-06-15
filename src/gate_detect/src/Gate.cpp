#include "Gate.h"

Gate::Gate() {
    lowVertThresh = 50; // Sets the maximum deviation from start to finish that
                        // will constitute a vertical line
    lowHorThresh = 100; // Sets the maximum deviation from start to finish that
                        // will constitute a horizontal line
    cannyLow = 50; // Sets the low threshold for the canny filter, adjust if dst
                   // is not displaying all lines
    cannyHigh = 200; // Sets the high threshold for the canny filter, adjust if
                     // dst not displaying all lines
    poleMax =
    200; // Sets the minimum threshold for two lines to be called a pole
    m                = -0.002673; // Readjust when calibrated
    b                = 5.134;     // Distance from gate = m * x + b
    camera_res_width = 1920;
}

std::vector<float> Gate::initialize(const cv::Mat matin) {
    cv::Canny(matin, dst, cannyLow, cannyHigh, 3);

    cv::HoughLinesP(dst, lines, 1, CV_PI / 180, 60, 50, 50);

    if (lines.empty()) return std::vector<float>(9, 0);

    _vertLines = filterVertLines(lines);

    _vertPoles = findVertPoles(_vertLines);

    _horLines = filterHorLines(lines);

    _horPoles = findHorPoles(_horLines);

    poleVector = gateVector(_vertPoles, _horPoles);

    return poleVector;
}

bool Gate::checkMat() {
    return (!dst.empty());
}

std::vector<int> Gate::filterVertLines(std::vector<cv::Vec4i> allLines) {
    std::vector<int> vertLines;
    vertLines.clear();

    for (int i = 0; i < (allLines.size() - 1); i++) {
        cv::Vec4i newLine = allLines[i];
        if ((abs(newLine[0] - newLine[2]) < lowVertThresh)) {
            vertLines.push_back((newLine[0] + newLine[2]) / 2);
        }
    }

    return vertLines;
}

std::vector<int> Gate::filterHorLines(std::vector<cv::Vec4i> allLines) {
    std::vector<int> horLines;
    horLines.clear();

    for (int i = 0; i < (allLines.size()); i++) {
        cv::Vec4i newLine = allLines[i];

        if (abs(newLine[1] - newLine[3]) < lowHorThresh) {
            horLines.push_back((newLine[1] + newLine[3]) / 2);
        }
    }

    return horLines;
}

std::vector<std::vector<float>>
Gate::findVertPoles(std::vector<int> vertLines) {
    std::vector<std::vector<float>> vertPoles;
    vertPoles.clear();

    if (vertLines.empty()) { return vertPoles; }

    for (auto it1 = vertLines.begin(); it1 != vertLines.end(); it1++) {
        for (auto it2 = vertLines.begin(); it2 != vertLines.end();) {
            if ((abs(*it1 - *it2) < 10) && (*it1 != *it2)) {
                vertLines.erase(it2);
            } else {
                it2++;
            }
        }
    }

    for (auto it3 = vertLines.begin(); it3 != vertLines.end(); it3++) {
        for (auto it4 = vertLines.begin(); it4 != vertLines.end();) {
            if ((abs(*it3 - *it4) < poleMax) && (*it3 != *it4)) {
                float vertPoleDist = m * abs(*it3 - *it4) + b;
                float horizontalDistanceFromCentreToPole =
                ((camera_res_width / 2) - (*it3 + *it4) / 2) * (.3048 / 4) /
                abs(*it3 - *it4);
                float vertPoleAngle =
                asin(horizontalDistanceFromCentreToPole / vertPoleDist);
                float a       = ((*it3 + *it4) / 2);
                float array[] = {a, vertPoleAngle, vertPoleDist};
                std::vector<float> vertPoleVector(
                array, array + sizeof(array) / sizeof(array[0]));
                vertPoles.push_back(vertPoleVector);
                vertLines.erase(it4);
            } else {
                if (it4 != vertLines.end()) it4++;
            }
        }
    }

    return vertPoles;
}

std::vector<std::vector<float>> Gate::findHorPoles(std::vector<int> horLines) {
    std::vector<std::vector<float>> horPoles;
    horPoles.clear();

    if (horLines.empty()) { return horPoles; }

    for (auto it1 = horLines.begin(); it1 != horLines.end(); it1++) {
        for (auto it2 = horLines.begin(); it2 != horLines.end();) {
            if ((abs(*it1 - *it2) < 10) && (*it1 != *it2)) {
                horLines.erase(it2);
            } else {
                it2++;
            }
        }
    }

    for (auto it3 = horLines.begin(); it3 != horLines.end(); it3++) {
        for (auto it4 = horLines.begin(); it4 != horLines.end();) {
            if ((abs(*it3 - *it4) < poleMax) && (*it3 != *it4)) {
                // ROS_INFO("Line is a Pole at %f", (float) ((*it3 + *it4) /
                // 2));
                float horPoleDist = m * abs(*it3 - *it4) + b;
                float horizontalDistanceFromCentreToPole =
                ((camera_res_width / 2) - (*it3 + *it4) / 2) * (.3048 / 4) /
                abs(*it3 - *it4);
                float horPoleAngle =
                asin(horizontalDistanceFromCentreToPole / horPoleDist);
                float a       = ((*it3 + *it4) / 2);
                float array[] = {a, horPoleAngle, horPoleDist};
                std::vector<float> horPoleVector(
                array, array + sizeof(array) / sizeof(array[0]));
                horPoles.push_back(horPoleVector);
                horLines.erase(it4);
            } else {
                if (it4 != horLines.end()) it4++;
            }
        }
    }

    return horPoles;
}

std::vector<float> Gate::gateVector(std::vector<std::vector<float>> vertPoles,
                                    std::vector<std::vector<float>> horPoles) {
    std::vector<float> gateCoord(9, 0.0f);

    for (std::vector<float> horPole : horPoles) {
        if (gateCoord[6] == 0.0f || horPole[0] > gateCoord[6]) {
            gateCoord[6] = horPole[0];
            gateCoord[7] = horPole[1];
            gateCoord[8] = horPole[2];
        }
    }

    for (std::vector<float> vertPole : vertPoles) {
        if (!horPoles.empty()) {
            if (gateCoord[0] == 0.0f) {
                gateCoord[0] = vertPole[0];
                gateCoord[1] = vertPole[1];
                gateCoord[2] = vertPole[2];
            } else if (vertPole[0] <= gateCoord[0]) {
                gateCoord[3] = gateCoord[0];
                gateCoord[4] = gateCoord[1];
                gateCoord[5] = gateCoord[2];

                gateCoord[0] = vertPole[0];
                gateCoord[1] = vertPole[1];
                gateCoord[2] = vertPole[2];
            } else if (vertPole[0] >= gateCoord[3]) {
                gateCoord[3] = vertPole[0];
                gateCoord[4] = vertPole[1];
                gateCoord[5] = vertPole[2];
            }
        }
    }
    return gateCoord;
}
