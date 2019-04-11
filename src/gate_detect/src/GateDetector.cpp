/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 * Description: Filter through image and determine if gate is in image and how
 * far away it is
 */

#include "GateDetector.h"

GateDetector::GateDetector(int cannyLow,
                           int houghLinesThreshold,
                           int houghLinesMinLength,
                           int houghLinesMaxLineGap,
                           int poleMax,
                           int lowVertThresh,
                           int lowHorThresh) {
    cannyLow_ = cannyLow;

    poleMax_ = poleMax;

    houghLinesThreshold_ = houghLinesThreshold;

    houghLinesMinLength_ = houghLinesMinLength;

    houghLinesMaxLineGap_ = houghLinesMaxLineGap;

    lowVertThresh_ = lowVertThresh;

    lowHorThresh_ = lowHorThresh;
}

GateDetector::GateDetector() {
    cannyLow_ = 50;

    poleMax_ = 200;

    houghLinesThreshold_ = 50;

    houghLinesMinLength_ = 50;

    houghLinesMaxLineGap_ = 50;

    lowVertThresh_ = 100;

    lowHorThresh_ = 200;
}

void GateDetector::setParams(int cannyLow,
                             int houghLinesThreshold,
                             int houghLinesMinLength,
                             int houghLinesMaxLineGap,
                             int poleMax,
                             int lowVertThresh,
                             int lowHorThresh) {
    cannyLow_ = cannyLow;

    poleMax_ = poleMax;

    houghLinesThreshold_ = houghLinesThreshold;

    houghLinesMinLength_ = houghLinesMinLength;

    houghLinesMaxLineGap_ = houghLinesMaxLineGap;

    lowVertThresh_ = lowVertThresh;

    lowHorThresh_ = lowHorThresh;

}

Gate GateDetector::initialize(const cv::Mat matin) {
    imagePixelWidth_ = matin.cols;

    imagePixelHeight_ = matin.rows;

    cv::Mat dst;

    cv::Canny(matin, dst, cannyLow_, cannyLow_ * 3, 3);

    std::vector<cv::Vec4i> detectedLines;

    cv::HoughLinesP(dst,
                    detectedLines,
                    1,
                    CV_PI / 180,
                    houghLinesThreshold_,
                    houghLinesMinLength_,
                    houghLinesMaxLineGap_);
    Gate gate = Gate();

    if (detectedLines.empty()) return gate;

    std::vector<cv::Vec4i> vertLines = filterVertLines(detectedLines);

    std::vector<Pole> vertPoles = findVertPoles(vertLines);

    std::vector<cv::Vec4i> horLines = filterHorLines(detectedLines);

    std::vector<Pole> horPoles = findHorPoles(horLines);

    gate = getGate(vertPoles, horPoles);

    return gate;
}

std::vector<cv::Vec4i>
GateDetector::filterVertLines(std::vector<cv::Vec4i> detectedLines) {
    std::vector<cv::Vec4i> vertLines;

    // Filter out lines that are vertical
    std::copy_if(detectedLines.begin(),
                 detectedLines.end(),
                 std::back_inserter(vertLines),
                 [=](cv::Vec4i line1) {
                     return abs((line1)[0] - (line1)[2]) < lowVertThresh_;
                 });

    return vertLines;
}

std::vector<cv::Vec4i>
GateDetector::filterHorLines(std::vector<cv::Vec4i> detectedLines) {
    std::vector<cv::Vec4i> horLines;

    // Filter out lines that are horizontal
    std::copy_if(detectedLines.begin(),
                 detectedLines.end(),
                 std::back_inserter(horLines),
                 [=](cv::Vec4i line1) {
                     return abs((line1)[1] - (line1)[3]) < lowHorThresh_;
                 });

    return horLines;
}

std::vector<Pole>
GateDetector::findVertPoles(std::vector<cv::Vec4i> vertLines) {
    std::vector<Pole> verticalPoles;

    std::vector<cv::Vec4i>::iterator line1;
    std::vector<cv::Vec4i>::iterator line2;

    // If onle 1 line, return empty vector
    if (vertLines.size() <= 1) return verticalPoles;

    // If theres 2 lines and they are close enough to be a pole, add them to
    // Pole vector and return them
    else if ((vertLines.size() == 2) &&
             (abs(vertLines.at(0)[0] - vertLines.at(1)[0]) < poleMax_)) {
        verticalPoles.push_back(Pole(vertLines.at(0), vertLines.at(1)));

        return verticalPoles;
    }

    // Iterate through all lines, if they are close enough to be a pole delete
    // one of them and add them to Pole vector
    for (line1 = vertLines.begin(); line1 != vertLines.end(); line1++) {
        for (line2 = std::next(line1); line2 != vertLines.end();) {
            if ((abs((*line1)[0] - (*line2)[0]) < 2) || (*line1 == *line2)) {
                line2 = vertLines.erase(line2);

            } else if (abs((*line1)[0] - (*line2)[0]) < poleMax_) {
                Pole verticalPole = Pole(*line1, *line2);

                verticalPoles.push_back(verticalPole);

                line2 = vertLines.erase(line2);

            } else {
                line2++;
            }
        }
    }

    return verticalPoles;
}

std::vector<Pole> GateDetector::findHorPoles(std::vector<cv::Vec4i> horLines) {
    std::vector<Pole> horizontalPoles;

    // If onle 1 line, return empty vector
    if (horLines.size() <= 1) return horizontalPoles;

    // If theres 2 lines and they are close enough to be a pole, add them to
    // Pole vector and return them
    else if ((horLines.size() == 2) &&
             (abs(horLines.at(0)[0] - horLines.at(1)[0]) < poleMax_)) {
        horizontalPoles.push_back(Pole(horLines.at(0), horLines.at(1)));

        return horizontalPoles;
    }

    std::vector<cv::Vec4i>::iterator line1;
    std::vector<cv::Vec4i>::iterator line2;

    if (horLines.size() == 2) {
        Pole verticalPole = Pole(*(horLines.begin()), *(horLines.begin()++));

        horizontalPoles.push_back(verticalPole);

        return horizontalPoles;
    }

    // Iterate through all lines, if they are close enough to be a pole delete
    // one of them and add them to Pole vector
    for (line1 = horLines.begin(); line1 != horLines.end(); line1++) {
        for (line2 = std::next(line1); line2 != horLines.end();) {
            if ((abs((*line1)[1] - (*line2)[1]) < 2) || (*line1 == *line2)) {
                line2 = horLines.erase(line2);

            } else if (abs((*line1)[1] - (*line2)[1]) < poleMax_) {
                Pole verticalPole = Pole(*line1, *line2);

                horizontalPoles.push_back(verticalPole);

                line2 = horLines.erase(line2);

            } else {
                line2++;
            }
        }
    }

    return horizontalPoles;
}

Gate GateDetector::getGate(std::vector<Pole> vertPoles,
                           std::vector<Pole> horPoles) {
    Gate gate;

    // If theres no vertical poles, check for horizontal pole and put in struct
    if (vertPoles.empty()) {
        if (!horPoles.empty()) {
            gate.topPole = *std::min_element(
            horPoles.begin(), horPoles.end(), [](Pole lhs, Pole rhs) {
                return lhs.getHorMid() > rhs.getHorMid(); // changed from vert
                                                          // to hor, might need
                                                          // testing
            });

            gate.topDetected = true;
        }
    }
    // If theres one pole, check for horizontal poles and determine which
    // vertical pole it is
    else if (vertPoles.size() == 1) {
        if (horPoles.empty()) {
            gate.leftPole     = *vertPoles.begin();
            gate.leftDetected = true;

        }

        else {
            Pole vertPole = *vertPoles.begin();
            gate.topPole     = *horPoles.begin();
            gate.topDetected = true;

            if (gate.topPole.getVertMid() > vertPole.getVertMid()) {
                gate.leftPole     = vertPole;
                gate.leftDetected = true;
            }

            else {
                gate.rightPole     = vertPole;
                gate.rightDetected = true;
            }
        }
    }
    // Find pole furthest to left and furthest to right, determine which pole is
    // which and return the struct that is most likely to define the gate
    else {
        gate.leftPole = *std::min_element(
        vertPoles.begin(), vertPoles.end(), [](Pole lhs, Pole rhs) {
            return lhs.getVertMid() < rhs.getVertMid();
        });

        gate.rightPole = *std::min_element(
        vertPoles.begin(), vertPoles.end(), [](Pole lhs, Pole rhs) {
            return lhs.getVertMid() > rhs.getVertMid();
        });

        if (!horPoles.empty()) {
            gate.topPole = *std::min_element(
            horPoles.begin(), horPoles.end(), [](Pole lhs, Pole rhs) {
                return lhs.getHorMid() < rhs.getHorMid();
            });

            if (gate.leftPole.getVertMid() > gate.topPole.getVertMid()) {
                gate.leftDetected = false;

                gate.rightDetected = true;

                gate.topDetected = true;

            }

            else if (gate.rightPole.getVertMid() < gate.topPole.getVertMid()) {
                gate.leftDetected = true;

                gate.rightDetected = false;

                gate.topDetected = true;

            }

            else {
                gate.leftDetected = true;

                gate.rightDetected = true;

                gate.topDetected = true;
            }

        } else {
            gate.leftDetected = true;

            gate.rightDetected = true;
        }
    }

    return gate;
}
