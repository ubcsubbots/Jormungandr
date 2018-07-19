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
                           double interpolationConstant1,
                           double interpolationConstant2) {
    cannyLow_ = cannyLow;

    poleMax_ = poleMax;

    houghLinesThreshold_ = houghLinesThreshold;

    houghLinesMinLength_ = houghLinesMinLength;

    houghLinesMaxLineGap_ = houghLinesMaxLineGap;

    lowVertThresh_ = 20;

    lowHorThresh_ = 60;

    VertInterpolationConstant1_ = HorInterpolationConstant1_ =
    interpolationConstant1;

    VertInterpolationConstant2_ = HorInterpolationConstant2_ =
    interpolationConstant2;
}

GateDetector::GateDetector() {
    cannyLow_ = 50;

    poleMax_ = 200;

    houghLinesThreshold_ = 50;

    houghLinesMinLength_ = 50;

    houghLinesMaxLineGap_ = 50;

    lowVertThresh_ = 20;

    lowHorThresh_ = 60;

    VertInterpolationConstant2_ = HorInterpolationConstant2_ = 81.88;

    VertInterpolationConstant1_ = HorInterpolationConstant1_ = -.92791;
}

void GateDetector::setParams(int cannyLow,
                             int houghLinesThreshold,
                             int houghLinesMinLength,
                             int houghLinesMaxLineGap,
                             int poleMax,
                             float interpolationConstant1,
                             float interpolationConstant2) {
    cannyLow_ = cannyLow;

    poleMax_ = poleMax;

    houghLinesThreshold_ = houghLinesThreshold;

    houghLinesMinLength_ = houghLinesMinLength;

    houghLinesMaxLineGap_ = houghLinesMaxLineGap;

    lowVertThresh_ = 20;

    lowHorThresh_ = 60;

    VertInterpolationConstant1_ = HorInterpolationConstant1_ =
    interpolationConstant1;

    VertInterpolationConstant2_ = HorInterpolationConstant2_ =
    interpolationConstant2;
}

GateCoordinates GateDetector::initialize(const cv::Mat matin) {
    imagePixelWidth_ = matin.cols;

    imagePixelHeight_ = matin.rows;

    GateCoordinates gateCoordinates = defaultGateCoordinates();

    cv::Mat gaussianBlurred, medianBlurred;

    cv::GaussianBlur(matin, gaussianBlurred, cv::Size(7, 7), 0, 0);

    cv::medianBlur(gaussianBlurred, medianBlurred, 7);

    cv::Mat dst;

    cv::Canny(medianBlurred, dst, cannyLow_, cannyLow_ * 3, 3);

    std::vector<cv::Vec4i> detectedLines;

    cv::HoughLinesP(dst,
                    detectedLines,
                    1,
                    CV_PI / 180,
                    houghLinesThreshold_,
                    houghLinesMinLength_,
                    houghLinesMaxLineGap_);

    if (detectedLines.empty()) return gateCoordinates;

    std::vector<cv::Vec4i> vertLines = filterVertLines(detectedLines);

    std::vector<Pole> vertPoles = findVertPoles(vertLines);

    std::vector<cv::Vec4i> horLines = filterHorLines(detectedLines);

    std::vector<Pole> horPoles = findHorPoles(horLines);

    gateCoordinates = getGateCoordinates(vertPoles, horPoles);

    return gateCoordinates;
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
        verticalPoles.push_back(Pole(vertLines.at(0),
                                     vertLines.at(1),
                                     (float) VertInterpolationConstant2_,
                                     (float) VertInterpolationConstant1_));

        return verticalPoles;
    }

    // Iterate through all lines, if they are close enough to be a pole delete
    // one of them and add them to Pole vector
    for (line1 = vertLines.begin(); line1 != vertLines.end(); line1++) {
        for (line2 = std::next(line1); line2 != vertLines.end();) {
            if ((abs((*line1)[0] - (*line2)[0]) < 2) || (*line1 == *line2)) {
                line2 = vertLines.erase(line2);

            } else if (abs((*line1)[0] - (*line2)[0]) < poleMax_) {
                Pole verticalPole = Pole(*line1,
                                         *line2,
                                         (float) HorInterpolationConstant2_,
                                         (float) HorInterpolationConstant1_);

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
        horizontalPoles.push_back(Pole(horLines.at(0),
                                       horLines.at(1),
                                       (float) VertInterpolationConstant2_,
                                       (float) VertInterpolationConstant1_));

        return horizontalPoles;
    }

    std::vector<cv::Vec4i>::iterator line1;
    std::vector<cv::Vec4i>::iterator line2;

    if (horLines.size() == 2) {
        Pole verticalPole = Pole(*(horLines.begin()),
                                 *(horLines.begin()++),
                                 (float) HorInterpolationConstant2_,
                                 (float) HorInterpolationConstant1_);

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
                Pole verticalPole = Pole(*line1,
                                         *line2,
                                         (float) HorInterpolationConstant2_,
                                         (float) HorInterpolationConstant1_);

                horizontalPoles.push_back(verticalPole);

                line2 = horLines.erase(line2);

            } else {
                line2++;
            }
        }
    }

    return horizontalPoles;
}

GateCoordinates GateDetector::getGateCoordinates(std::vector<Pole> vertPoles,
                                                 std::vector<Pole> horPoles) {
    GateCoordinates gateCoordinates = defaultGateCoordinates();

    // If theres no vertical poles, check for horizontal pole and to struct
    if (vertPoles.empty()) {
        if (!horPoles.empty()) {
            Pole topPole = *std::min_element(
            horPoles.begin(), horPoles.end(), [](Pole lhs, Pole rhs) {
                return lhs.getVertMid() > rhs.getVertMid();
            });

            gateCoordinates.detectedTopPole = 1;
            gateCoordinates.angleTopPole =
            topPole.getHorAngle(imagePixelHeight_);
            gateCoordinates.distanceTopPole = topPole.getHorDistance();
        }
    }
    // If theres one pole, check for horizontal poles and determine which
    // vertical pole it is
    else if (vertPoles.size() == 1) {
        if (horPoles.empty()) {
            Pole pole = *vertPoles.begin();

            gateCoordinates.detectedLeftPole = 1;
            gateCoordinates.angleLeftPole = pole.getVertAngle(imagePixelWidth_);
            gateCoordinates.distanceLeftPole = pole.getVertDistance();

        } else {
            Pole vertPole = *vertPoles.begin();
            Pole horPole  = *horPoles.begin();

            gateCoordinates.detectedTopPole = 1;
            gateCoordinates.angleTopPole =
            horPole.getHorAngle(imagePixelHeight_);
            gateCoordinates.distanceTopPole = horPole.getHorDistance();

            if (horPole.getVertMid() > vertPole.getVertMid()) {
                gateCoordinates.detectedLeftPole = 1;
                gateCoordinates.angleLeftPole =
                vertPole.getVertAngle(imagePixelWidth_);
                gateCoordinates.distanceLeftPole = vertPole.getVertDistance();

            } else {
                gateCoordinates.detectedRightPole = 1;
                gateCoordinates.angleRightPole    = vertPole.getVertDistance();
                gateCoordinates.distanceRightPole =
                vertPole.getVertAngle(imagePixelWidth_);
            }
        }
    }
    // Find pole furthest to left and furthest to right, determine which pole is
    // which and return the struct that is most likely to define the gate
    else {
        Pole leftPole = *std::min_element(
        vertPoles.begin(), vertPoles.end(), [](Pole lhs, Pole rhs) {
            return lhs.getVertMid() < rhs.getVertMid();
        });

        Pole rightPole = *std::min_element(
        vertPoles.begin(), vertPoles.end(), [](Pole lhs, Pole rhs) {
            return lhs.getVertMid() > rhs.getVertMid();
        });

        if (!horPoles.empty()) {
            Pole topPole = *std::min_element(
            horPoles.begin(), horPoles.end(), [](Pole lhs, Pole rhs) {
                return lhs.getHorMid() < rhs.getHorMid();
            });

            if (leftPole.getVertMid() > topPole.getVertMid()) {
                gateCoordinates.detectedLeftPole = 0;
                gateCoordinates.angleLeftPole    = 0;
                gateCoordinates.distanceLeftPole = 0;

                gateCoordinates.detectedRightPole = 1;
                gateCoordinates.angleRightPole =
                rightPole.getVertAngle(imagePixelWidth_);
                gateCoordinates.distanceRightPole = rightPole.getVertDistance();

                gateCoordinates.detectedTopPole = 1;
                gateCoordinates.angleTopPole =
                topPole.getHorAngle(imagePixelHeight_);
                gateCoordinates.distanceTopPole = topPole.getHorDistance();

            } else if (rightPole.getVertMid() < topPole.getVertMid()) {
                gateCoordinates.detectedLeftPole = 1;
                gateCoordinates.angleLeftPole =
                leftPole.getVertAngle(imagePixelWidth_);
                gateCoordinates.distanceLeftPole = leftPole.getVertDistance();

                gateCoordinates.detectedRightPole = 0;
                gateCoordinates.angleRightPole    = 0;
                gateCoordinates.distanceRightPole = 0;

                gateCoordinates.detectedTopPole = 1;
                gateCoordinates.angleTopPole =
                topPole.getHorAngle(imagePixelHeight_);
                gateCoordinates.distanceTopPole = topPole.getHorDistance();
            } else {
                gateCoordinates.detectedLeftPole = 1;
                gateCoordinates.angleLeftPole =
                leftPole.getVertAngle(imagePixelWidth_);
                gateCoordinates.distanceLeftPole = leftPole.getVertDistance();

                gateCoordinates.detectedRightPole = 1;
                gateCoordinates.angleRightPole =
                rightPole.getVertAngle(imagePixelWidth_);
                gateCoordinates.distanceRightPole = rightPole.getVertDistance();

                gateCoordinates.detectedTopPole = 1;
                gateCoordinates.angleTopPole =
                topPole.getHorAngle(imagePixelHeight_);
                gateCoordinates.distanceTopPole = topPole.getHorDistance();
            }

        } else {
            gateCoordinates.detectedLeftPole = 1;
            gateCoordinates.angleLeftPole =
            leftPole.getVertAngle(imagePixelWidth_);
            gateCoordinates.distanceLeftPole = leftPole.getVertDistance();

            gateCoordinates.detectedRightPole = 1;
            gateCoordinates.angleRightPole =
            rightPole.getVertAngle(imagePixelWidth_);
            gateCoordinates.distanceRightPole = rightPole.getVertDistance();
        }
    }

    if (gateCoordinates.distanceTopPole > 15) {
        gateCoordinates.detectedTopPole = 0;
        gateCoordinates.distanceTopPole = 0;
        gateCoordinates.angleTopPole    = 0;
    }

    return gateCoordinates;
}
