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
                           int poleMax) {
    _cannyLow = cannyLow;

    _poleMax = poleMax;

    _houghLinesThreshold = houghLinesThreshold;

    _houghLinesMinLength = houghLinesMinLength;

    _houghLinesMaxLineGap = houghLinesMaxLineGap;

    _lowVertThresh = 20;

    _lowHorThresh = 60;

    _VertInterpolationConstant2 = _HorInterpolationConstant2 = 81.88;

    _VertInterpolationConstant1 = _HorInterpolationConstant1 = -.92791;
}

GateDetector::GateDetector() {
    _cannyLow = 50;

    _poleMax = 200;

    _houghLinesThreshold = 50;

    _houghLinesMinLength = 50;

    _houghLinesMaxLineGap = 50;

    _lowVertThresh = 20;

    _lowHorThresh = 60;

    _VertInterpolationConstant2 = _HorInterpolationConstant2 = 81.88;

    _VertInterpolationConstant1 = _HorInterpolationConstant1 = -.92791;
}

GateCoordinates GateDetector::initialize(const cv::Mat matin) {
    _imagePixelWidth = matin.cols;

    _imagePixelHeight = matin.rows;

    GateCoordinates gateCoordinates = defaultGateCoordinates();

    cv::Mat gaussianBlurred, medianBlurred;

    cv::GaussianBlur(matin, gaussianBlurred, cv::Size(7, 7), 0, 0);

    cv::medianBlur(gaussianBlurred, medianBlurred, 7);

    cv::Mat dst;

    cv::Canny(medianBlurred, dst, _cannyLow, _cannyLow * 3, 3);

    std::vector<cv::Vec4i> detectedLines;

    cv::HoughLinesP(dst,
                    detectedLines,
                    1,
                    CV_PI / 180,
                    _houghLinesThreshold,
                    _houghLinesMinLength,
                    _houghLinesMaxLineGap);

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
                     return abs((line1)[0] - (line1)[2]) < _lowVertThresh;
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
                     return abs((line1)[1] - (line1)[3]) < _lowHorThresh;
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
             (abs(vertLines.at(0)[0] - vertLines.at(1)[0]) < _poleMax)) {
        verticalPoles.push_back(Pole(vertLines.at(0),
                                     vertLines.at(1),
                                     _VertInterpolationConstant2,
                                     _VertInterpolationConstant1));

        return verticalPoles;
    }

    // Iterate through all lines, if they are close enough to be a pole delete
    // one of them and add them to Pole vector
    for (line1 = vertLines.begin(); line1 != vertLines.end(); line1++) {
        for (line2 = std::next(line1); line2 != vertLines.end();) {
            if ((abs((*line1)[0] - (*line2)[0]) < 2) || (*line1 == *line2)) {
                line2 = vertLines.erase(line2);

            } else if (abs((*line1)[0] - (*line2)[0]) < _poleMax) {
                Pole verticalPole = Pole(*line1,
                                         *line2,
                                         _HorInterpolationConstant2,
                                         _HorInterpolationConstant1);

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
             (abs(horLines.at(0)[0] - horLines.at(1)[0]) < _poleMax)) {
        horizontalPoles.push_back(Pole(horLines.at(0),
                                       horLines.at(1),
                                       _VertInterpolationConstant2,
                                       _VertInterpolationConstant1));

        return horizontalPoles;
    }

    std::vector<cv::Vec4i>::iterator line1;
    std::vector<cv::Vec4i>::iterator line2;

    if (horLines.size() == 2) {
        Pole verticalPole = Pole(*(horLines.begin()),
                                 *(horLines.begin()++),
                                 _HorInterpolationConstant2,
                                 _HorInterpolationConstant1);

        horizontalPoles.push_back(verticalPole);

        return horizontalPoles;
    }

    // Iterate through all lines, if they are close enough to be a pole delete
    // one of them and add them to Pole vector
    for (line1 = horLines.begin(); line1 != horLines.end(); line1++) {
        for (line2 = std::next(line1); line2 != horLines.end();) {
            if ((abs((*line1)[1] - (*line2)[1]) < 2) || (*line1 == *line2)) {
                line2 = horLines.erase(line2);

            } else if (abs((*line1)[1] - (*line2)[1]) < _poleMax) {
                Pole verticalPole = Pole(*line1,
                                         *line2,
                                         _HorInterpolationConstant2,
                                         _HorInterpolationConstant1);

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
    GateCoordinates gateCoordinates;

    // If theres no vertical poles, check for horizontal pole and to struct
    if (vertPoles.empty()) {
        if (!horPoles.empty()) {
            Pole topPole = *std::min_element(
            horPoles.begin(), horPoles.end(), [](Pole lhs, Pole rhs) {
                return lhs.getVertMid() > rhs.getVertMid();
            });

            gateCoordinates.detectedTop = 1;
            gateCoordinates.angleTop = topPole.getHorAngle(_imagePixelHeight);
            gateCoordinates.distanceTop = topPole.getHorDistance();
        }

    }
    // If theres one pole, check for horizontal poles and determine which
    // vertical pole it is
    else if (vertPoles.size() == 1) {
        if (horPoles.empty()) {
            Pole pole = *vertPoles.begin();

            gateCoordinates.detectedLeft = 1;
            gateCoordinates.angleLeft    = pole.getVertAngle(_imagePixelWidth);
            gateCoordinates.distanceLeft = pole.getVertDistance();

        } else {
            Pole vertPole = *vertPoles.begin();
            Pole horPole  = *horPoles.begin();

            gateCoordinates.detectedTop = 1;
            gateCoordinates.angleTop = horPole.getHorAngle(_imagePixelHeight);
            gateCoordinates.distanceTop = horPole.getHorDistance();

            if (horPole.getVertMid() > vertPole.getVertMid()) {
                gateCoordinates.detectedLeft = 1;
                gateCoordinates.angleLeft =
                vertPole.getVertAngle(_imagePixelWidth);
                gateCoordinates.distanceLeft = vertPole.getVertDistance();

            } else {
                gateCoordinates.detectedRight = 1;
                gateCoordinates.angleRight    = vertPole.getVertDistance();
                gateCoordinates.distanceRight =
                vertPole.getVertAngle(_imagePixelWidth);
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
                gateCoordinates.detectedLeft = 0;
                gateCoordinates.angleLeft    = 0;
                gateCoordinates.distanceLeft = 0;

                gateCoordinates.detectedRight = 1;
                gateCoordinates.angleRight =
                rightPole.getVertAngle(_imagePixelWidth);
                gateCoordinates.distanceRight = rightPole.getVertDistance();

                gateCoordinates.detectedTop = 1;
                gateCoordinates.angleTop =
                topPole.getHorAngle(_imagePixelHeight);
                gateCoordinates.distanceTop = topPole.getHorDistance();

            } else if (rightPole.getVertMid() < topPole.getVertMid()) {
                gateCoordinates.detectedLeft = 1;
                gateCoordinates.angleLeft =
                leftPole.getVertAngle(_imagePixelWidth);
                gateCoordinates.distanceLeft = leftPole.getVertDistance();

                gateCoordinates.detectedRight = 0;
                gateCoordinates.angleRight    = 0;
                gateCoordinates.distanceRight = 0;

                gateCoordinates.detectedTop = 1;
                gateCoordinates.angleTop =
                topPole.getHorAngle(_imagePixelHeight);
                gateCoordinates.distanceTop = topPole.getHorDistance();
            } else {
                gateCoordinates.detectedLeft = 1;
                gateCoordinates.angleLeft =
                leftPole.getVertAngle(_imagePixelWidth);
                gateCoordinates.distanceLeft = leftPole.getVertDistance();

                gateCoordinates.detectedRight = 1;
                gateCoordinates.angleRight =
                rightPole.getVertAngle(_imagePixelWidth);
                gateCoordinates.distanceRight = rightPole.getVertDistance();

                gateCoordinates.detectedTop = 1;
                gateCoordinates.angleTop =
                topPole.getHorAngle(_imagePixelHeight);
                gateCoordinates.distanceTop = topPole.getHorDistance();
            }

        } else {
            gateCoordinates.detectedLeft = 1;
            gateCoordinates.angleLeft = leftPole.getVertAngle(_imagePixelWidth);
            gateCoordinates.distanceLeft = leftPole.getVertDistance();

            gateCoordinates.detectedRight = 1;
            gateCoordinates.angleRight =
            rightPole.getVertAngle(_imagePixelWidth);
            gateCoordinates.distanceRight = rightPole.getVertDistance();
        }
    }
    return gateCoordinates;
}
