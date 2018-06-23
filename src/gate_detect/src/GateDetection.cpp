/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 * Description: Filter through image and determine if gate is in image and how
 * far away it is
 */

#include "GateDetection.h"

GateDetection::GateDetection(int cannyLow,
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

    m_Hor = m_Vert = 81.88;

    b_Hor = b_Vert = -.92791;
}

GateDetection::GateDetection() {
    _cannyLow = 50;

    _poleMax = 200;

    _houghLinesThreshold = 50;

    _houghLinesMinLength = 50;

    _houghLinesMaxLineGap = 50;

    _lowVertThresh = 20;

    _lowHorThresh = 60;

    m_Hor = m_Vert = 81.88;

    b_Hor = b_Vert = -.92791;
}

std::vector<float> GateDetection::initialize(const cv::Mat matin) {
    _imagePixelWidth = matin.cols;

    _imagePixelHeight = matin.rows;

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

    std::vector<float> gateVector = std::vector<float>(9, 0);

    if (detectedLines.empty()) return gateVector;

    std::vector<cv::Vec4i> vertLines = filterVertLines(detectedLines);

    std::vector<PoleDetection> vertPoles = findVertPoles(vertLines);

    std::vector<cv::Vec4i> horLines = filterHorLines(detectedLines);

    std::vector<PoleDetection> horPoles = findHorPoles(horLines);

    gateVector = getGateVector(vertPoles, horPoles);

    return gateVector;
}

std::vector<cv::Vec4i>
GateDetection::filterVertLines(std::vector<cv::Vec4i> detectedLines) {
    std::vector<cv::Vec4i> vertLines;

    std::copy_if(detectedLines.begin(),
                 detectedLines.end(),
                 std::back_inserter(vertLines),
                 [=](cv::Vec4i line1) {
                     return abs((line1)[0] - (line1)[2]) < _lowVertThresh;
                 });

    return vertLines;
}

std::vector<cv::Vec4i>
GateDetection::filterHorLines(std::vector<cv::Vec4i> detectedLines) {
    std::vector<cv::Vec4i> horLines;

    std::copy_if(detectedLines.begin(),
                 detectedLines.end(),
                 std::back_inserter(horLines),
                 [=](cv::Vec4i line1) {
                     return abs((line1)[1] - (line1)[3]) < _lowHorThresh;
                 });

    return horLines;
}

std::vector<PoleDetection>
GateDetection::findVertPoles(std::vector<cv::Vec4i> vertLines) {
    std::vector<PoleDetection> verticalPoles;

    std::vector<cv::Vec4i>::iterator line1;
    std::vector<cv::Vec4i>::iterator line2;

    if (vertLines.size() <= 1)
        return verticalPoles;

    else if (vertLines.size() == 2) {
        verticalPoles.push_back(
        PoleDetection(vertLines.at(0), vertLines.at(1), m_Hor, b_Hor));

        return verticalPoles;
    }

    for (line1 = vertLines.begin(); line1 != vertLines.end(); line1++) {
        for (line2 = std::next(line1); line2 != vertLines.end();) {
            if ((abs((*line1)[0] - (*line2)[0]) < 2) || (*line1 == *line2)) {
                line2 = vertLines.erase(line2);

            } else if (abs((*line1)[0] - (*line2)[0]) < _poleMax) {
                PoleDetection verticalPole =
                PoleDetection(*line1, *line2, m_Vert, b_Vert);

                verticalPoles.push_back(verticalPole);

                line2 = vertLines.erase(line2);

            } else {
                line2++;
            }
        }
    }

    return verticalPoles;
}

std::vector<PoleDetection>
GateDetection::findHorPoles(std::vector<cv::Vec4i> horLines) {
    std::vector<PoleDetection> horizontalPoles;

    if (horLines.size() <= 1)
        return horizontalPoles;

    else if (horLines.size() == 2) {
        horizontalPoles.push_back(
        PoleDetection(horLines.at(0), horLines.at(1), m_Hor, b_Hor));

        return horizontalPoles;
    }

    std::vector<cv::Vec4i>::iterator line1;
    std::vector<cv::Vec4i>::iterator line2;

    if (horLines.size() == 2) {
        PoleDetection verticalPole = PoleDetection(
        *(horLines.begin()), *(horLines.begin()++), m_Vert, b_Vert);

        horizontalPoles.push_back(verticalPole);

        return horizontalPoles;
    }

    for (line1 = horLines.begin(); line1 != horLines.end(); line1++) {
        for (line2 = std::next(line1); line2 != horLines.end();) {
            if ((abs((*line1)[1] - (*line2)[1]) < 2) || (*line1 == *line2)) {
                line2 = horLines.erase(line2);

            } else if (abs((*line1)[1] - (*line2)[1]) < _poleMax) {
                PoleDetection verticalPole =
                PoleDetection(*line1, *line2, m_Vert, b_Vert);

                horizontalPoles.push_back(verticalPole);

                line2 = horLines.erase(line2);

            } else {
                line2++;
            }
        }
    }

    return horizontalPoles;
}

std::vector<float>
GateDetection::getGateVector(std::vector<PoleDetection> vertPoles,
                             std::vector<PoleDetection> horPoles) {
    std::vector<float> gateCoord(9, 0.0f);

    if (vertPoles.empty()) {
        if (!horPoles.empty()) {
            PoleDetection topPole =
            *std::min_element(horPoles.begin(),
                              horPoles.end(),
                              [](PoleDetection lhs, PoleDetection rhs) {
                                  return lhs.getVertMid() > rhs.getVertMid();
                              });

            gateCoord[6] = 1;
            gateCoord[7] = topPole.getHorAngle(_imagePixelHeight);
            gateCoord[8] = topPole.getHorDistance();
        }

    } else if (vertPoles.size() == 1) {
        if (horPoles.empty()) {
            PoleDetection pole = *vertPoles.begin();

            gateCoord[0] = 1;
            gateCoord[1] = pole.getVertAngle(_imagePixelWidth);
            gateCoord[2] = pole.getVertDistance();

        } else {
            PoleDetection vertPole = *vertPoles.begin();
            PoleDetection horPole  = *horPoles.begin();

            gateCoord[6] = 1;
            gateCoord[7] = horPole.getHorAngle(_imagePixelHeight);
            gateCoord[8] = horPole.getHorDistance();

            if (horPole.getVertMid() > vertPole.getVertMid()) {
                gateCoord[0] = 1;
                gateCoord[1] = vertPole.getVertAngle(_imagePixelWidth);
                gateCoord[2] = vertPole.getVertDistance();

            } else {
                gateCoord[3] = 1;
                gateCoord[4] = vertPole.getVertDistance();
                gateCoord[5] = vertPole.getVertAngle(_imagePixelWidth);
            }
        }
    } else {
        PoleDetection leftPole =
        *std::min_element(vertPoles.begin(),
                          vertPoles.end(),
                          [](PoleDetection lhs, PoleDetection rhs) {
                              return lhs.getVertMid() < rhs.getVertMid();
                          });

        PoleDetection rightPole =
        *std::min_element(vertPoles.begin(),
                          vertPoles.end(),
                          [](PoleDetection lhs, PoleDetection rhs) {
                              return lhs.getVertMid() > rhs.getVertMid();
                          });

        if (!horPoles.empty()) {
            PoleDetection topPole =
            *std::min_element(horPoles.begin(),
                              horPoles.end(),
                              [](PoleDetection lhs, PoleDetection rhs) {
                                  return lhs.getHorMid() < rhs.getHorMid();
                              });

            if (leftPole.getVertMid() > topPole.getVertMid()) {
                gateCoord[0] = 0;
                gateCoord[1] = 0;
                gateCoord[2] = 0;

                gateCoord[3] = 1;
                gateCoord[4] = rightPole.getVertAngle(_imagePixelWidth);
                gateCoord[5] = rightPole.getVertDistance();

                gateCoord[6] = 1;
                gateCoord[7] = topPole.getHorAngle(_imagePixelHeight);
                gateCoord[8] = topPole.getHorDistance();

            } else if (rightPole.getVertMid() < topPole.getVertMid()) {
                gateCoord[0] = 1;
                gateCoord[1] = leftPole.getVertAngle(_imagePixelWidth);
                gateCoord[2] = leftPole.getVertDistance();

                gateCoord[3] = 0;
                gateCoord[4] = 0;
                gateCoord[5] = 0;

                gateCoord[6] = 1;
                gateCoord[7] = topPole.getHorAngle(_imagePixelHeight);
                gateCoord[8] = topPole.getHorDistance();
            } else {
                gateCoord[0] = 1;
                gateCoord[1] = leftPole.getVertAngle(_imagePixelWidth);
                gateCoord[2] = leftPole.getVertDistance();

                gateCoord[3] = 1;
                gateCoord[4] = rightPole.getVertAngle(_imagePixelWidth);
                gateCoord[5] = rightPole.getVertDistance();

                gateCoord[6] = 1;
                gateCoord[7] = topPole.getHorAngle(_imagePixelHeight);
                gateCoord[8] = topPole.getHorDistance();
            }

        } else {
            gateCoord[0] = 1;
            gateCoord[1] = leftPole.getVertAngle(_imagePixelWidth);
            gateCoord[2] = leftPole.getVertDistance();

            gateCoord[3] = 1;
            gateCoord[4] = rightPole.getVertAngle(_imagePixelWidth);
            gateCoord[5] = rightPole.getVertDistance();
        }
    }
    return gateCoord;
}
