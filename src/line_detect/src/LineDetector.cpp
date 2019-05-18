/*
 * Created By: Cameron Newton
 * Created On: July 17, 2018
 * Detect line in image
 */

#include "LineDetector.h"
#include <iostream>

LineDetector::LineDetector() {
    cannyLow_ = 50;

    houghLinesThreshold_ = 50;

    houghLinesMinLength_ = 50;

    houghLinesMaxLineGap_ = 50;
}

LinesToFollow LineDetector::initialize(const cv::Mat mat_in) {
    // Blur image and apply Canny edge detector and HoughLines
    cv::Mat medianBlurred;
    cv::medianBlur(mat_in, medianBlurred, 7);
    cv::Mat dst, contours;
    cv::Canny(medianBlurred, dst, cannyLow_, cannyLow_ * 3, 3);

    imagePixelWidth_  = dst.cols;
    imagePixelHeight_ = dst.rows;

    LinesToFollow linesToFollow = defaultLinesToFollow();

    // Detect lines in image
    //std::vector<cv::Vec4i> detectedLines;
    cv::HoughLinesP(dst,
                    detectedLines,
                    1,
                    CV_PI / 360,
                    houghLinesThreshold_,
                    houghLinesMinLength_,
                    houghLinesMaxLineGap_);

    // Detect markers in image
    std::vector<LineToFollow> markers = findMarkers(detectedLines);

    if (markers.empty()) return linesToFollow;

    // Filter through lines to get marker in top half of vision
    linesToFollow.frontLine = *std::max_element(
    markers.begin(), markers.end(), [&](LineToFollow lhs, LineToFollow rhs) {
        if (lhs.frontOfMarker < (imagePixelHeight_ / 2)) {
            return (lhs.frontOfMarker > rhs.frontOfMarker);
        }
    });

    // Filter through lines to get marker in bottom half of vision
    linesToFollow.rearLine = *std::min_element(
    markers.begin(), markers.end(), [&](LineToFollow lhs, LineToFollow rhs) {
        if ((lhs.frontOfMarker > (imagePixelHeight_ / 2))) {
            return (lhs.frontOfMarker < rhs.frontOfMarker);
        }
    });

    return linesToFollow;
}

std::vector<LineToFollow>
LineDetector::findMarkers(std::vector<cv::Vec4i> allDetectedLines) {
    std::vector<cv::Vec4i>::iterator it1, it2;

    std::vector<LineToFollow> markers;

    it1 = allDetectedLines.begin();

    for (it1; it1 != allDetectedLines.end(); it1++) {
        for (it2 = it1 + 1; it2 != allDetectedLines.end();) {
            // Calculate width of lines, only return lines that are between 10
            // and 300 pixels apart

            float width = calculateWidth(*it1, *it2);

            if ((width < 300) && (width > 10)) {
                LineToFollow lineToFollow;
                lineToFollow.slope =
                (calculateSlope(*it1) + calculateSlope(*it2)) / 2;
                lineToFollow.width = width;
                int ints[] = {(*it1)[1], (*it1)[3], (*it2)[1], (*it2)[3]};
                lineToFollow.frontOfMarker  = *std::min_element(ints, ints + 4);
                lineToFollow.middleOfMarker = ((*it1) + (*it2)) / 2;

                markers.push_back(lineToFollow);
                allDetectedLines.erase(it2);
            } else
                it2++;
        }
    }

    return markers;
}

float LineDetector::calculateSlope(cv::Vec4i detectedLine) {
    if (detectedLine[3] == detectedLine[1])
        return 3.1415f / 2.0f;

    else
        return ((float) (detectedLine[2] - detectedLine[0])) /
               (float) (detectedLine[3] - detectedLine[1]);
}

float LineDetector::calculateWidth(cv::Vec4i line1, cv::Vec4i line2) {
    // If slopes are not similar return -1
    float slope1 = calculateSlope(line1), slope2 = calculateSlope(line2);
    if ((std::abs((slope1 - slope2) > 0.05)))
        return -1;
    else
        return abs(((line1[0] + line1[2] - line2[0] - line2[2]) / 2));
}

float LineDetector::calcProjectedDistance(cv::Vec4i line1,
                                          float pixelWidthOfMarker) {
    return (((line1[0] + line1[2]) / 2.0f) - (imagePixelWidth_ / 2.0f)) *
           (0.5f / pixelWidthOfMarker) * (.3048f);
}

float LineDetector::calcProjectedDistanceToEndOfLine(
float pixelOfForwardMostPoint, float pixelWidthOfMarker) {
    return (((imagePixelHeight_ / 2.0f) - pixelOfForwardMostPoint) *
            (0.5f / pixelWidthOfMarker) * (.3048f));
}
