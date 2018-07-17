/*
 * Created By: Cameron Newton
 * Created On: July 17, 2018
 * Detect line in image
 */

#include "LineDetector.h"

LineDetector::LineDetector() {
    cannyLow_ = 50;

    maxLineWidth_ = 200;

    houghLinesThreshold_ = 50;

    houghLinesMinLength_ = 50;

    houghLinesMaxLineGap_ = 50;

}


LineStruct LineDetector::initialize(const cv::Mat mat_in){
    imagePixelWidth_ = mat_in.cols;
    imagePixelHeight_ = mat_in.rows;

    LineStruct lineStruct = defaultLineStruct();

    cv::Mat gaussianBlurred, medianBlurred;
    cv::GaussianBlur(mat_in, gaussianBlurred, cv::Size(7, 7), 0, 0);
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

    std::vector<std::pair<cv::Vec4i,cv::Vec4i>> detectedMarkerLines = findMarker(detectedLines);

    if(detectedMarkerLines.empty()) return lineStruct;

    std::pair<cv::Vec4i, cv::Vec4i> markerLines;

    if(detectedMarkerLines.size() > 1) {
        markerLines = *std::max_element(detectedMarkerLines.begin(), detectedMarkerLines.end(),
                                                                               [](std::pair<cv::Vec4i, cv::Vec4i> lhs,
                                                                                  std::pair<cv::Vec4i, cv::Vec4i> rhs) {
                                                                                   return (((lhs.first + lhs.second) /
                                                                                            2)[1] <
                                                                                           ((rhs.first + rhs.second) /
                                                                                            2)[1]);
                                                                               });

    }else markerLines = detectedMarkerLines.at(0);


    cv::Vec4i averageOfLines = (markerLines.first + markerLines.second)/2;

    lineStruct.angleToParallel = (calculateSlope(markerLines.first) + calculateSlope(markerLines.second))/2;

    lineStruct.distanceFromLine = calcProjectedDistance(averageOfLines);

    lineStruct.distanceFromEnd = calcProjectedDistanceToEndOfLine(averageOfLines);

    return lineStruct;
}


std::vector<std::pair<cv::Vec4i,cv::Vec4i>> LineDetector::findMarker(std::vector<cv::Vec4i> allDetectedLines){
    std::vector<std::pair<cv::Vec4i,cv::Vec4i>> markerLines;

    std::vector<cv::Vec4i>::iterator it1, it2;

    it1 = allDetectedLines.begin();

    it2 = allDetectedLines.begin()++;

    for(it1;it1 != allDetectedLines.end();it1++){

        for(it2; it2 != allDetectedLines.end() ; it2++){

            if(((calculateSlope(*it1) - calculateSlope(*it2)) < 0.05) && (calculateWidth(*it1,*it2) - maxLineWidth_) < 10)

                markerLines.push_back(std::pair<cv::Vec4i,cv::Vec4i>(*it1,*it2));
        }
    }

    return markerLines;
}


float LineDetector::calculateSlope(cv::Vec4i detectedLine){
    return(detectedLine[3] - detectedLine[1])/(detectedLine[2] - detectedLine[0]);
}

float LineDetector::calculateWidth(cv::Vec4i line1 , cv::Vec4i line2){
    float m0,m1,b1,xi0,yi0,xi1,yi1, mi, bi;

    m0 = calculateSlope(line1);

    m1 = calculateSlope(line2);

    b1 = line2[1] - m0 * line2[0];

    xi0 = (line1[0] + line1[2]) / 2;

    yi0 = (line1[1] + line1[3]) / 2;

    mi = -(1/m0);

    bi = yi0 - mi * xi0;

    xi1 = (b1 - bi)/(mi - m1);

    yi1 = mi * xi1 + bi;

    return sqrt(pow((xi1 - xi0),2) + pow((yi1 - yi0),2));
}

float LineDetector::calcProjectedDistance(cv::Vec4i line1) {
    float x0 = line1[0], y0 = line1[1], x1 = line1[2], y1 = line1[3];

    float m0,b0,m1,b1;

    if(x1 - x0 == 0) return -1;

    m0 = calculateSlope(line1);

    b0 = y0 - m1 * x0;

    m1 = -(1/m0);

    b1 = (imagePixelHeight_/2) - m1 * (imagePixelWidth_/2);

    float xi = (b1 - b0)/(m1 - m0);

    float yi = m1 * xi + b1;

    return sqrt(pow((yi - (imagePixelHeight_/2)),2) + pow((xi - imagePixelWidth_/2),2));
}

float LineDetector::calcProjectedDistanceToEndOfLine(cv::Vec4i line1){

    return sqrt(pow((line1[0] - line1[2])/2,2) + pow((line1[1] - line1[3])/2,2));
}

