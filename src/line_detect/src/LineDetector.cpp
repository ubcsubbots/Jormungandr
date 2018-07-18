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
    //cv::GaussianBlur(mat_in, gaussianBlurred, cv::Size(7, 7), 0, 0);
    cv::medianBlur(mat_in, medianBlurred, 7);
    cv::Mat dst;
    cv::Canny(medianBlurred, dst, cannyLow_, cannyLow_ * 3, 3);

    std::vector<cv::Vec4i> detectedLines;
    cv::HoughLinesP(dst,
                    detectedLines,
                    1,
                    CV_PI / 360,
                    100,
                    150,
                    100);

    std::vector<MarkerStruct> markers= findMarkers(detectedLines);;

    if(markers.empty()) return lineStruct;

    cv::Mat cdst;

    cv::cvtColor(dst,cdst,CV_GRAY2BGR);

    if(markers.size() == 1){
        MarkerStruct marker = markers.at(0);

        lineStruct.angleToParallelFrontMarker = marker.slope;

        lineStruct.lateralDistanceFromFrontMarker = calcProjectedDistance(marker.middleOfMarker,marker.width);

        lineStruct.distanceFromEndFrontMarker = calcProjectedDistanceToEndOfLine(marker.frontOfMarker, marker.width);
    } else if(markers.size() > 1) {
        std::sort(markers.begin(),markers.end(), [](MarkerStruct lhs, MarkerStruct rhs){
            return (lhs.frontOfMarker < rhs.frontOfMarker);});

        MarkerStruct frontMarker = markers.at(0);

        markers.erase(std::remove_if(markers.begin(),markers.end(),[frontMarker](const MarkerStruct lhs){
            return (lhs.slope - frontMarker.slope) < 0.01;}),markers.end());

        if(!markers.empty()){
            std::sort(markers.begin(),markers.end(), [](MarkerStruct lhs, MarkerStruct rhs){
                return (lhs.frontOfMarker < rhs.frontOfMarker);});

            MarkerStruct rearMarker = markers.at(0);

            lineStruct.angleToParallelRearMarker = rearMarker.slope;

            lineStruct.lateralDistanceFromRearMarker = calcProjectedDistance(rearMarker.middleOfMarker,rearMarker.width);

            lineStruct.distanceFromEndRearMarker = calcProjectedDistanceToEndOfLine(rearMarker.frontOfMarker, rearMarker.width);

            cv::line(cdst, cv::Point(imagePixelWidth_/2,imagePixelHeight_/2),cv::Point(rearMarker.middleOfMarker[2],rearMarker.middleOfMarker[3]),cv::Scalar(0,0,255),1);
        }

        lineStruct.angleToParallelFrontMarker = frontMarker.slope;

        lineStruct.lateralDistanceFromFrontMarker = calcProjectedDistance(frontMarker.middleOfMarker,frontMarker.width);

        lineStruct.distanceFromEndFrontMarker = calcProjectedDistanceToEndOfLine(frontMarker.frontOfMarker, frontMarker.width);

        cv::line(cdst, cv::Point(imagePixelWidth_/2,imagePixelHeight_/2),cv::Point(frontMarker.middleOfMarker[2],frontMarker.middleOfMarker[3]),cv::Scalar(0,0,255),1);
    }

    return lineStruct;
}


std::vector<MarkerStruct> LineDetector::findMarkers(std::vector<cv::Vec4i> allDetectedLines){
    std::vector<cv::Vec4i>::iterator it1, it2;

    std::vector<MarkerStruct> markers;

    it1 = allDetectedLines.begin();

    for(it1; it1 != allDetectedLines.end(); it1++){

        for(it2 = it1 + 1; it2 != allDetectedLines.end() ; ) {

            if (((calculateSlope(*it1) - calculateSlope(*it2)) < 0.05)){
                int width = calculateWidth(*it1, *it2);
                if ((width < 300) && (width > 10)) {
                    MarkerStruct marker;
                    marker.slope = (calculateSlope(*it1) + calculateSlope(*it2)) / 2;
                    marker.width = width;
                    int ints[] = {(*it1)[1], (*it1)[3], (*it2)[1], (*it2)[3]};
                    marker.frontOfMarker = *std::min_element(ints, ints + 4);
                    marker.middleOfMarker = ((*it1) + (*it2)) / 2;

                    markers.push_back(marker);
                    allDetectedLines.erase(it2);
                }
                else it2++;
            }else it2++;
        }
    }

    return markers;
}


float LineDetector::calculateSlope(cv::Vec4i detectedLine){
    if(detectedLine[3] == detectedLine[1]) return 3.1415/2;
    else return (float)(detectedLine[2] - detectedLine[0])/(float)(detectedLine[3] - detectedLine[1]);;
}

float LineDetector::calculateWidth(cv::Vec4i line1 , cv::Vec4i line2){
    return abs((float)((line1[0] + line1[2] - line2[0] - line2[2])/2));
}

float LineDetector::calcProjectedDistance(cv::Vec4i line1, float pixelWidthOfMarker) {
    return (((line1[0] + line1[2])/2) - (imagePixelWidth_/2.0)) * (0.5 / pixelWidthOfMarker) * (.3048);
}

float LineDetector::calcProjectedDistanceToEndOfLine(float pixelOfForwardMostPoint, float pixelWidthOfMarker){
    return (((imagePixelHeight_/2.0) - pixelOfForwardMostPoint) * (0.5 / pixelWidthOfMarker) * (.3048));
}

