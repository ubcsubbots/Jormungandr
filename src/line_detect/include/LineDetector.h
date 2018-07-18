/*
 * Created By: Cameron Newton
 * Created On: July 17, 2018
 * Description: Class to scan cv::Mat object and return 3 parameter
 * stuct representing orientation with pool marker
 */

#ifndef PROJECT_GATE_H
#define PROJECT_GATE_H

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>



struct LineStruct {
    float lateralDistanceFromFrontMarker;
    float distanceFromEndFrontMarker;
    float angleToParallelFrontMarker;
    float lateralDistanceFromRearMarker;
    float distanceFromEndRearMarker;
    float angleToParallelRearMarker;
};

static LineStruct defaultLineStruct() {
    LineStruct lineStruct;

    lineStruct.angleToParallelFrontMarker = -1.0f;

    lineStruct.distanceFromEndFrontMarker = -1.0f;

    lineStruct.lateralDistanceFromFrontMarker = -1.0f;

    lineStruct.lateralDistanceFromRearMarker=-1.0f;

    lineStruct.distanceFromEndRearMarker=-1.0f;

    lineStruct.angleToParallelRearMarker=-1.0f;

    return lineStruct;
}

struct MarkerStruct {
    float width;
    float slope;
    cv::Vec4i middleOfMarker;
    int frontOfMarker;
};

class LineDetector {
  public:
    LineDetector();

    LineStruct initialize(const cv::Mat mat_in);

  private:
    int cannyLow_;

    // Maximum separation for two lines to be called a marker on the ground
    int maxLineWidth_;

    int imagePixelWidth_, imagePixelHeight_;

    int houghLinesThreshold_, houghLinesMinLength_, houghLinesMaxLineGap_;

    std::vector<MarkerStruct> findMarkers(std::vector<cv::Vec4i> allDetectedLines);

    float calculateSlope(cv::Vec4i detectedLine);

    float calculateWidth(cv::Vec4i line1, cv::Vec4i line2);

    float calcProjectedDistance(cv::Vec4i line1, float widthOfMarker);

    float calcProjectedDistanceToEndOfLine(float distanceOfForwardmostPoint, float widthOfMarker);
};

#endif // PROJECT_GATE_H
