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


struct LineStruct {
    float distanceFromLine;
    float distanceFromEnd;
    float angleToParallel;
};

static LineStruct defaultLineStruct() {
    LineStruct lineStruct;

    lineStruct.angleToParallel = -1.0f;

    lineStruct.distanceFromEnd = -1.0f;

    lineStruct.distanceFromLine = -1.0f;

    return lineStruct;
}

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

    std::vector<std::pair<cv::Vec4i,cv::Vec4i>>  findMarker(std::vector<cv::Vec4i> allDetectedLines);

    float calculateSlope(cv::Vec4i detectedLine);

    float calculateWidth(cv::Vec4i line1, cv::Vec4i line2);

    float calcProjectedDistance(cv::Vec4i line1);

    float calcProjectedDistanceToEndOfLine(cv::Vec4i line1);
};

#endif // PROJECT_GATE_H
