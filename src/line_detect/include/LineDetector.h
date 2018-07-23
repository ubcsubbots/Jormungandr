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



struct LineToFollow {
    float width;
    float slope;
    cv::Vec4i middleOfMarker;
    int frontOfMarker;
};

struct LinesToFollow{
    LineToFollow frontLine;

    LineToFollow rearLine;
};

static LinesToFollow defaultLinesToFollow(){
    LinesToFollow linesToFollow;

    linesToFollow.frontLine.frontOfMarker = -1;
    linesToFollow.frontLine.width = -1;
    linesToFollow.frontLine.slope = -1;
    linesToFollow.frontLine.middleOfMarker = cv::Vec4i(0,0,0,0);

    linesToFollow.rearLine.frontOfMarker = -1;
    linesToFollow.rearLine.width = -1;
    linesToFollow.rearLine.slope = -1;
    linesToFollow.rearLine.middleOfMarker = cv::Vec4i(0,0,0,0);

    return linesToFollow;
}

class LineDetector {
  public:
    LineDetector();

    LinesToFollow initialize(const cv::Mat mat_in);

    float calcProjectedDistance(cv::Vec4i line1, float widthOfMarker);

    float calcProjectedDistanceToEndOfLine(float distanceOfForwardmostPoint, float widthOfMarker);

  private:
    int cannyLow_;

    // Maximum separation for two lines to be called a marker on the ground
    int maxLineWidth_;

    int imagePixelWidth_, imagePixelHeight_;

    int houghLinesThreshold_, houghLinesMinLength_, houghLinesMaxLineGap_;

    std::vector<LineToFollow> findMarkers(std::vector<cv::Vec4i> allDetectedLines);

    float calculateSlope(cv::Vec4i detectedLine);

    float calculateWidth(cv::Vec4i line1, cv::Vec4i line2);
};

#endif // PROJECT_GATE_H
