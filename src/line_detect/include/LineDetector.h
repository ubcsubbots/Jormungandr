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

/**
*      Struct representing a marker on the bottom of the pool
*          width:   width of marker, used to interpolate distance from marker,
* -1 if not seen
*          slope:   slope of marker seen, -1 if not seen
*          middleOfMarker:   vector representing middle of marker, -1 if not
* seen
*          frontOfMarker:  distance to end of marker, -1 if not seen
*/
struct LineToFollow {
    float width;
    float slope;
    cv::Vec4i middleOfMarker;
    int frontOfMarker;
};

/**
*      Struct representing 2 markers seen on bottom of pool
*          frontLine: Marker to follow
*          rearLine: Marker used in case we can't see front marker
*/
struct LinesToFollow {
    LineToFollow frontLine;

    LineToFollow rearLine;
};

/**
*      Default Constructor
*
*/
static LinesToFollow defaultLinesToFollow() {
    LinesToFollow linesToFollow;

    linesToFollow.frontLine.frontOfMarker  = -1;
    linesToFollow.frontLine.width          = -1;
    linesToFollow.frontLine.slope          = -1;
    linesToFollow.frontLine.middleOfMarker = cv::Vec4i(0, 0, 0, 0);

    linesToFollow.rearLine.frontOfMarker  = -1;
    linesToFollow.rearLine.width          = -1;
    linesToFollow.rearLine.slope          = -1;
    linesToFollow.rearLine.middleOfMarker = cv::Vec4i(0, 0, 0, 0);

    return linesToFollow;
}

class LineDetector {
  public:
    LineDetector();

    LinesToFollow initialize(const cv::Mat mat_in);

    /**
    *      Estimates projected lateral distance of marker from middle of hsv_filter
    *
    *      @param middleOfMarker line representing middle of marker
    *      @param widthOfMarker width of marker
    *      @return projected distance projected lateral distance to marker
    *
    */
    float calcProjectedDistance(cv::Vec4i middleOfMarker, float widthOfMarker);

    /**
    *      Estimates projected lateral distance to end of marker
    *
    *      @param distanceOfForwardmostPoint point of forward most point of
    * marker
    *      @param widthOfMarker width of marker
    *      @return projected distance projected lateral distance to marker
    *
    */
    float calcProjectedDistanceToEndOfLine(float distanceOfForwardmostPoint,
                                           float widthOfMarker);

  private:
    // Canny filter threshold
    int cannyLow_;

    int imagePixelWidth_, imagePixelHeight_;

    int houghLinesThreshold_, houghLinesMinLength_, houghLinesMaxLineGap_;

    std::vector<LineToFollow>
    findMarkers(std::vector<cv::Vec4i> allDetectedLines);

    /**
    *   Calculate slope of line
     *
     *   @param cv::Vec4i line
     *   @return slope of line
    */
    float calculateSlope(cv::Vec4i detectedLine);

    /**
    *   Estimate width of line
     *
     *   @param cv::Vec4i line1
     *   @param cv::Vec4i line2
     *   @return estimated width of line
     *
    */
    float calculateWidth(cv::Vec4i line1, cv::Vec4i line2);
};

#endif // PROJECT_GATE_H
