/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 * Description: Class to scan cv::Mat object and return 9 parameter vector
 * defining where gate is seen on cv::Mat
 */

#ifndef PROJECT_GATE_H
#define PROJECT_GATE_H

#include "Pole.h"
#include "Gate.h"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
* [detectedLeftPole, distanceLeftPole, angleLeftPole, detectedRightPole,
*          distanceRightPole, angleRightPole, detectedTopPole, distanceTopPole,
* angleTopPole]
*
*          detectedLeftPole:   0 if left pole not seen, 1 if seen
*
*          angleLeftPole:      Angle from vertical centre to left pole
*
*          distanceLeftPole:   Distance to left pole, 0 if not seen
*
*          detectedRightPole:  0 if right pole not seen, 1 if seen
*
*          angleRightPole:     Angle from right centre to left pole
*
*          distanceRightPole:  Distance to right pole, 0 if not seen
*
*          detectedTopPole:    0 if top pole not seen, 1 if seen
*
*          angleTopPole:       Angle from horizontal centre to top pole
*
*          distanceTopPole:    Distance to top pole, 0 if not seen
*
*/
struct GateCoordinates {
    float detectedLeftPole;
    float angleLeftPole;
    float distanceLeftPole;
    float detectedRightPole;
    float angleRightPole;
    float distanceRightPole;
    float detectedTopPole;
    float angleTopPole;
    float distanceTopPole;
};

static GateCoordinates defaultGateCoordinates() {
    GateCoordinates gateCoordinates;

    gateCoordinates.distanceTopPole   = 0.0f;
    gateCoordinates.angleTopPole      = 0.0f;
    gateCoordinates.detectedTopPole   = 0.0f;
    gateCoordinates.detectedLeftPole  = 0.0f;
    gateCoordinates.angleLeftPole     = 0.0f;
    gateCoordinates.distanceLeftPole  = 0.0f;
    gateCoordinates.distanceRightPole = 0.0f;
    gateCoordinates.angleRightPole    = 0.0f;
    gateCoordinates.detectedRightPole = 0.0f;

    return gateCoordinates;
}

class GateDetector {
  public:
    GateDetector(int cannyLow,
                 int houghLinesThreshold,
                 int houghLinesMinLength,
                 int houghLinesMaxLineGap,
                 int poleMax,
                 double verticalInterpolationConstant,
                 double horizontalInterpolationConstanct,
                 int lowVertThresh,
                 int lowHorThresh);

    GateDetector();

    /**
     * Function that takes in cv::Mat object and returns vector defining where
     * gate is seen
     *
     * @param mat_in
     * @return vector defining where gate is seen (definition in
     * getGateCoordinates
     * comment)
     */
    Gate initialize(const cv::Mat mat_in);

    /**
     * Function that sets the parameters of the Gate Detector
     *
     * @param cannyLow
     * @param houghLinesThreshold
     * @param houghLinesMinLength
     * @param houghLinesMaxLineGap
     * @param poleMax
     * @param verticalInterpolationConstance
     * @param horizontalInterpolationConstance
     */
    void setParams(int cannyLow,
                   int houghLinesThreshold,
                   int houghLinesMinLength,
                   int houghLinesMaxLineGap,
                   int poleMax,
                   float interpolationConstant1,
                   float interpolationConstant2,
                   int lowVertThresh,
                   int LowHorThresh);

  private:
    // Parameters defining the maximum deviation from begining to end that a
    // line can have to still be considered
    // Vertical or horizontal
    int lowVertThresh_, lowHorThresh_;

    /*
    * Canny( detected_edges, detected_edges, cannyLow_, lowThreshold*ratio,
    * kernel_size );
    *
    * Where the arguments are:
    *
    * detected_edges: Source image, grayscale
    * detected_edges: Output of the detector (can be the same as the input)
    * cannyLow_: The value entered by the user moving the Trackbar
    * highThreshold: Set in the program as three times the lower threshold
    * (following Canny's recommendation)
    * kernel_size: We defined it to be 3 (the size of the Sobel kernel to be
    * used internally)
    */
    int cannyLow_;

    // Maximum separation for two lines to be called a pole
    int poleMax_;

    // Input image Parameters
    int imagePixelWidth_, imagePixelHeight_;

    /*
    * HoughLinesP(dst,detectedLines,rho,theta,houghLinesThreshold_,houghLinesMinLength_,houghLinesMaxLineGap_)
    * dst: Output of the edge detector. It should be a grayscale image (although
    * in fact it is a binary one)
    * detectedLines : A vector that will store the parameters (x_{start},
    * y_{start}, x_{end}, y_{end}) of the detected lines
    * rho : The resolution of the parameter r in pixels. We use 1 pixel.
    * theta: The resolution of the parameter \theta in radians. We use 1 degree
    * (CV_PI/180)
    * houghLinesThreshold_: The minimum number of intersections to “detect” a
    * line
    * houghLinesMinLength_: The minimum number of points that can form a line.
    * Lines with less than this number of points are disregarded.
    * houghLinesMaxLineGap_: The maximum gap between two points to be considered
    * in the same line.
    */
    int houghLinesThreshold_, houghLinesMinLength_, houghLinesMaxLineGap_;

    /*
     * Constants defining relationship
     * between pixel width and distance
     * gathered from calibration
     *  y = m * x + b
     *
     *  y = Distance from pole
     *
     *  x = pixel width of pole
     */
    double VertInterpolationConstant2_, HorInterpolationConstant2_,
    VertInterpolationConstant1_, HorInterpolationConstant1_;

    /**
     * Function to filter through vector of cv::Vector4i objects and filter out
     * the horizontal detectedLines
     *
     * @param   allLines
     *
     *          Vector of cv::Vector4i objects in which the Vector4i
     *          represents a detectedLines such that [xo , yo , xf, yf]
     *
     *          xo = x coordinate of start of line
     *
     *          yo = y coordinate of start of line
     *
     *          xf = x coordinate of end of line
     *
     *          yf = y coordinate of end of line
     *
     * @return  Vector of all lines that are horizontal
     */
    std::vector<cv::Vec4i> filterHorLines(std::vector<cv::Vec4i> allLines);

    /**
     * Function to filter through vector of cv::Vector4i objects and filter out
     * the vertical detectedLines
     *
     *  @param  allLines
     *
     *          Vector of cv::Vector4i objects in which the Vector4i
     *          represents a detectedLines such that [xo , yo , xf, yf]
     *
     *          xo = x coordinate of start of line
     *
     *          yo = y coordinate of start of line
     *
     *          xf = x coordinate of end of line
     *
     *          yf = y coordinate of end of line
     *
     *  @return Vector of all lines that are vertical
     */
    std::vector<cv::Vec4i> filterVertLines(std::vector<cv::Vec4i> allLines);

    /**
     * Function to filter through vector of cv::Vector4i objects and filter out
     * the horizontal poles.
     *
     *  @param   horLines  Vector of cv::Vector4i objects that are horizontal
     *
     *  @return  Vector of PoleDetection objects that represent Poles that have
     * been detected
     */
    std::vector<Pole> findHorPoles(std::vector<cv::Vec4i> horLines);

    /**
     * Function to filter through vector of cv::Vector4i objects and filter out
     * the vertical poles
     *
     *  @param   vertLines Vector of cv::Vector4i objects that are horizontal
     *
     *  @return  Vector of PoleDetection objects that represent Poles that have
     * been detected
     */
    std::vector<Pole> findVertPoles(std::vector<cv::Vec4i> vertLines);

    /**
     * Function to filter through set of vertical and horizontal poles to
     * determine which poles constitute the detected gate
     *
     *  @param vertPoles Vector of vertical poles
     *
     *  @param horPoles Vector of horizontal poles
     *
     *  @return vector of parameters of detected gate
     *
     */
    Gate getGate(std::vector<Pole> vertPoles, std::vector<Pole> horPoles);
};

#endif // PROJECT_GATE_H
