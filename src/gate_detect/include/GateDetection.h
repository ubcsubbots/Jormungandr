/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 * Description: Class to scan cv::Mat object and return 9 parameter vector
 * defining where gate is seen on cv::Mat
 */

#ifndef PROJECT_GATE_H
#define PROJECT_GATE_H

#include "PoleDetection.h"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class GateDetection {
  public:
    GateDetection(int cannyLow,
                  int houghLinesThreshold,
                  int houghLinesMinLength,
                  int houghLinesMaxLineGap,
                  int poleMax);

    GateDetection();

    /**
     * Function that takes in cv::Mat object and returns vector defining where
     * gate is seen
     *
     * @param mat_in
     * @return vector defining where gate is seen (definition in getGateVector
     * comment)
     */
    std::vector<float> initialize(const cv::Mat mat_in);

  private:
    // Parameters defining the maximum deviation from begining to end that a
    // line can have to still be considered
    // Vertical or horizontal
    int _lowVertThresh, _lowHorThresh;

    /*
    * Canny( detected_edges, detected_edges, _cannyLow, lowThreshold*ratio,
    * kernel_size );
    *
    * Where the arguments are:
    *
    * detected_edges: Source image, grayscale
    * detected_edges: Output of the detector (can be the same as the input)
    * _cannyLow: The value entered by the user moving the Trackbar
    * highThreshold: Set in the program as three times the lower threshold
    * (following Canny's recommendation)
    * kernel_size: We defined it to be 3 (the size of the Sobel kernel to be
    * used internally)
    */
    int _cannyLow;

    // Maximum separation for two lines to be called a pole
    int _poleMax;

    // Input image Parameters
    int _imagePixelWidth, _imagePixelHeight;

    /*
    * HoughLinesP(dst,detectedLines,rho,theta,_houghLinesThreshold,_houghLinesMinLength,_houghLinesMaxLineGap)
    * dst: Output of the edge detector. It should be a grayscale image (although
    * in fact it is a binary one)
    * detectedLines : A vector that will store the parameters (x_{start},
    * y_{start}, x_{end}, y_{end}) of the detected lines
    * rho : The resolution of the parameter r in pixels. We use 1 pixel.
    * theta: The resolution of the parameter \theta in radians. We use 1 degree
    * (CV_PI/180)
    * _houghLinesThreshold: The minimum number of intersections to “detect” a
    * line
    * _houghLinesMinLength: The minimum number of points that can form a line.
    * Lines with less than this number of points are disregarded.
    * _houghLinesMaxLineGap: The maximum gap between two points to be considered
    * in the same line.
    */
    int _houghLinesThreshold, _houghLinesMinLength, _houghLinesMaxLineGap;

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
    float m_Hor, m_Vert, b_Hor, b_Vert;

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
    std::vector<PoleDetection> findHorPoles(std::vector<cv::Vec4i> horLines);

    /**
     * Function to filter through vector of cv::Vector4i objects and filter out
     * the vertical poles
     *
     *  @param   vertLines Vector of cv::Vector4i objects that are horizontal
     *
     *  @return  Vector of PoleDetection objects that represent Poles that have
     * been detected
     */
    std::vector<PoleDetection> findVertPoles(std::vector<cv::Vec4i> vertLines);

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
     *          [detectedLeft, distanceLeft, angleLeft, detectedRight,
     *          distanceRight, angleRight, detectedTop, distanceTop, angleTop]
     *
     *          detectedLeft:   0 if left pole not seen, 1 if seen
     *
     *          angleLeft:      Angle from vertical centre to left pole
     *
     *          distanceLeft:   Distance to left pole, 0 if not seen
     *
     *          detectedRight:  0 if right pole not seen, 1 if seen
     *
     *          angleRight:     Angle from right centre to left pole
     *
     *          distanceRight:  Distance to right pole, 0 if not seen
     *
     *          detectedTop:    0 if top pole not seen, 1 if seen
     *
     *          angleTop:       Angle from horizontal centre to top pole
     *
     *          distanceTop:    Distance to top pole, 0 if not seen
     */
    std::vector<float> getGateVector(std::vector<PoleDetection> vertPoles,
                                     std::vector<PoleDetection> horPoles);
};

#endif // PROJECT_GATE_H
