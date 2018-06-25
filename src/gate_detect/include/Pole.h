/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 */

#ifndef PROJECT_POLE_H
#define PROJECT_POLE_H

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Pole {
  public:
    Pole();

    /**
     * Object representing a detected Pole, horizontal or vertical
     *
     * @param side1 1 side of the pole
     *
     * @param side2 the other side of the pole
     *
     * @param m, b      y = m * x ^ -b, Function interpolated from data
     *
     *                  y = distance from pole
     *
     *                  x = pixel width of pole
     */
    Pole(cv::Vec4i side1, cv::Vec4i side2, float m, float b);

    int getVertMid();

    int getHorMid();

    int getVertWidth();

    int getHorWidth();

    float getVertDistance();

    float getHorDistance();

    float getVertAngle(int pixelDistanceFromMid);

    float getHorAngle(int pixelDistanceFromMid);

  private:
    cv::Vec4i _side1, _side2;

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
    float _interpolationConstant1, _interpolationConstant2;
};

#endif // PROJECT_POLE_H
