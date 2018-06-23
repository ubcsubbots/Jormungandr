//
// Created by da-cam on 15/06/18.
//

#ifndef PROJECT_POLE_H
#define PROJECT_POLE_H

#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class PoleDetection {
    cv::Vec4i _side1, _side2;
    float _m, _b;

  public:
    PoleDetection();

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
    PoleDetection(cv::Vec4i side1, cv::Vec4i side2, float m, float b);

    int getVertMid();

    int getHorMid();

    int getVertWidth();

    int getHorWidth();

    float getVertDistance();

    float getHorDistance();

    float getVertAngle(int pixelDistanceFromMid);

    float getHorAngle(int pixelDistanceFromMid);
};

#endif // PROJECT_POLE_H
