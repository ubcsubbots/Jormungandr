/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 * Description: Hold 2 lines that are determined to represent a pole
 */

#include <Pole.h>

Pole::Pole() {
    _interpolationConstant1 = 0;

    _interpolationConstant2 = 0;
}

Pole::Pole(cv::Vec4i side1, cv::Vec4i side2, float m, float b) {
    _side1 = side1;

    _side2 = side2;

    _interpolationConstant1 = m;

    _interpolationConstant2 = b;
}

int Pole::getVertMid() {
    return (_side1[0] + _side1[2] + _side2[0] + _side2[2]) / 4;
}

int Pole::getHorMid() {
    return (_side1[1] + _side1[3] + _side2[1] + _side2[3]) / 4;
}

int Pole::getVertWidth() {
    return abs((_side1[0] + _side1[2] - _side2[0] - _side2[2]) / 2);
}

int Pole::getHorWidth() {
    return abs((_side1[1] + _side1[3] - _side2[1] - _side2[3]) / 2);
}

float Pole::getVertDistance() {
    return _interpolationConstant1 *
           pow(getVertWidth(), _interpolationConstant2);
}

float Pole::getHorDistance() {
    return _interpolationConstant1 *
           pow(getHorWidth(), _interpolationConstant2);
}

float Pole::getVertAngle(int pixelWidthOfImage) {
    return asin(((((pixelWidthOfImage / 2) - getVertMid()) * (.3048 / 4)) /
                 getVertWidth()) /
                getVertDistance());
}

float Pole::getHorAngle(int pixelHeightOfImage) {
    return asin(
    ((((pixelHeightOfImage / 2) - getHorMid()) * (.3048 / 4)) / getHorWidth()) /
    getHorDistance());
}
