//
// Created by da-cam on 15/06/18.
//

#include <PoleDetection.h> //
// Created by da-cam on 15/06/18.
//

PoleDetection::PoleDetection() {
    _m = 0;

    _b = 0;
}

PoleDetection::PoleDetection(cv::Vec4i side1,
                             cv::Vec4i side2,
                             float m,
                             float b) {
    _side1 = side1;

    _side2 = side2;

    _m = m;

    _b = b;
}

int PoleDetection::getVertMid() {
    return (_side1[0] + _side1[2] + _side2[0] + _side2[2]) / 4;
}

int PoleDetection::getHorMid() {
    return (_side1[1] + _side1[3] + _side2[1] + _side2[3]) / 4;
}

int PoleDetection::getVertWidth() {
    return abs((_side1[0] + _side1[2] - _side2[0] - _side2[2]) / 2);
}

int PoleDetection::getHorWidth() {
    return abs((_side1[1] + _side1[3] - _side2[1] - _side2[3]) / 2);
}

float PoleDetection::getVertDistance() {
    return _m * pow(getVertWidth(), _b);
}

float PoleDetection::getHorDistance() {
    return _m * pow(getHorWidth(), _b);
}

float PoleDetection::getVertAngle(int pixelWidthOfImage) {
    return asin(((((pixelWidthOfImage / 2) - getVertMid()) * (.3048 / 4)) /
                 getVertWidth()) /
                getVertDistance());
}

float PoleDetection::getHorAngle(int pixelHeightOfImage) {
    return asin(
    ((((pixelHeightOfImage / 2) - getHorMid()) * (.3048 / 4)) / getHorWidth()) /
    getHorDistance());
}
