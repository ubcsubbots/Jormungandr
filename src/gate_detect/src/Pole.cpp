/*
 * Created By: Cameron Newton
 * Created On: February 27th, 2018
 * Description: Hold 2 lines that are determined to represent a pole
 */

#include <Pole.h>

Pole::Pole() {
    interpolationConstant1_ = 0;

    interpolationConstant2_ = 0;
}

Pole::Pole(cv::Vec4i side1, cv::Vec4i side2, float m, float b) {
    side1_ = side1;

    side2_ = side2;

    interpolationConstant1_ = m;

    interpolationConstant2_ = b;
}

int Pole::getVertMid() {
    return (side1_[0] + side1_[2] + side2_[0] + side2_[2]) / 4;
}

int Pole::getHorMid() {
    return (side1_[1] + side1_[3] + side2_[1] + side2_[3]) / 4;
}

int Pole::getVertWidth() {
    return abs((side1_[0] + side1_[2] - side2_[0] - side2_[2]) / 2);
}

float Pole::calculateSlope(cv::Vec4i detectedLine){
    return(detectedLine[3] - detectedLine[1])/(detectedLine[2] - detectedLine[0]);
}

int Pole::getHorWidth() {
        float m0,m1,b1,xi0,yi0,xi1,yi1, mi, bi;

        m0 = calculateSlope(side1_);

        m1 = calculateSlope(side2_);

        b1 = side2_[1] - m0 * side2_[0];

        xi0 = (side1_[0] + side1_[2]) / 2;

        yi0 = (side1_[1] + side1_[3]) / 2;

        mi = -(1/m0);

        bi = yi0 - mi * xi0;

        xi1 = (b1 - bi)/(mi - m1);

        yi1 = mi * xi1 + bi;

        return sqrt(pow((xi1 - xi0),2) + pow((yi1 - yi0),2));
}

float Pole::getVertDistance() {
    return interpolationConstant1_ *
           pow(getVertWidth(), interpolationConstant2_);
}

float Pole::getHorDistance() {
    return interpolationConstant1_ *
           pow(getHorWidth(), interpolationConstant2_);
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
