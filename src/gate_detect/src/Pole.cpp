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

int Pole::getHorWidth() {
    return abs((side1_[1] + side1_[3] - side2_[1] - side2_[3]) / 2);
}


