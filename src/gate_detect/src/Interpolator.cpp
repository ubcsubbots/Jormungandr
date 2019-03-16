//
// Created by cam on 09/03/19.
//

#include "../include/Interpolator.h"

float Interpolator::getVertDistance() {
    return interpolationConstant1_ *
           pow(getVertWidth(), interpolationConstant2_);
}

float Interpolator::getHorDistance() {
    return interpolationConstant1_ *
           pow(getHorWidth(), interpolationConstant2_);
}

float Interpolator::getVertAngle(int pixelWidthOfImage) {
    return asin(((((pixelWidthOfImage / 2) - getVertMid()) * (.3048 / 4)) /
                 getVertWidth()) /
                getVertDistance());
}

float Interpolator::getHorAngle(int pixelHeightOfImage) {
    return asin(
            ((((pixelHeightOfImage / 2) - getHorMid()) * (.3048 / 4)) / getHorWidth()) /
            getHorDistance());
}