//
// Created by cam on 09/03/19.
//

#include "../include/Interpolator.h"

Interpolator::Interpolator(int interpolation_constant1,
                           int interpolation_constant2) {
    interpolation_constant1_ = interpolation_constant1;
    interpolation_constant2_ = interpolation_constant2;
}

float Interpolator::getVertDistance(int pixel_width) {
    return interpolation_constant1_ *
           pow(pixel_width, interpolation_constant2_);
}

float Interpolator::getHorDistance(int pixel_width) {
    return interpolation_constant1_ *
           pow(pixel_width, interpolation_constant2_);
}

float Interpolator::getVertAngle(int vertical_mid,
                                 int vert_width,
                                 int width_of_image,
                                 float vertical_distance) {
    return asin(
    ((((width_of_image / 2) - vertical_mid) * (.3048 / 4)) / vert_width) /
    vertical_distance);
}

float Interpolator::getHorAngle(int pixel_height_of_image,
                                int horizontal_middle,
                                int horizontal_width,
                                float horizontal_distance) {
    return asin(
    ((((pixel_height_of_image / 2) - horizontal_middle) * (.3048 / 4)) /
     horizontal_width) /
    horizontal_distance);
}
