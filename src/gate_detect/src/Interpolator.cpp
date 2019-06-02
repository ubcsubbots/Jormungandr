/*
 * Created By: Cameron Newton
 * Created On: March 16th, 2019
 * Description: Interpolator class to calculate distance and angle of Pole from
 * position and width of Pole in image
 */

#include "../include/Interpolator.h"

Interpolator::Interpolator(double interpolation_constant1,
                           double interpolation_constant2) {
    interpolation_constant1_ = (float) interpolation_constant1;
    interpolation_constant2_ = (float) interpolation_constant2;
}

Interpolator::Interpolator() {
    interpolation_constant1_ = 0;
    interpolation_constant2_ = 0;
}

float Interpolator::getVertDistance(int pixel_width) {
    return interpolation_constant1_ *
           pow(pixel_width, interpolation_constant2_);
}

float Interpolator::getHorDistance(int pixel_width) {
    return interpolation_constant1_ *
           pow(pixel_width, interpolation_constant2_);
}

float Interpolator::getVertAngle(int x_coord_of_vertical_pole,
                                 int pixel_width_of_pole,
                                 int pixel_width_of_image,
                                 float vertical_distance) {
    return asin(((((pixel_width_of_image / 2) - x_coord_of_vertical_pole)) /
                 pixel_width_of_pole) /
                vertical_distance);
}

float Interpolator::getHorAngle(int y_coord_of_horizontal_pole,
                                int pixel_height_of_pole,
                                int pixel_height_of_image,
                                float horizontal_distance) {
    return asin(((((y_coord_of_horizontal_pole / 2) - pixel_height_of_pole)) /
                 pixel_height_of_image) /
                horizontal_distance);
}
