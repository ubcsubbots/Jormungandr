/*
 * Created By: Cameron Newton
 * Created On: March 16th, 2019
 * Description: Class to scan cv::Mat object and return 9 parameter vector
 * defining where gate is seen on cv::Mat
 */

#ifndef PROJECT_INTERPOLATOR_H
#define PROJECT_INTERPOLATOR_H

#include <cmath>

class Interpolator {
  public:
    Interpolator(double interpolation_constant1,
                 double interpolation_constant2);
    Interpolator();

    enum TYPE { EXPONENTIAL, LINEAR };

    float getVertDistance(int pixel_width);

    float getHorDistance(int pixel_width);

    float getVertAngle(int vertical_mid,
                       int vert_width,
                       int width_of_image,
                       float vertical_distance);

    float getHorAngle(int pixel_height_of_image,
                      int horizontal_middle,
                      int horizontal_width,
                      float horizontal_distance);

  private:
    /**
    * Constants defining relationship
    * between pixel width and distance
    * gathered from calibration
    *  y = m * x + b
    *
    *  y = Distance from pole
    *
    *  x = pixel width of pole
    */
    float interpolation_constant1_, interpolation_constant2_;
};

#endif // PROJECT_INTERPOLATOR_H
