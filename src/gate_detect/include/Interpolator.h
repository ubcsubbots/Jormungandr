/*
 * Created By: Cameron Newton
 * Created On: March 16th, 2019
 * Description: Interpolator class to calculate distance and angle of Pole from
 * position and width of Pole in image
 */

#ifndef PROJECT_INTERPOLATOR_H
#define PROJECT_INTERPOLATOR_H

#include <cmath>

class Interpolator {
  public:
    Interpolator(double interpolation_constant1,
                 double interpolation_constant2);
    Interpolator();

    /**
     * Get the estimated distance from a vertical pole based on the pixel width
     * of the pole
     *
     * @param pixel_width The width of the pole in pixels
     * @return The estimated distance from the pole based on the interpolation
     * method
     */
    float getVertDistance(int pixel_width);

    /**
     * Get the distance from a horizontal pole based on the pixel width of the
     * pole
     *
     * @param pixel_width The height of the image in pixels
     * @return The estimated distance from the pole based on the Interpolation
     * method
     */
    float getHorDistance(int pixel_width);

    /**
     * Get the vertical angle of the pole (left or right) from the centre of the
     * image
     *
     * @param vertical_mid The average x-coordinate of the middle of the pole in
     * the image
     * @param vert_width The x-width of the pole in the image
     * @param width_of_image The total pixel width of the image that the pole
     * was detected in
     * @param vertical_distance The estimated distance from the vertical pole
     * @return The angle of the pole in the image, measured from the vertical
     * middle of the image to the pole in radians
     */
    float getVertAngle(int x_coord_of_vertical_pole,
                       int pixel_width_of_pole,
                       int pixel_width_of_image,
                       float vertical_distance);

    /**
     * Get the Horizontal angle of the top pole from the centre of the image
     *
     * @param horizontal_middle The average x-coordinate of the middle of the
     * pole in the image
     * @param horizontal_width The x-width of the pole in the image
     * @param width_of_image The total pixel width of the image that the pole
     * was detected in
     * @param pixel_height_of_image The estimated distance from the horizontal
     * pole
     * @return The angle of the pole in the image, measured from the horizontal
     * middle of the image to the pole in radians
     */
    float getHorAngle(int y_coord_of_horizontal_pole,
                      int pixel_height_of_pole,
                      int pixel_height_of_image,
                      float horizontal_distance);

  private:
    /**
     * Constants defining relationship between pixel width and distance from
     * Pole
     * gathered from data collected in real world. Interpolation constants are
     * derived from
     *
     * y = distance from pole
     * x = pixel width of pole
     * m = interpolation constant 1
     * b = interpolation constant 2
     *
     * For Exponential interpolation
     * y = m*(b^x)
     */
    float interpolation_constant1_, interpolation_constant2_;
};

#endif // PROJECT_INTERPOLATOR_H
