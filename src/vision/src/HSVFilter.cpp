/*
 * Created By: Reid Oliveira
 * Created On: November 18th, 2017
 * Description:
 */

#include <HSVFilter.h>

HSVFilter::HSVFilter(int hue_lower,
                     int hue_upper,
                     int sat_lower,
                     int sat_upper,
                     int val_lower,
                     int val_upper) {
    hue_lower_ = hue_lower;
    hue_upper_ = hue_upper;
    sat_lower_ = sat_lower;
    sat_upper_ = sat_upper;
    val_lower_ = val_lower;
    val_upper_ = val_upper;
}

HSVFilter::HSVFilter() {
    // default is international orange
    hue_lower_ = 5;
    hue_upper_ = 15;
    sat_lower_ = 230;
    sat_upper_ = 255;
    val_lower_ = 230;
    val_upper_ = 255;
}

void HSVFilter::apply(const cv::Mat& original, cv::Mat& filtered) {
    cv::Mat input_hsv;
    cv::cvtColor(original, input_hsv, cv::COLOR_BGR2HSV, 0);

    cv::Scalar lower_bound(hue_lower_, sat_lower_, val_lower_);
    cv::Scalar upper_bound(hue_upper_, sat_upper_, val_upper_);

    // remember filter is of type CV_8U
    cv::inRange(input_hsv, lower_bound, upper_bound, filtered);
}
