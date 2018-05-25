/*
 * Created By: Reid Oliveira
 * Created On: November 18th, 2017
 * Description:
 */
#ifndef VISION_HSVFILTER_H
#define VISION_HSVFILTER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class HSVFilter {
    int hue_lower_;
    int hue_upper_;
    int sat_lower_;
    int sat_upper_;
    int val_lower_;
    int val_upper_;

  public:
    HSVFilter(int hue_lower,
              int hue_upper,
              int sat_lower,
              int sat_upper,
              int val_lower,
              int val_upper);
    HSVFilter();
    void apply(const cv::Mat& original, cv::Mat& filtered);

  private:
};

#endif // VISION_HSVFILTER_H
