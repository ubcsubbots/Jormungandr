/*
 * Created By: Viral Galaiya
 * Created On: November 16th, 2018
 * Description:
 */
#ifndef PROJECT_ACCUMULATOR_H
#define PROJECT_ACCUMULATOR_H

#include<opencv2\opencv.hpp>
#include<opencv2\core\core.hpp>
#include <opencv2/highgui/highgui.hpp>


class Accululator {


public:
    Accumulator();

    void initAccumulator((const cv::mat& video);

    void mask(const cv::mat& video);

private:
    cv::mat accumulator;


};



#endif //PROJECT_ACCUMULATOR_H

