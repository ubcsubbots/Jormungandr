/*
 * Created By: reidoliveira
 * Created On: January 27, 2018
 * Description:
 */
#include "ImageTestUtils.h"

/**
 * checks to see if two cv::Mat objects are the same, assuming they contain
 * color data
 * @param first
 * @param second
 * @return true if they are equal, else false
 */
bool ImageTestUtils::compareMat(cv::Mat& first, cv::Mat& second) {
    // initial dimension checks
    if (first.dims != second.dims) { return false; }
    if (first.rows != second.rows) { return false; }
    if (first.cols != second.cols) { return false; }
    if ((int) *first.step.p != (int) *second.step.p) { return false; }

    // compareImg(first, second);

    unsigned char* f_data = first.data;
    unsigned char* s_data = second.data;
    int r                 = first.rows;
    int c                 = first.cols;
    int numVals           = (int) *first.step.p / c;
    // extract data and check
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            for (int v = 0; v < numVals; v++) {
                int val1 = f_data[(int) (*first.step.p) * i + numVals * j + v];
                int val2 = s_data[(int) (*second.step.p) * i + numVals * j + v];

                if (val1 != val2) { return false; }
            }
        }
    }

    return true;
}

void ImageTestUtils::compareImg(cv::Mat& expected, cv::Mat& actual) {
    cv::namedWindow("expected");
    cv::namedWindow("actual");
    cv::imshow("expected", expected);
    cv::imshow("actual", actual);
    cv::moveWindow("expected", 200, 200);
    cv::moveWindow("actual", 400, 200);
    cv::waitKey(0);
}
