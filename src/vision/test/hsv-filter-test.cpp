/*
 * Created By: Reid Oliveira
 * Created On: January 06, 2018
 * Description:
 */

#include <HSVFilter.h>
#include <gtest/gtest.h>
#include <opencv2/highgui.hpp>

/**
 * checks to see if two cv::Mat objects are the same, assuming they contain color data
 * @param first
 * @param second
 * @return true if they are equal, else false
 */
bool compareMat(cv::Mat& first, cv::Mat& second) {
    unsigned char *data = first.data;
    int r, c;

    // initial dimension checks
    if(first.dims != second.dims){
        return false;
    }
    if(first.rows != second.rows) {
        r = first.rows;
        return false;
    }
    if(first.cols != second.cols) {
        c = first.cols;
        return false;
    }

    // extract data and check
    for(int i = 0; i < r; i++) {
        for(int j = 0; j < c; j++) {
            int first_h = data[(int)(*first.step.p) * i + 3 * j];
            int first_s = data[(int)(*first.step.p) * i + 3 * j + 1];;
            int first_v = data[(int)(*first.step.p) * i + 3 * j + 2];

            int second_h = data[(int)(*second.step.p) * i + 3 * j];
            int second_s = data[(int)(*second.step.p) * i + 3 * j + 1];
            int second_v = data[(int)(*second.step.p) * i + 3 * j + 2];

            if (first_h != second_h || first_s != second_s || first_v != second_v) {
                return false;
            }
        }
    }

    return true;
}

TEST(HSVFilter, filterImage) {
    cv::Mat img, result, expected;
    HSVFilter filter;
    img = cv::imread("test_img/test1.png", CV_LOAD_IMAGE_COLOR);
    expected = cv::imread("test_img/result1.png", CV_LOAD_IMAGE_COLOR);
    if(!img.empty() && !expected.empty()) {
        filter.apply(img, result);
        EXPECT_TRUE(compareMat(expected, result));
    } else {
        std::cout << "could not find test images" << std::endl;
        FAIL();
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}