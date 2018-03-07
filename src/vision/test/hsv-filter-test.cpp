/*
 * Created By: Reid Oliveira
 * Created On: January 06, 2018
 * Description:
 */

#include "ImageTestUtils.h"
#include <HSVFilter.h>
#include <gtest/gtest.h>
#include <opencv2/highgui.hpp>

TEST(HSVFilter, filterImage) {
    cv::Mat img, result, expected;
    HSVFilter filter;
    img      = cv::imread("test_img/test1.png", CV_LOAD_IMAGE_COLOR);
    expected = cv::imread("test_img/result1.png", CV_LOAD_IMAGE_GRAYSCALE);
    //    expected = cv::imread("test_img/result1.png", CV_LOAD_IMAGE_COLOR);
    if (!img.empty() && !expected.empty()) {
        filter.apply(img, result);
        EXPECT_TRUE(ImageTestUtils::compareMat(expected, result));
    } else {
        std::cout << "could not find test images" << std::endl;
        FAIL();
    }
}

TEST(HSVFilter, failCase) {
    cv::Mat img, result, expected;
    HSVFilter filter;
    img      = cv::imread("test_img/test1.png", CV_LOAD_IMAGE_COLOR);
    expected = cv::imread("test_img/result1.png", CV_LOAD_IMAGE_COLOR);
    if (!img.empty() && !expected.empty()) {
        EXPECT_FALSE(ImageTestUtils::compareMat(expected, result));
    } else {
        std::cout << "could not find test images" << std::endl;
        FAIL();
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}