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

    int hsv_first[first.rows][first.cols][3];
    int hsv_second[second.rows][second.cols][3];

    for(int i = 0; i < r; i++) {
        for(int j = 0; j < c; j++) {
            hsv_first[i][j][0] = data[(int)(*first.step.p) * i + 3 * j];
            hsv_first[i][j][1] = data[(int)(*first.step.p) * i + 3 * j + 1];
            hsv_first[i][j][2] = data[(int)(*first.step.p) * i + 3 * j + 2];

            hsv_second[i][j][0] = data[(int)(*second.step.p) * i + 3 * j];
            hsv_second[i][j][1] = data[(int)(*second.step.p) * i + 3 * j + 1];
            hsv_second[i][j][2] = data[(int)(*second.step.p) * i + 3 * j + 2];
        }
    }
}

TEST(HSVFilter, filterImage) {
    cv::Mat img, result, expected;
    HSVFilter filter;
    img = cv::imread("test_img/test1.png", CV_LOAD_IMAGE_COLOR);
    expected = cv::imread("test_img/result1.png", CV_LOAD_IMAGE_COLOR);
    if(!img.empty() && !expected.empty()) {
        filter.apply(img, result);
        EXPECT_EQ(expected.data, result.data);
    } else {
        std::cout << "could not find test images" << std::endl;
        FAIL();
    }


}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}