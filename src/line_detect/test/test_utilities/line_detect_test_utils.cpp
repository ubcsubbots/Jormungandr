/*
 * Created By: Cameron Newton
 * Created On: June 16th, 2018
 * Description: Utilities to aid in debugging LineDetector module
                mat_in  -> input matrix of original image
                LineDetector -> all the lines which are detected
                LineToFollow -> detected line struct of type LineDetector which have to be followed
 */

#include "line_detect_test_utils.h"
#include <iostream>


using namespace cv;


cv::Mat TestUtils::drawLineToFollow(cv::Mat mat_in, LineDetector lineDetector, LinesToFollow linesToFollow ) {
    line(mat_in,
         cv::Point(linesToFollow.rearLine.middleOfMarker[0],
                   linesToFollow.rearLine.middleOfMarker[1]),
         cv::Point(linesToFollow.rearLine.middleOfMarker[2],
                   linesToFollow.rearLine.middleOfMarker[3]),
         cv::Scalar(0, 255, 0),
         1);
    line(mat_in,
         cv::Point(linesToFollow.frontLine.middleOfMarker[0],
                   linesToFollow.frontLine.middleOfMarker[1]),
         cv::Point(linesToFollow.frontLine.middleOfMarker[2],
                   linesToFollow.frontLine.middleOfMarker[3]),
         cv::Scalar(0, 0, 255),
         1);


    for( size_t i = 0; i <  lineDetector.detectedLines.size(); i++ )
    {
        Vec4i l =  lineDetector.detectedLines[i];
        line( mat_in, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
    return mat_in;
        //imshow("mat_in", mat_in);


   // waitKey(0);
}


