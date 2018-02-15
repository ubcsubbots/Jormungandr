#include <opencv2/highgui.hpp>
#include <iostream>

#ifndef GATE_DETECT_GATEDETECT_H
#define GATE_DETECT_GATEDETECT_H

class imDisplay{
    private:
    cv::Mat image;

    imDisplay(int argc ,char** argv, std::string imgpath){

        image = cv::imread(imgpath, CV_LOAD_IMAGE_COLOR);
        if(! image.data ) {
        std::cout <<  "Could not open or find the image" << std::endl ;
        }
    }

    void printImage(){
        cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
        cv::imshow( "Display window", image );
    
        cv::waitKey(0);
    }

};

#endif //GATE_DETECT_GATEDETECT_H