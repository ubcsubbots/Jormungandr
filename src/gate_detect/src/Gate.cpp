#include "Gate.h"

Gate::Gate(const cv::Mat matin){

}

Gate::Gate(){
    lowThreshold = 200;

}

cv::Mat Gate::initialize(const cv::Mat matin){
    ROS_INFO("Initialize");
    cv::Canny(matin, dst, 150, 200, 3);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dst, lines, 1, 0.5, 0, 10);



}

bool Gate::checkMat() {
    return (!cdst.empty());
}

std::vector<cv::Vec4i> Gate::filterVertLines(std::vector<cv::Vec4i> allLines){
    std::vector<cv::Vec4i> vertLines;

    for (i = 0; ){
        if(abs(it[0] - it[2]) < lowVertThresh){
            vertLines.push_back(*it);
        }
    }

    return vertLines;
}

std::vector<cv::Vec4i> Gate::filterHorLines(std::vector<cv::Vec4i> allLines){
    std::vector<cv::Vec4i> horLines;

    for (std::vector<cv::Vec4i>::iterator it = alLLines.begin() ; it != allLines.end(); ++it){
        if((it[1] - it[3]) < lowHorThresh){
            horLines.push_back(*it);
        }
    }

    return horLines;
}


/*
ROS_INFO("Initialize");
cv::Canny(matin, dst, 50, 200, 3);
cv::HoughLinesP(dst, lines, 1, 0.5, 0, 10);

for( size_t i = 0; i < lines.size(); i++ )
{
cv::Vec4i l = lines[i];
cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, CV_AA);
}

ROS_INFO("Columns %i", cdst.cols);
ROS_INFO("Rows %i", cdst.rows);
ROS_INFO("CV lines %i", lines.size());

cv::imshow("post",cdst);
cv::waitKey();
 */