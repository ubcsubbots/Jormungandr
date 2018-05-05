#include "Gate.h"


Gate::Gate() {
    lowVertThresh = 50; //Sets the minimum deviation from start to finish that will constitute a vertical line
    lowHorThresh = 100; //Sets the minimum deviation from start to finish that will constitute a horizontal line
    cannyLow = 80; //Sets the low threshold for the canny filter, adjust if dst is not displaying all lines
    cannyHigh = 200; //Sets the high threshold for the canny filter, adjust if dst not displaying all lines
    poleMax = 60; //Sets the minimum threshold for two lines to be called a pole
    fiveMetreWidthofPole = 10.0f;
}

std::vector<float> Gate::initialize(const cv::Mat matin) {

    cv::blur( matin, dst, cv::Size(3,3) );

    cv::Canny(dst, dst, cannyLow, cannyHigh, 3);

    dst.copyTo( dst, detected_edges);

    //cv::imshow("new window", dst );
    //cv::waitKey(0);

    /*   cv::imshow("after filter" , dst);
        cv::waitKey(0);
    */

    std::vector<cv::Vec4i> lines;

    cv::HoughLinesP(dst, lines, 1, CV_PI / 180, 60, 50, 50);

    /*
    for(cv::Vec4i houghLine : lines)
        ROS_INFO("HoughLine detected at %i %i %i %i", houghLine[0] , houghLine[1] , houghLine[2] , houghLine[3]);
*/

    cv::Mat cdst;

    cvtColor(dst, cdst, CV_GRAY2BGR);

    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
    }

    //cv::imshow("New Window" , cdst);
    //cv::waitKey(0);


    vertLines = filterVertLines(lines);

    vertPoles = findVertPoles(vertLines);

    horLines = filterHorLines(lines);

    horPoles = findHorPoles(horLines);

    poleVector = gateVector(vertPoles, horPoles);

    return poleVector;

}

bool Gate::checkMat() {
    return (!dst.empty());
}

std::vector<int> Gate::filterVertLines(std::vector<cv::Vec4i> allLines) {
    std::vector<int> vertLines;
    vertLines.clear();

    for (int i = 0; i < (allLines.size() - 1); i++) {
        cv::Vec4i newLine = allLines[i];
        if ((abs(newLine[0] - newLine[2]) < lowVertThresh)) {
            vertLines.push_back((newLine[0] + newLine[2]) / 2);
        }
    }

    return vertLines;
}

std::vector<int> Gate::filterHorLines(std::vector<cv::Vec4i> allLines) {
    std::vector<int> horLines;
    horLines.clear();

    for (int i = 0; i < (allLines.size()); i++) {

        cv::Vec4i newLine = allLines[i];

        if (abs(newLine[1] - newLine[3]) < lowHorThresh) {
            horLines.push_back((newLine[1] + newLine[3]) / 2);
        }
    }

    return horLines;
}

std::vector<std::vector<float> > Gate::findVertPoles(std::vector<int> vertLines) {
    std::vector<std::vector<float> > vertPoles;
    vertPoles.clear();

    if(vertLines.empty()){
        return vertPoles;
    }

    for (auto it1 = vertLines.begin(); it1 != vertLines.end(); it1++){
        for (auto it2 = vertLines.begin(); it2 != vertLines.end(); ) {
            if ((abs(*it1 - *it2) < 10)&&(*it1 != *it2)) {
                vertLines.erase(it2);
            }
            else {
                it2++;
            }
        }
    }

    for (int vertLine : vertLines) ROS_INFO(" %i " , vertLine);

    for (auto it3 = vertLines.begin(); it3 != vertLines.end();it3++) {
        //ROS_INFO("First Line %i", *it3);

        for (auto it4 = vertLines.begin(); it4 != vertLines.end();) {
            //ROS_INFO("Second Line %i", *it4);

            if ((abs(*it3 - *it4) < poleMax) && (*it3!=*it4)) {
                //ROS_INFO("Line is a Pole at %f", (float) ((*it3 + *it4) / 2));
                float vertPoleDist = (abs(*it3 - *it4) / (fiveMetreWidthofPole)) * 5;
                float vertPoleAngle = atan(((abs((*it3 - *it4) - (dst.cols / 2)) / fiveMetreWidthofPole) * 5) / vertPoleDist);
                float a = ((*it3 + *it4) / 2);
                float array[] = {a, vertPoleAngle, vertPoleDist};
                std::vector<float> vertPoleVector(array, array + sizeof(array) / sizeof(array[0]));
                vertPoles.push_back(vertPoleVector);
                vertLines.erase(it4);
            }
            else{
                if(it4 != vertLines.end())
                    it4++;
            }
        }
    }

    return vertPoles;

}

std::vector<std::vector<float> > Gate::findHorPoles(std::vector<int> horLines) {
    std::vector<std::vector<float> > horPoles;
    horPoles.clear();

    if(horLines.empty()){
        ROS_INFO("horLines empty");
        return horPoles;
    }

    for (auto it1 = horLines.begin(); it1 != horLines.end(); it1++){
        for (auto it2 = horLines.begin(); it2 != horLines.end(); ) {
            if ((abs(*it1 - *it2) < 10)&&(*it1 != *it2)) {
                horLines.erase(it2);
            }
            else {
                it2++;
            }
        }
    }

    //for (int horLine : horLines) ROS_INFO(" %i " , horLine);

    for (auto it3 = horLines.begin(); it3 != horLines.end();it3++) {
        //ROS_INFO("First Line %i", *it3);

        for (auto it4 = horLines.begin(); it4 != horLines.end();) {
            //ROS_INFO("Second Line %i", *it4);

            if ((abs(*it3 - *it4) < poleMax) && (*it3!=*it4)) {
                //ROS_INFO("Line is a Pole at %f", (float) ((*it3 + *it4) / 2));
                float horPoleDist = (abs(*it3 - *it4) / (fiveMetreWidthofPole)) * 5;
                float horPoleAngle = atan(((abs((*it3 - *it4) - (dst.cols / 2)) / fiveMetreWidthofPole) * 5) / horPoleDist);
                float a = ((*it3 + *it4) / 2);
                float array[] = {a, horPoleAngle, horPoleDist};
                std::vector<float> horPoleVector(array, array + sizeof(array) / sizeof(array[0]));
                horPoles.push_back(horPoleVector);
                horLines.erase(it4);
            }
            else{
                if(it4 != horLines.end())
                    it4++;
            }
        }
    }

    return horPoles;
}

std::vector<float> Gate::gateVector(std::vector<std::vector<float> > vertPoles, std::vector<std::vector<float> > horPoles) {
    std::vector<float> gateCoord(9, 0.0f);

    for (std::vector<float> horPole : horPoles) {
        if (gateCoord[6] == 0.0f || horPole[0] > gateCoord[6]) {
            gateCoord[6] = horPole[0];
            gateCoord[7] = horPole[1];
            gateCoord[8] = horPole[2];
        }
    }

    for (std::vector<float> vertPole : vertPoles) {
        if (gateCoord[0] == 0.0f || vertPole[0] <= gateCoord[0]) {
            gateCoord[0] = vertPole[0];
            gateCoord[1] = vertPole[1];
            gateCoord[2] = vertPole[2];
        } else if (vertPole[0] >= gateCoord[3]) {
            gateCoord[3] = vertPole[0];
            gateCoord[4] = vertPole[1];
            gateCoord[5] = vertPole[2];
        }
    }

    return gateCoord;

}





