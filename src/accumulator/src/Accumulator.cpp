/*
 * Created By: Viral Galaiya
 * Created On: July 23, 2018
 * Description:
 */
//https://stackoverflow.com/questions/18748775/c-opencv-accumulateweighted-strange-behaviour

#include <Accumulator.h>


Accumulator::Accumulator(){
  flag = 0;

    //CV_32FC2 means a 2-channel (complex) floating-point array
}

void Accumulator::mask(const cv::Mat& frame){

  if (flag == 0){  // initiaizes the matrix for the accumlator, can't do it in the constructor because it needs the frame dimentions

    acc = cv::Mat::zeros(frame.size(), CV_32F); 
    flag = 1;
  }
  
  cvtColor(frame ,gray ,CV_BGR2GRAY,0); // converts the image frame to grayscale
  accumulateWeighted(gray, acc, 0.05);
  	 


}
