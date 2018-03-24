//
// Created by da-cam on 23/03/18.
//

#include "Gate.h"

using namespace cv;

int main(int argc, char** argv){

    Mat image;
    //! [mat]

    //! [imread]
    image = imread( "/home/da-cam/Jormungandr/src/gate_detect/test/gate.png", IMREAD_COLOR );

    if(! image.data )                              // Check for invalid input
    {
        ROS_INFO( "Could not open or find the image" ) ;
        return -1;
    }

    /*
    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);                                          // Wait for a keystroke in the window

     */

    Gate newGate;

    newGate.initialize(image);

    return 0;
}
