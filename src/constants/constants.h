/*
 * Created By: Reid Oliveira
 * Created On: May 05, 2018
 * Description: Holds project wide constant values.
 */
#ifndef PROJECT_CONSTANTS_H
#define PROJECT_CONSTANTS_H

namespace subbots {
namespace global_constants {
    // minimum height in cm from the camera's position that an
    // overhead obstacle should be
    static const float CLEARANCE_HEIGHT = 60;

    // minimum width in cm from camera's position that
    // side obstacles should be
    static const float CLEARANCE_WIDTH = 30;
}
}

#endif // PROJECT_CONSTANTS_H
