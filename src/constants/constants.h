/*
 * Created By: Reid Oliveira
 * Created On: May 05, 2018
 * Description: Holds project wide constant values.
 */
#ifndef PROJECT_CONSTANTS_H
#define PROJECT_CONSTANTS_H

namespace subbots {
namespace global_constants {
    //GATE PASSTHROUGH CONSTANTS
    static const float TARGET_TOP_POLE_CLEARANCE = 2.0;
    static const float ERROR_TOLERANCE_TOP_POLE_CLEARANCE = 0.1;
    static const float TARGET_TOP_POLE_DISTANCE = 6;
    static const float ERROR_TOLERANCE_TOP_POLE_DISTANCE = 0.05;
    static const float ERROR_TOLERANCE_SIDE_POLES_ANGLE = 0.1;
    static const float TARGET_SIDE_POLES_DISTANCE = 6;
    static const float ERROR_TOLERANCE_SIDE_POLES_DISTANCE = 0.05;
    static const float TARGET_AVERAGE_GATE_DISTANCE = 5;

    //LINE FOLLOW CONSTANTS
    static const float ERROR_TOLERANCE_LINE_ANGLE = 0.1;
    static const float ERROR_TOLERANCE_LINE_LATERAL_DISTANCE = 0.1;
    static const float ERROR_TOLERANCE_LINE_DISTANCE_TO_END = 0.1;
}
}

#endif // PROJECT_CONSTANTS_H
