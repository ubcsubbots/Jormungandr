/*
 * Created by tyler on 09/03/19.
 * Structure for storing poles as a gate.
 * Will be outputted from the gate initialization function.
 * Used as input to debug image function.
 * Used as input to interpolation function.
 *
**/

#ifndef PROJECT_GATE_H
#define PROJECT_GATE_H

#endif //PROJECT_GATE_H

#include "Pole.h"
#include <iostream>

struct Gate{
    Gate(){
        leftDetected = false;
        rightDetected = false;
        topDetected = false;
    }
    Pole leftPole,
         rightPole,
         topPole;
    bool leftDetected,
         rightDetected,
         topDetected;
};