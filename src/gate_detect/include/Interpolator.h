//
// Created by cam on 09/03/19.
//

#ifndef PROJECT_INTERPOLATOR_H
#define PROJECT_INTERPOLATOR_H

enum TYPE {
    EXPONENTIAL, LINEAR
};

class Interpolator {
public:
    Interpolator();

    float getVertDistance() {};

    float getHorDistance() {};

    float getVertAngle(int pixelWidthOfImage) {};

    float getHorAngle(int pixelHeightOfImage) {};

private:
    float interpolation_constant1_, interpolation_constant2_;
};


#endif //PROJECT_INTERPOLATOR_H
