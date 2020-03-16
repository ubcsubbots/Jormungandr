/*
 * Created By: Logan Fillo
 * Created On: October 25 2019
 * Description: Struct for AUV state
 */

#ifndef AUV_STATE
#define AUV_STATE

/***
 * Struct to store AUV state data
 */
struct AUVState
{   
    // Body frame linear/angular velocity
    float u;
    float v;
    float w;
    float p;
    float q;
    float r;

    // Word frame position/orientation
    float x;
    float y;
    float z;
    float phi;
    float theta;
    float psi;
};

#endif // AUV_STATE
