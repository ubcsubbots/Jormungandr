/*
 * Created By: Viral Galaiya
 * Created On: July 23, 2018
 * Description:
 */
#ifndef PROJECT_CONTROLLER_H
#define PROJECT_CONTROLLER_H

#include <Eigen/Dense> //matrix manipulation library

class Controller {
public:
    Controller();
    static const double PERIOD = 0.02;

    void setImuData(double angular_x, double angular_y, double angular_z, double linear_x, double linear_y, double linear_z);
    Eigen::Matrix setDesiredVelocity(double linear_x, double linear_y, double angular_x, double angular_y, double angular_z, double position_z);
private:
    //IMU data
    Eigen::MatrixXd IMUangularvelocity; //3x1
    Eigen::MatrixXd IMUangularaccelaration; //3x1

    Eigen::Vector3d linearvelocity;
    Eigen::MatrixXd IMUlinearaccelaration;

    Eigen::Vector3d previousIMUangularvelocity;
    Eigen::Vector3d previousIMUlinearaccelaration;

    //differentiation calculated

    Eigen::MatrixXd Desiredvelocity;
    Eigen::MatrixXd previousY;
    Eigen::MatrixXd previousDesiredvelocity;
    Eigen::MatrixXd intergratorAcumulator;

    Eigen::MatrixXd Currentvelocity;
    Eigen::MatrixXd Currentacceleration;
    Eigen::MatrixXd X;
    Eigen::MatrixXd K;//X
    Eigen::MatrixXd Ki;//ki*(cY-R)
    Eigen::MatrixXd Y;
    Eigen::MatrixXd torquematrix;
    Eigen::MatrixXd PWMmatrix;
};

#endif //PROJECT_CONTROLLER_H
