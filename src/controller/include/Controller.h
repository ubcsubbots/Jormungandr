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
    static const double PERIOD;

    void setImuData(double angular_x,
                    double angular_y,
                    double angular_z,
                    double linear_x,
                    double linear_y,
                    double linear_z);
    Eigen::MatrixXd setDesiredVelocity(double linear_x,
                                       double linear_y,
                                       double angular_x,
                                       double angular_y,
                                       double angular_z,
                                       double position_z);

    void setDepthData(double depth);


  private:
    // IMU data
    Eigen::MatrixXd imu_angular_velocity_;     // 3x1
    Eigen::MatrixXd imu_angular_accelaration_; // 3x1

    Eigen::Vector3d linear_velocity_;
    Eigen::MatrixXd imu_linear_accelaration_;

    Eigen::Vector3d previous_imu_angular_velocity_;
    Eigen::Vector3d previous_imu_linear_accelaration_;

    // differentiation calculated

    Eigen::MatrixXd desired_velocity_;
    Eigen::MatrixXd previous_y_;
    Eigen::MatrixXd previous_desired_velocity_;
    Eigen::MatrixXd intergrator_acumulator_;

    Eigen::MatrixXd current_velocity_;
    Eigen::MatrixXd current_acceleration_;
    Eigen::MatrixXd x_;
    Eigen::MatrixXd k_;  // X
    Eigen::MatrixXd ki_; // ki*(cY-R)
    Eigen::MatrixXd y_;
    Eigen::MatrixXd torque_matrix_;
    Eigen::MatrixXd pwm_matrix_;

    double depth_;

};

#endif // PROJECT_CONTROLLER_H
