/*
 * Created By: Viral Galaiya
 * Created On: July 23, 2018
 * Description:
 */
#include "Controller.h"

const double Controller::PERIOD = 0.02;

Controller::Controller() {
    previous_imu_angular_velocity_ << 0, 0, 0;
    linear_velocity_ << 0, 0, 0;
    previous_imu_linear_accelaration_ << 0, 0, 0;
    previous_y_ << 0, 0, 0, 0, 0, 0;
    desired_velocity_ << 0, 0, 0, 0, 0, 0;
    previous_y_ << 0, 0, 0, 0, 0, 0;
    previous_desired_velocity_ << 0, 0, 0, 0, 0, 0;
    intergrator_acumulator_ << 0, 0, 0, 0, 0, 0;
    ki_ << -5.0075, 4.6995, 0.0742, -0.0000, 0.0000, -1.6221, -4.1898, -5.8310,
    -0.0892, 0.0000, -0.0000, 1.7134, 0.1424, -2.3458, -1.6064, -0.0000,
    -0.0000, -3.2109, -0.1494, 3.0911, -1.5075, -0.0000, 0.0000, 3.2398;

    k_ << 0.1138, -0.1855, -0.0318, -0.1248, 0.0023, 0.1622, 1.5326, -0.7185,
    -0.0201, 0.2112, 0.0016, 0.1854, 0.0952, 0.2302, 0.0382, 0.1272, -0.0020,
    -0.1713, 1.5493, 0.7596, 0.0245, -0.2109, 0.0015, -0.1943, -0.0032, 0.0926,
    0.6885, 0.1288, -0.0127, 0.3211, -0.0460, 0.8994, 0.3173, -0.0805, 0.0456,
    0.3836, 0.0034, -0.1220, 0.6461, 0.8552, -0.0102, -0.3240, 0.0473, -1.0260,
    0.4615, 0.0795, 0.0456, -0.3864;
}

void Controller::setImuData(double angular_x,
                            double angular_y,
                            double angular_z,
                            double linear_x,
                            double linear_y,
                            double linear_z) {
    imu_angular_velocity_ << angular_x, angular_y, angular_z;
    imu_linear_accelaration_ << linear_x, linear_y, linear_z;
}

Eigen::MatrixXd Controller::setDesiredVelocity(double linear_x,
                                               double linear_y,
                                               double angular_x,
                                               double angular_y,
                                               double angular_z,
                                               double position_z) {
    imu_angular_accelaration_ =
    (imu_angular_velocity_ - imu_angular_velocity_) / PERIOD;

    desired_velocity_ << linear_x, linear_y, position_z, angular_x, angular_y,
    angular_z;

    // Actual velocity to be got from the IMU and depth to be got from the
    // pressure/depth sensor
    linear_velocity_ =
    linear_velocity_ +
    (imu_linear_accelaration_ + previous_imu_linear_accelaration_) * PERIOD *
    0.5; // account for cummilation error and add reset

    current_velocity_ << imu_angular_velocity_.coeff(0, 0),
    imu_angular_velocity_.coeff(1, 0),
    imu_angular_velocity_.coeff(2, 0), // position, not velocity
    linear_velocity_.coeff(0, 0), linear_velocity_.coeff(1, 0),
    linear_velocity_.coeff(2, 0);

    current_acceleration_ << imu_angular_accelaration_.coeff(0, 0),
    imu_angular_accelaration_.coeff(1, 0),
    imu_angular_accelaration_.coeff(2, 0), // to be changed from vel to osition
    imu_linear_accelaration_.coeff(0, 0), imu_linear_accelaration_.coeff(1, 0),
    imu_linear_accelaration_.coeff(2, 0);

    // u = −K(x − r) − Ki*Y where r is Desiredvelocity and Y is desired velocity
    // u place holder

    // parameters used to get the K from LQI in MATLAB
    // Q= diag([3 3 3 3 3 3 3 3 3 3 3 3 44 76 7 3 3 30]*1); may need to change,
    // to heavy a weight on feedback and error on the intergrator (may?) be
    // super high
    // R = diag([1 1 1 1]);
    // N = eye(18,4)*1;

    // PWM matrix needs to be multiplied to get the PWM value, since its in
    // terms of torque rightnow

    y_ << current_velocity_(0, 0), current_velocity_(1, 0),
    current_velocity_(2, 0), current_velocity_(3, 0), current_velocity_(4, 0),
    current_velocity_(5, 0), x_ << current_velocity_(0, 0),
    current_velocity_(1, 0), current_velocity_(2, 0), current_velocity_(3, 0),
    current_velocity_(4, 0), current_velocity_(5, 0),
    current_acceleration_(0, 0), current_acceleration_(1, 0),
    current_acceleration_(2, 0), current_acceleration_(3, 0),
    current_acceleration_(4, 0), current_acceleration_(5, 0);

    intergrator_acumulator_ =
    intergrator_acumulator_ +
    ((desired_velocity_ + previous_desired_velocity_) / 2 -
     (y_ + previous_y_) / 2) *
    PERIOD;

    torque_matrix_ = -ki_ * intergrator_acumulator_ + -k_ * x_;
    // T100 equation (for <0 thrust) 1454 +298*x+56.7*x^2
    // T100 equation (for >0 thrust) 1543+179*-16.5*x^2
    // T200 equation (for <0 thrust) 1457+104*x+5.1+x^2
    // T200 equation (for >0 thrust) 1539+89.9*x-3.91*x^2

    // c convert to torque
    if (torque_matrix_(0, 0) < 0) {
        pwm_matrix_(0, 0) = 1454 + 298 * torque_matrix_(0, 0) +
                            56.7 * torque_matrix_(0, 0) * torque_matrix_(0, 0);
    } else if (torque_matrix_(1, 0) > 0) {
        pwm_matrix_(0, 0) = 1543 + 179 * torque_matrix_(0, 0) -
                            16.5 * torque_matrix_(0, 0) * torque_matrix_(0, 0);
    } else {
        pwm_matrix_(0, 0) = 0;
    }

    if (torque_matrix_(1, 0) < 0) {
        pwm_matrix_(1, 0) = 1454 + 298 * torque_matrix_(1, 0) +
                            56.7 * torque_matrix_(1, 0) * torque_matrix_(1, 0);
    } else if (torque_matrix_(1, 0) > 0) {
        pwm_matrix_(1, 0) = 1543 + 179 * torque_matrix_(1, 0) -
                            16.5 * torque_matrix_(1, 0) * torque_matrix_(1, 0);
    } else {
        pwm_matrix_(1, 0) = 0;
    }

    if (torque_matrix_(2, 0) < 0) {
        pwm_matrix_(2, 0) = 1454 + 104 * torque_matrix_(2, 0) +
                            5.1 * torque_matrix_(2, 0) * torque_matrix_(2, 0);
    } else if (torque_matrix_(2, 0) > 0) {
        pwm_matrix_(2, 0) = 539 + 89.9 * torque_matrix_(2, 0) -
                            3.91 * torque_matrix_(2, 0) * torque_matrix_(2, 0);
    } else {
        pwm_matrix_(2, 0) = 0;
    }

    if (torque_matrix_(3, 0) < 0) {
        pwm_matrix_(3, 0) = 1454 + 104 * torque_matrix_(3, 0) +
                            5.1 * torque_matrix_(3, 0) * torque_matrix_(3, 0);
    } else if (torque_matrix_(1, 0) > 0) {
        pwm_matrix_(3, 0) = 539 + 89.9 * torque_matrix_(3, 0) -
                            3.91 * torque_matrix_(3, 0) * torque_matrix_(3, 0);
    } else {
        pwm_matrix_(3, 0) = 0;
    }

    previous_imu_linear_accelaration_ = imu_linear_accelaration_;
    previous_imu_angular_velocity_    = imu_angular_velocity_;
    desired_velocity_                 = previous_desired_velocity_;
    previous_y_                       = y_;

    return torque_matrix_;
}
