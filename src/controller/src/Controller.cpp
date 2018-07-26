/*
 * Created By: Viral Galaiya
 * Created On: July 23, 2018
 * Description:
 */
#include "Controller.h"

const double Controller::PERIOD = 0.02;

Controller::Controller():imu_angular_velocity_(3,1),
                         imu_angular_accelaration_(3,1),
			 linear_velocity_(3,1),
			 imu_linear_accelaration_(3,1),
			 previous_imu_angular_velocity_(3,1),
			 previous_imu_linear_accelaration_(3,1),
			 desired_velocity_(6,1),
			 previous_y_(6,1),
			 previous_desired_velocity_(6,1),
			 intergrator_acumulator_(6,1),
			 current_velocity_(6,1),
			 current_acceleration_(6,1),
			 x_(12,1),
			 k_(4,12),
			 ki_(4,6),
			 y_(6,1),
			 torque_matrix_(4,1),
			 pwm_matrix_(4,1){
  previous_imu_angular_velocity_ << 0, 0, 0;
  linear_velocity_ << 0, 0, 0;
  previous_imu_linear_accelaration_ << 0, 0, 0;
  previous_y_ << 0, 0, 0, 0, 0, 0;
  desired_velocity_ << 0, 0, 0, 0, 0, 0;
  previous_y_ << 0, 0, 0, 0, 0, 0;
  previous_desired_velocity_ << 0, 0, 0, 0, 0, 0;
  intergrator_acumulator_ << 0, 0, 0, 0, 0, 0;
  ki_ <<
      -4.8922,    4.7175,    0.0424,   -0.0000,   -0.0000,   -1.6231,
      -4.1080,   -5.8281,   -0.0439,    0.0000,    0.0000,    1.7127,
       0.1307,   -2.5481,   -1.8883,    0.0000,    0.0000,   -3.2061,
      -0.1489,    2.8989,   -1.8216,   -0.0000,    0.0000,    3.2437;
  k_ <<
          0.333,   -0.1862,   -0.4010 ,  -0.1260,    0.0019,    0.1623,    1.4647,   -0.6226 ,  -0.0760,    0.2136,    0.0016,    0.1497,
          0.280,    0.2301,    0.4448 ,   0.1284,   -0.0019,   -0.1713,    1.4973,    0.6371 ,   0.0845,   -0.2133,    0.0015,   -0.1565,
         -0.008,    0.1006,   14.5062 ,   0.1082,   -0.0050,    0.3206,   -0.0413,    0.8910 ,   2.6541,   -0.0839,    0.0449,    0.3121,
          0.010,   -0.1144,   13.8843 ,   0.8328,   -0.0030,   -0.3244,    0.0462,   -0.9390 ,   2.7067,    0.0779,    0.0449,   -0.3149;



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
void Controller::setDepthData(double depth){
  depth_ = depth;
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

  y_ << current_velocity_(0, 0),
    current_velocity_(1, 0),
    depth_,  //Current_velocity_(2, 0)
    current_velocity_(3, 0),
    current_velocity_(4, 0),
    current_velocity_(5, 0);
  x_ << current_velocity_(0, 0),
    current_velocity_(1, 0),
    current_velocity_(2, 0), current_velocity_(3, 0),
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
    pwm_matrix_(1, 0) = 0;
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
    pwm_matrix_(2, 0) = 0;
  }

  // keep the PWM values within required range
  if (pwm_matrix_(0,0)> 1900){
    pwm_matrix_(0,0) = 1900;
  } else if (pwm_matrix_(0,0)< 1100){
    pwm_matrix_(0,0) = 1100;
  }
  if (pwm_matrix_(1,0)> 1900){
    pwm_matrix_(1,0) = 1900;
  } else if (pwm_matrix_(1,0)< 1100){
    pwm_matrix_(1,0) = 1100;
  }
  if (pwm_matrix_(2,0)> 1900){
    pwm_matrix_(2,0) = 1900;
  } else if (pwm_matrix_(2,0)< 1100){
    pwm_matrix_(2,0) = 1100;
  }
  if (pwm_matrix_(3,0)> 1900){
    pwm_matrix_(3,0) = 1900;
  } else if (pwm_matrix_(3,0)< 1100){
    pwm_matrix_(3,0) = 1100;
  }

  previous_imu_linear_accelaration_ = imu_linear_accelaration_;
  previous_imu_angular_velocity_    = imu_angular_velocity_;
  desired_velocity_                 = previous_desired_velocity_;
  previous_y_                       = y_;

  return torque_matrix_;
}
