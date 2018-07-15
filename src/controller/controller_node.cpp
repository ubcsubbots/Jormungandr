/*
 * Created By: Viral Galaiya
 * Created On: July 7th 2018
 * Description: The LQR controller with the intergrator, dertmined using Simulink. Takes in the Ros geometry message and returns a//  ros cutom message with the PWM
 */

//#include <Controller.h>

int main(int argc, char** argv) {
  // Setup your ROS node
  std::string node_name = "controller";

  // Create an instance of your class
  Controller controller(argc, argv, node_name);

  //     // Start up ros. This will continue to run until the node is killed
  ros::spin();

  //     // Once the node stops, return 0
  return 0;
}
