/*
 * Created By: Logan Fillo
 * Created On: March 5th, 2019
 * Description: A node which subscribes to /uwsim/g500_odom (the simulation robot's pose and twist)
 *             and prints a string specifiying if the robot made it through the gate in a given
 *             amount of time.
 *
 */

GateTestResultNode::GateTestResultNodeint argc, char** argv, std::string node_name) {
  // Setup Nodehandle
  ros::init(argc, argv, "test result");
  ros::NodeHandle nh;

  // Setup subscriber
  std::string topic = "/uwsim/g500_odom";
  int refresh_rate  = 10;
  ros::Subscriber sim_sub = nh.subscribe(
    topic, refresh_rate, &odometryCallBack);
}

void GateTestResultNode::odometryCallBack(
  const nav_msgs::Odometry::ConstPtr& msg){

  ROS_INFO("[HEARD]: ODOMETRY_MESSAGE");
  currMessage = msg;
}
