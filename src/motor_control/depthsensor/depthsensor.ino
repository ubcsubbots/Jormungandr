#include <ros.h>
#include <std_msgs/Float32.h>

std_msgs::Float32 depth;

ros::NodeHandle nh;
ros::Publisher depth_pub("depth_publisher",&depth);


void setup(){

    nh.initNode();
    nh.advertise(depth_pub);
     Serial.begin(9600);
}
void loop(){
    
    depth.data=map(analogRead(1),0,1023,-15748,16732);  //depth in milimeters to avoid loss of resolution due to integer divisio
    Serial.println(map(analogRead(1),0,1023,-15748,16732));
    depth_pub.publish(&depth);
    nh.spinOnce();
    delay(100);
}
