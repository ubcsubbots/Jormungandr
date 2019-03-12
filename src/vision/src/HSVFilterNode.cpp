/*
 * Created By: Reid Oliveira
 * Created On: November 11th, 2017
 * Description: Performs an HSV filter on incoming data and republishes it
 */

#include <HSVFilterNode.h>

HSVFilterNode::HSVFilterNode(int argc, char** argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    image_transport::ImageTransport it(nh);

    std::string subscribeTopic = "/camera/image_raw";
    std::string publishTopic   = "/vision/output";

    //Bool to retrieve from the param-server that was loaded in the launch file.
    bool isDynRecon;

    //the parameters we care about to be pulled from rosparam server
    int h_high, h_low, v_high, v_low, s_high, s_low;

    //Load all the parameters from the rosparam server
    nh.getParam("/" + node_name + "/h_high",h_high);
    nh.getParam("/" + node_name + "/h_low",h_low);
    nh.getParam("/" + node_name + "/v_high",v_high);
    nh.getParam("/" + node_name + "/v_low",v_low);
    nh.getParam("/" + node_name + "/s_high",s_high);
    nh.getParam("/" + node_name + "/s_low",s_low);
    nh.getParam("/" + node_name + "/isDynRecon",isDynRecon);

    //Create a new filter object with the values pulled from the rosparam server (which is configured in the launch file)
    filter_ = HSVFilter(h_low,h_high,s_low,s_high,v_low,v_high);

    //If this line is inside the if statement, this node doesnt show up on the dynamic reconfigure GUI
    dynamic_reconfigure::Server<vision::hsvfilterConfig> server;

    if(isDynRecon) {
        dynamic_reconfigure::Server<vision::hsvfilterConfig>::CallbackType f;
        f = boost::bind(&HSVFilterNode::dynamicreconfigCallback,this, _1, _2);
        server.setCallback(f);
    }else{
        //If the Dynamic Reconfigure is not desired, then set all the parameters in the server back to the original values
        //Not needed but looks better
        nh.setParam("/" + node_name + "/h_high",h_high);
        nh.setParam("/" + node_name + "/h_low",h_low);
        nh.setParam("/" + node_name + "/v_high",v_high);
        nh.setParam("/" + node_name + "/v_low",v_low);
        nh.setParam("/" + node_name + "/s_high",s_high);
        nh.setParam("/" + node_name + "/s_low",s_low);
    }


    int refresh_rate = 1;
    subscriber_      = it.subscribe(
    subscribeTopic, refresh_rate, &HSVFilterNode::subscriberCallBack, this);

    int queue_size = 1;
    publisher_     = it.advertise(publishTopic, queue_size);

    //system("echo 'ROS Params dumped at: ' $(pwd); rosparam dump brysonDump.yaml");

    // Start up ros. This will continue to run until the node is killed
    ros::spin();
}


void HSVFilterNode::subscriberCallBack(
const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat filtered;
    filter_.apply(cv_ptr->image, filtered);
    publishFilteredImage(filtered);
}

void HSVFilterNode::publishFilteredImage(const cv::Mat& filtered_image) {
    publisher_.publish(
    cv_bridge::CvImage(std_msgs::Header(), "mono8", filtered_image)
    .toImageMsg());
}

void HSVFilterNode::dynamicreconfigCallback(
const vision::hsvfilterConfig& config, uint32_t level) {

    ROS_INFO("Reconfigure Request: %i %i %i %i %i %i",
             config.h_low,
             config.s_low,
             config.v_low,
             config.h_high,
             config.s_high,
             config.v_high);

    filter_ = HSVFilter(config.h_low,
                        config.h_high,
                        config.s_low,
                        config.s_high,
                        config.v_low,
                        config.v_high);

}
