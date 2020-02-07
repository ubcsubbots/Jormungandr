/*
 * Created By: Kevin Huang
 * Created On: February 2nd, 2020
 * Description: Performs visual odometry
 */

#include <OpticalFlowNode.h>

OpticalFlowNode::OpticalFlowNode(int argc, char** argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    image_transport::ImageTransport it(nh);
    need_init = true;

    std::string subscribeTopic = "/camera/image_raw";
    std::string publishTopic   = "/visual_odometry/output";

    int refresh_rate = 1;
    subscriber_      = it.subscribe(
            subscribeTopic, refresh_rate, &OpticalFlowNode::subscriberCallBack, this);

    int queue_size = 1;
    publisher_     = it.advertise(publishTopic, queue_size);

    ROS_INFO("Starting Node");

    namedWindow("Optical Flow");

    // Start up ros. This will continue to run until the node is killed
    ros::spin();
}

void OpticalFlowNode::subscriberCallBack(const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat next_frame = cv_ptr->image;

    vector<Point2f> next_points, flow_points;

    goodFeaturesToTrack(next_frame, next_points, MAX_CORNERS, DBL_MIN, 1, Mat(), 3, false, 0.04);

    if (need_init){
        prev_frame=next_frame;
        prev_points[0]=next_points;
        need_init = false;
    }

    vector<uchar> status;
    vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 500, 1);
    //TermCriteria criteria = TermCriteria(TermCriteria::COUNT, 500, DBL_MAX);

    flow_points = next_points; //gets modified by optical flow function so make a copy
    //if you don't make a copy it'll only track features existing in the first frame; might be some middle ground here
    calcOpticalFlowPyrLK(prev_frame, next_frame, prev_points[0], flow_points, status, err, Size(40,40), 3, criteria);

    Mat img;
    cvtColor(next_frame,img,CV_GRAY2BGR);
    Mat feature_mask = Mat::zeros(img.size(), img.type());
    for (int i = 0; i < flow_points.size(); i++){
        circle(feature_mask, flow_points[i], 5, Scalar(0,0,255), 1);
        line(feature_mask, flow_points[i], prev_points[0][i], Scalar(0,0,255), 2);
    }
    add(img,feature_mask,img);

    imshow("Optical Flow", img);
    waitKey(3);

    prev_frame = next_frame.clone();
    for (int i = 4; i > 0; i--){
        prev_points[i] = prev_points[i-1];
    }
    prev_points[0] = next_points; //points before optical flow function
/*
    //epipolar check + ransac
    //p3p
    //map
    //triangulation
*/
    publishTracking(next_frame);//placeholder
}

void OpticalFlowNode::publishTracking(const cv::Mat& filtered_image) {
    publisher_.publish(
            cv_bridge::CvImage(std_msgs::Header(), "mono8", filtered_image)
                    .toImageMsg());
}