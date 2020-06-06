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
    publisher_       = nh.advertise<geometry_msgs::Transform>(publishTopic, queue_size);

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

    // curr_frame is the current frame being processed
    Mat curr_frame = cv_ptr->image;

    vector<Point2f> curr_points, lost_points;


    if (need_init){ // If no features have been detected yet
        goodFeaturesToTrack(curr_frame, curr_points, MAX_CORNERS, KLT_QUALITY, KLT_MIN_DIST, Mat(), 3, false, 0.04);
        prev_frame[0]=curr_frame.clone();
        prev_keyframe[0]=curr_frame.clone();
        prev_points[0]=curr_points;
        prev_keypoints[0]=curr_points;
        need_init = false;
    }

    vector<uchar> status;
    vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 500, 1);
    //TermCriteria criteria = TermCriteria(TermCriteria::COUNT, 500, DBL_MAX);

    calcOpticalFlowPyrLK(prev_frame[0], curr_frame, prev_points[0], curr_points, status, err, Size(40, 40), 3,
                         criteria);

    //get number of persistent features
    uint16_t num_kept = 0;
    for (uint16_t i = 0; i < status.size(); i++) {
        num_kept = num_kept + status[i];
        if (status[i] == 0) {
            lost_points.push_back(prev_points[0][i]);
        }
    }

    if (num_kept < RETRACK_THRESH){ //only find features when enough are lost
        goodFeaturesToTrack(curr_frame, curr_points, MAX_CORNERS, KLT_QUALITY, KLT_MIN_DIST, Mat(), 3, false, 0.04);
        ROS_INFO("%d features remaining. Retracking", num_kept);
    } else{ //otherwise check if the relative distance is above threshold

        // use Nister's 5-point algorithm to find relative rotation and translation
        Mat E, R, t, mask;
        Mat triang_points;
        Mat points3D;
        E = findEssentialMat(curr_points,prev_points[0], camera_matrix, RANSAC, 0.999,1.0, mask);
        recoverPose(E, curr_points, prev_points[0], camera_matrix, R, t, 1, mask, triang_points);

        //TODO: Verify triang_points

        //TODO: Determine distance metric
        double dist = 0;//placeholder
        //median of norms of distances between points
        //or use R, t from Nister's along with scale factor

        if (dist < KEYFRAME_THRESH){
            //retrack between keyframes
            calcOpticalFlowPyrLK(prev_keyframe[0], curr_frame, prev_keypoints[0], curr_points, status, err, Size(40, 40), 3,
                                 criteria);
            E = findEssentialMat(curr_points,prev_points[0], camera_matrix, RANSAC, 0.999,1.0,mask);
            recoverPose(E, curr_points, prev_points[0], camera_matrix, R, t, 1, mask, triang_points);
            convertPointsFromHomogeneous(triang_points.t(), points3D);
            //cout << points3D << endl;
            Mat R_vec1, t_vec1, R_vec2, t_vec2;

            //TODO: Verify 3d world points
            solvePnPRansac(points3D, prev_keypoints[0], camera_matrix, {}, R_vec1, t_vec1, true, RANSAC_ITERATIONS);
            solvePnPRansac(points3D, curr_points, camera_matrix, {}, R_vec2, t_vec2, true, RANSAC_ITERATIONS);
            //TODO: Publish pose
            publishTracking(R,t);

            for (int i = STORED_KEYFRAMES-1; i > 0; i--){
                prev_keypoints[i] = prev_keypoints[i-1];
                prev_keyframe[i] = prev_keyframe[i-1];
            }
            prev_keyframe[0] = curr_frame.clone();
            prev_keypoints[0] = curr_points;
        }
    }
    //make this calculate only for each keyframe

    Mat img;
    cvtColor(curr_frame,img,CV_GRAY2BGR);
    Mat feature_mask = Mat::zeros(img.size(), img.type());
    for (int i = 0; i < curr_points.size(); i++){
        circle(feature_mask, curr_points[i], 5, Scalar(0,0,255), 1);
        line(feature_mask, curr_points[i], prev_points[0][i], Scalar(0,0,255), 2);
    }
    add(img,feature_mask,img);

    imshow("Optical Flow", img);
    waitKey(3);

    for (int i = STORED_FRAMES-1; i > 0; i--){
        prev_points[i] = prev_points[i-1];
        prev_frame[i] = prev_frame[i-1];
    }
    prev_frame[0] = curr_frame.clone();
    prev_points[0] = curr_points;
}

void OpticalFlowNode::publishTracking(const cv::Mat& Rot, const cv::Mat& trans) {
    geometry_msgs::Vector3 translation;
    geometry_msgs::Quaternion rotation;
    geometry_msgs::Transform pose_update;
    //TODO: 3x3 rotation matrix to Quaternion form
    //TODO: 1x3 translation to Vector3 form
    pose_update.translation = translation;
    pose_update.rotation = rotation;
    publisher_.publish(pose_update);
}