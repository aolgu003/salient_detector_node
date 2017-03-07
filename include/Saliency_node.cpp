SaliencyNode();
~SaliencyNode();

//! Publish the message.
void publishMessage(ros::Publisher *pub_message);
void poseLogCallback();
void imageCallback();

//Callback for each image that is caputured
void imageCaptureCallback(const sensor_msgs::ImageConstPtr& color_img);

//Callback that grabs pose information and handles it
void uavPoseCallback(const sensor_msgs::Imu& uav_pose);

void gimbalTwistCallback(const sensor_msgs::Imu& gimbal_pose);
