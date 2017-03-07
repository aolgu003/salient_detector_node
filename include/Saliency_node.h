#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <salient_detector/uav_targets.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include "Saliency.h"

#include <boost/tokenizer.hpp>

static const std::string g_ROBOT_DESCRIPTION = "robot_description";
// name of the robot description (a param name, so it can be changed

class SaliencyNode {
	public:
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

	private:
    ros::NodeHandle nh_;
    ros::Publisher pub_; 
    ros::Subscriber cam_img_sub_;
    ros::Subscriber uav_imu_;
    ros::Subscriber uav_pos_;
    sensor_msgs::Imu current_pose_;
};
