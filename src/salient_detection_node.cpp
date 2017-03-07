#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <salient_detector/uav_targets.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include "Saliency.h"



void imageCallback(const sensor_msgs::ImageConstPtr& color_img);
void geolocation(const sensor_msgs::Imu);

ros::Publisher pub; 
ros::Subscriber cam_img_sub;
ros::Subscriber uav_imu;
ros::Subscriber uav_pos;



int main(int argc, char **argv) {
	// Initialize ros and create node handle
	ros::init(argc, argv, "target_detector");
	ros::NodeHandle nh;
	
	pub = nh.advertise<salient_detector::uav_targets>("/salient_detector/targets", 10);
	cam_img_sub = nh.subscribe("/cv_camera/image_raw", 10, &imageCallback);

	ros::spin();
}

void imageCallback(const sensor_msgs::ImageConstPtr& color_img) {
	//Create uav target object
	cv_bridge::CvImagePtr img_ptr;
	cv_bridge::CvImage ros_img;
	ros_img.header = color_img->header;
	ros_img.encoding = sensor_msgs::image_encodings::BGR8;

	cv::Mat img_rgb, Lab, salient_map;
	Saliency saliency_engine;
	
	vector<Rect> roi;
	vector<Mat> targets;	
	vector<sensor_msgs::RegionOfInterest> ros_roi;
	sensor_msgs::RegionOfInterest object;
	salient_detector::uav_targets uav_packet;
	sensor_msgs::Image target_image;
  image_geometry::PinholeCameraModel camera_model;
  cv::Point3d unit_vector;
  double phi, psi;
	
	try {
		img_ptr = cv_bridge::toCvCopy( color_img, sensor_msgs::image_encodings::BGR8 );
		img_rgb = img_ptr->image;
		
		cvtColor(img_rgb, Lab, 	CV_BGR2Lab);

		saliency_engine.spectral_saliency_multi(Lab, salient_map);
		saliency_engine.find_targets( img_rgb, salient_map,  roi, targets, .2, 10, 20, 1, 2);

		// Resize image original image and convert to sensor message
		
		// Create array of GPS coords

		for (int i = 0; i < roi.size(); i++) {
			object.height = roi[i].width;
			object.width = roi[i].height;
			object.x_offset = roi[i].x;
			object.y_offset = roi[i].y;
      
      unit_vector = camera_model.projectPixelTo3dRay( cv::Point2d( roi[i].x + roi[i].width/2, 
                                                                   roi[i].y - roi[i].height/2 ) );

      phi = atan2(unit_vector.y, unit_vector.x);
      psi = atan2(unit_vector.z, unit_vector.x);
      
			ros_img.image = targets[i];
			ros_img.toImageMsg(target_image);
 			//target_image = ros_img.toImageMsg();

			uav_packet.roi.push_back( object );
			uav_packet.targets.push_back( target_image );

			// Create array of regions of interest
			rectangle( img_rgb, roi[i],  Scalar(0, 255, 0) );
		}
		
		pub.publish(uav_packet);
		cv::imshow("Camera output", img_rgb);
		cv::imshow("Saliency map", salient_map);
		cv::waitKey(3);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: \%s", e.what());
		return;
	}

}
