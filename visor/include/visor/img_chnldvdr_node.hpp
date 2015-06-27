#ifndef IMG_CHNLDVDR_NODE_HPP_
#define IMG_CHNLDVDR_NODE_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class ImageChannelDivider {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher bgr_image_pub_;
	image_transport::Publisher hsv_image_pub_;
	
public:
	ImageChannelDivider();
	~ImageChannelDivider();
	void dividerCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif