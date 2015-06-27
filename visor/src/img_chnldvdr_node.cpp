/**
 * Image Channel Divider Node
 *
 * This node divides the resized image from the 'Image Resizer Node'
 * into a BGR8 and HSV image.
 *
 */ 

#include <visor/img_chnldvdr_node.hpp>

ImageChannelDivider::ImageChannelDivider()
  : it_(nh_)
{
	image_sub_ = it_.subscribe("/resized_img/image_raw", 1, 
		&ImageChannelDivider::dividerCallback, this);
	bgr_image_pub_ = it_.advertise("/processed_img/bgr8_img", 1);
	hsv_image_pub_ = it_.advertise("/processed_img/hsv_img",1);
}

ImageChannelDivider::~ImageChannelDivider() {
}

void ImageChannelDivider::dividerCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv_bridge::CvImagePtr cv_hsv;
	cv_bridge::CvImageConstPtr cv_bgr;

	try {
		cv_hsv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cv_bgr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::cvtColor(cv_hsv->image, cv_hsv->image, CV_BGR2HSV);

	bgr_image_pub_.publish(cv_bgr->toImageMsg());
	hsv_image_pub_.publish(cv_hsv->toImageMsg());
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_channel_divider");
	ImageChannelDivider icd;
	ros::spin();
	return 0;
}