/**
 * Image Resizer Node
 *
 * This node gets a live image feed from a single USB Cam.
 * Useful for reducing computational time.
 *
 */ 

#include <visor/img_resizer_node.hpp>

ImageResizer::ImageResizer(
	ros::NodeHandle &n,
	std::string camera_base_topic,
	std::string resized_base_topic,
	int resized_width,
	int resized_height,
	bool convert_to_bw )
	: nh_(n),
	it_(n)
{
	resized_width_ = resized_width;
	resized_height_ = resized_height;
	convert_to_bw_ = convert_to_bw;
	image_sub_ = it_.subscribe(camera_base_topic, 1,
		&ImageResizer::resizerCallback, this);
	image_pub_ = it_.advertise(resized_base_topic, 1, true);
	std::string transport_in = image_sub_.getTransport();
	ROS_INFO("Subscribed using %s for transport", transport_in.c_str());
}

ImageResizer::~ImageResizer() {
}

void ImageResizer::resizerCallback(const sensor_msgs::ImageConstPtr& img) {
	// Convert to OpenCV
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(img);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Make destination
	cv::Mat resized;
	if (convert_to_bw_) {
		// Drop to 1-channel
		resized = cv::Mat(cv::Size(resized_width_, resized_height_), CV_8UC1);
	} else {
		// Keep RGB channels
		resized = cv::Mat(cv::Size(resized_width_, resized_height_), 
			cv_bridge::getCvType(img->encoding));
	}
	// Remove color if desired
	cv::Mat intermediate;
	if (convert_to_bw_) {
		// Convery to grayscale
		// Pick which conversion to use depending on the source encoding
		if (img->encoding == sensor_msgs::image_encodings::RGB8   ||
			img->encoding == sensor_msgs::image_encodings::RGB16  ){
			cv::cvtColor(cv_ptr->image, intermediate, CV_RGB2GRAY);
		} else if (	img->encoding == sensor_msgs::image_encodings::RGBA8	||
					img->encoding == sensor_msgs::image_encodings::RGBA16	){
			cv::cvtColor(cv_ptr->image, intermediate, CV_RGBA2GRAY);
		} else if (	img->encoding == sensor_msgs::image_encodings::BGR8 	||
					img->encoding == sensor_msgs::image_encodings::BGR16 	){
			cv::cvtColor(cv_ptr->image, intermediate, CV_BGR2GRAY);
		} else if ( img->encoding == sensor_msgs::image_encodings::BGRA8 	||
					img->encoding == sensor_msgs::image_encodings::BGRA16 ){
			cv::cvtColor(cv_ptr->image, intermediate, CV_BGRA2GRAY);
		} else {
			ROS_WARN("Unable to match image encoding to an OpenCV conversion option,"
					" using CV_BGR2GRAY as a default");
			cv::cvtColor(cv_ptr->image, intermediate, CV_BGR2GRAY);
		}
	} else {
		// Keep RGB
		intermediate = cv_ptr->image;
	}
	// Resize image
	if (resized_width_ < img->width && resized_height_ < img->height) {
		// If we're resizing smaller, use CV_INTER_AREA interpolation
		cv::resize(intermediate, resized, resized.size(), 0.0, 0.0, CV_INTER_AREA);
	} else {
		// If we're resizing bigger, use CV_INTER_LINEAR interpolation
		cv::resize(intermediate, resized, resized.size(), 0.0, 0.0, CV_INTER_LINEAR);
	}
	// Convert back to ROS
	sensor_msgs::Image resized_image;
	cv_bridge::CvImage converted;
	if (convert_to_bw_) {
		converted = cv_bridge::CvImage(img->header, "mono8", resized);
	} else {
		converted = cv_bridge::CvImage(img->header, img->encoding, resized);
	}
	converted.toImageMsg(resized_image);
	// Republish
	image_pub_.publish(resized_image);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_resizer");
	ros::NodeHandle nh;
	ros::NodeHandle nhp("~");
	std::string camera_base_topic;
	std::string resized_base_topic;
	int resized_width;
	int resized_height;
	bool convert_to_bw;
	nhp.param(std::string("camera_base_topic"), camera_base_topic,
		std::string("/usb_cam/image_raw")); // DEFAULT FOR USB CAM
	nhp.param(std::string("resized_base_topic"), resized_base_topic, 
		std::string("/resized_img/image_raw"));
	nhp.param(std::string("resized_width"), resized_width, 160); // 1/4 x
	nhp.param(std::string("resized_height"), resized_height, 120);
	nhp.param(std::string("convert_to_bw"), convert_to_bw, false);
	ImageResizer resizer(nh, camera_base_topic, resized_base_topic,
		resized_width, resized_height, convert_to_bw);
	ros::spin();
	return 0;
}