#ifndef ATTN_WINDOW_NODE_HPP_
#define ATTN_WINDOW_NODE_HPP_

#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// FOR IMAGE
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/Image.h>
// FOR OTHER TYPES OF MSG
#include <visor/blobInfo.h>
#include <visor/detEntityArray.h>

static const std::string OPENCV_WINDOW = "LOCAL AREA BLOB"; 
static const char IMG_WINDOW[] = "Cropped Captured";


class AttnWindow {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	bool display_first_object_;
	typedef image_transport::SubscriberFilter ImageSubscriber;
	ImageSubscriber img_sub_;
	// The following lines are for multiple topics subscription ************
	message_filters::Subscriber<visor::blobInfo> blob_sub_;

	typedef message_filters::sync_policies::ApproximateTime<
		sensor_msgs::Image,
		visor::blobInfo
	> MySyncPolicy;

	message_filters::Synchronizer< MySyncPolicy > sync;
	//**********************************************************************
	ros::Publisher det_ent_pub_;
	visor::detEntityArray dea_; // declared msg
public:
	AttnWindow(bool view_object);
	~AttnWindow();

	void attnWindowCallback(
		const sensor_msgs::ImageConstPtr& img_msg,
		const visor::blobInfo& blob_msg
	);
};

#endif