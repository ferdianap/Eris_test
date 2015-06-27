#ifndef SAL_DET_NODE_HPP_
#define SAL_DET_NODE_HPP_

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <visor/blobInfo.h> // Published message
#include <visor/call.h> // Service

#include <visor/Vattn/vattn.hpp>

cv_bridge::CvImageConstPtr curr_bgr, curr_gray; // Latest value of Img
ros::Time curr_header;
// cv::Size imgSize; // Not used probably
std::vector<cv::Rect> bblobs;
ros::Publisher blob_pub;
sensor_msgs::Image publishedSceneMsg;
const std::string ORI_CAPTURED_IMG = "Last Scene";
const std::string MOVT_DETECTOR = "Movement";
const std::string SALIENCY_MAP = "Saliency";
const std::string LIVE_FEED = "Live Feed";
// WINDOW MANAGEMENT VARS
int win_ref_x = 500;
int img_col = 160;
int img_row = 100;
int title_bar = 30;
cv::Point row1_col1(win_ref_x,win_ref_x);
cv::Point row1_col2(win_ref_x+img_col,
                    win_ref_x);
cv::Point row1_col3(win_ref_x+(img_col*2),
                    win_ref_x);
cv::Point row1_col4(win_ref_x+(img_col*3),
                    win_ref_x);
cv::Point row2_col1(win_ref_x,
                    win_ref_x+img_row+title_bar);
cv::Point row2_col2(win_ref_x+img_col,
                    win_ref_x+img_row+title_bar);
cv::Point row3_col3(win_ref_x+(img_col*2),
                    win_ref_x+(img_row+title_bar)*2);

class SaliencyDet {
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::ServiceServer service_;
	visor::blobInfo blob_info_; // Message var

	Vattn va;
	int max_blob_cnt_;
	float entropy_ratio_;
public:
	SaliencyDet(int max_blob_cnt, double entropy_ratio);
	~SaliencyDet();

	/**
	 * Node Callback
	 * Store the latest value of img in the global var 
	 *
	 **/
	void saliencyCallback(const sensor_msgs::ImageConstPtr& img_msg);

	/**
	 * Service Callback
	 * Do the image processing HERE ! 
	 *
	 **/
	bool serviceCallback(
		visor::call::Request  &req,
		visor::call::Response &res
	);

	void assignInfoToMSG(const ros::Time &curr_header);
};


#endif