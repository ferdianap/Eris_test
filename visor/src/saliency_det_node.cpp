/**
 * Saliency Detection Node
 *
 * This node performs localization based on  
 *   Visual Attention Algorithm from an HSV image.
 * It yields a saliency map,
 *   and localization info of each detected entities.
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <visor/saliency_det_node.hpp>


SaliencyDet::SaliencyDet(int max_blob_cnt, double entropy_ratio)
	: it_(nh_)
{
	// 1. init vars for va members
	va.setMaxBlobCount(max_blob_cnt);//20;//10;//4;
	va.setEntropyRatio((float)entropy_ratio);//0.7;//0.5;//0.3;
	va.setWeightIntensity(.0005);
	va.setWeightColor(.3);
	va.setWeightForm(.01);
	va.setChannels(3);
	va.setDepthBit(8);
	va.setMaskMin(3);
	va.setMaskMax(100);
	image_sub_ = it_.subscribe("/processed_img/bgr8_img", 1, 
		&SaliencyDet::saliencyCallback, this);
	blob_pub = nh_.advertise<visor::blobInfo>("/blob_info",1);
	// Init Display for live feed
    cv::namedWindow( LIVE_FEED, CV_WINDOW_AUTOSIZE);
    cv::moveWindow(LIVE_FEED, row1_col1.x, row1_col1.y);
	// SERVICE
	service_ = nh_.advertiseService("vattn_server",&SaliencyDet::serviceCallback, this);
	ROS_INFO("Saliency Detection Server is Ready.");
}

SaliencyDet::~SaliencyDet() {}

void SaliencyDet::saliencyCallback(const sensor_msgs::ImageConstPtr& img_msg) {
	try { // Store the latest image value to a Global var
		curr_bgr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
		curr_gray = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
		curr_header = curr_bgr->header.stamp;
		cv::imshow(LIVE_FEED, curr_bgr->image);
		cvWaitKey(30);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

bool SaliencyDet::serviceCallback(
	visor::call::Request  &req,
	visor::call::Response &res
){
	// ROS_WARN_STREAM("SRV CALLBACK " << req.call << " " << max_blob_count);
	res.resp = "Processing Image now (vattn_server)";

	try { // Process img here!
		// Convert cv_bridge::CvImageConstPtr to cv::Mat
		const cv::Mat cbgr = curr_bgr->image;
		const cv::Mat cgray = curr_gray->image;
		// 2. assign cbgr and cgray to member of va
		// va.setCurrBGR(cbgr); // input should be cv::Mat
		va.setCurrGray(cgray);

		static uint64_t c;
		if (c++ == 0 ) { // One time loop
			cv::namedWindow( MOVT_DETECTOR, CV_WINDOW_AUTOSIZE );
			cv::moveWindow(MOVT_DETECTOR, row2_col1.x, row2_col1.y);

			cv::namedWindow( SALIENCY_MAP, CV_WINDOW_AUTOSIZE);
			cv::moveWindow(SALIENCY_MAP, row1_col2.x, row1_col2.y);

			cv::namedWindow( ORI_CAPTURED_IMG, CV_WINDOW_AUTOSIZE );
			cv::moveWindow(ORI_CAPTURED_IMG, row1_col3.x, row1_col3.y);
		}
		// 3. call the mainmethod to start processing as many as you need
		// need a BGR8 input of cv::Mat
		// no need to set the BGR as the va member var
		// but currGray must be set as va member var
		va.mainmethod(cbgr);

        cv::imshow(ORI_CAPTURED_IMG, va.getStillImg());
        cv::imshow(MOVT_DETECTOR, va.getDiffGray()); 
        cv::imshow(SALIENCY_MAP, va.getSalmapImg());
		// cv::imshow(DIFF_SALMAP,va.getDiffSalmapImg());
		cv::waitKey(30);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return false;
	}
	bblobs.clear();
    bblobs = va.getFinalDetectedObj();
    assignInfoToMSG(curr_header);
	blob_pub.publish(blob_info_);
    
	return true;
}

void SaliencyDet::assignInfoToMSG( const ros::Time &time ) {
    blob_info_.header.stamp = time;//img_msg->header.stamp; //ros::Time::now();
    blob_info_.minx.clear();
    blob_info_.maxx.clear();
    blob_info_.miny.clear();
    blob_info_.maxy.clear();
    blob_info_.count = bblobs.size(); 

    for (int i=0; i<blob_info_.count; i++) {
    	blob_info_.minx.push_back(bblobs[i].x);
    	blob_info_.maxx.push_back(bblobs[i].x+bblobs[i].width);
    	blob_info_.miny.push_back(bblobs[i].y);
    	blob_info_.maxy.push_back(bblobs[i].y+bblobs[i].height);
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "image_processing_node");
    ros::NodeHandle nhp("~");
    int max_blob_cnt;
	double entropy_ratio;
	// nhp.getParam("somefloat2", max_blob_cnt);
    nhp.param(std::string("maxblobcount"), max_blob_cnt, 20);
	nhp.param(std::string("entropyratio"), entropy_ratio, 0.7); 
	SaliencyDet imgproc(max_blob_cnt, entropy_ratio);
	ros::spin();
	return 0;
}
