/**
 * attn_window_node class provides the pre-processing
 * of cropping matched detected blob and the corresponding image.
 * This class should be called before the local features
 * were processed.
 * This class subscribes to both:
 *    /processed_img/bgr8_img and /blob_info
 * and publishes an array of images of detected entities
 *    /detected_ent_img
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visor/attn_window_node.hpp>
#include <visor/object_extractor.hpp>

AttnWindow::AttnWindow(bool view_object)
	: it_(nh_),
	img_sub_(it_, "/processed_img/bgr8_img",1),
	blob_sub_(nh_, "/blob_info",1),
	sync( MySyncPolicy( 1 ), img_sub_, blob_sub_
){
	display_first_object_ = view_object;
	sync.registerCallback( &AttnWindow::attnWindowCallback, this );
	det_ent_pub_ = nh_.advertise<visor::detEntityArray>("/detected_ent_img",1);

	cv::namedWindow( "test", CV_WINDOW_AUTOSIZE );
}

AttnWindow::~AttnWindow() {
}

void AttnWindow::attnWindowCallback(
	const sensor_msgs::ImageConstPtr& img_msg,
	const visor::blobInfo& blob_msg
){
	cv_bridge::CvImagePtr cv_img;
	try {
		cv_img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
		// CROPPING
		cv_bridge::CvImage finalCropped;
		sensor_msgs::Image finalImage;
    
		dea_.header.stamp = img_msg->header.stamp;
		dea_.count = blob_msg.count;
		dea_.entity.clear();
		dea_.pos_minx.clear();
		dea_.pos_miny.clear();
		dea_.pos_maxx.clear();
		dea_.pos_maxy.clear();
		dea_.pos_minx = blob_msg.minx;
		dea_.pos_miny = blob_msg.miny;
		dea_.pos_maxx = blob_msg.maxx;
		dea_.pos_maxy = blob_msg.maxy;
		ObjectExtractor oe;
    	oe.setBlobCount((int)blob_msg.count);
    	// Assign value manually to member vars
    	for (int i = 0; i < blob_msg.count; ++i) {
    		oe.setBlobsManually(
    			(int)blob_msg.minx[i],(int)blob_msg.miny[i],
    			(int)blob_msg.maxx[i],(int)blob_msg.maxy[i]
    			);
    	}
    	cv::Mat srcimg = cv_img->image;
    	oe.setSourceImg(srcimg); // should be cv::Mat
    	oe.computeROIs();
    	oe.processCroppedImgs();
    	for (int i=0; i<oe.getObjCount(); ++i) {
    		if (oe.objectIsBigEnough(i)) {
    			cv::Mat scrop = oe.getSingleCroppedImg(i);
    			finalCropped = cv_bridge::CvImage(img_msg->header, img_msg->encoding, scrop);
				finalCropped.toImageMsg(finalImage);
    		}
			// assign all cropped entities img to the msg to be published.
			dea_.entity.push_back(finalImage);
    	}
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	det_ent_pub_.publish(dea_);
  
//*  *********** THIS IS FOR DISPLAYING EACH CROPPED ENTITIES *****
	if (display_first_object_) {
		cv::Mat croppedRef;
		//if (blob_msg.count > 0) {
			ROS_INFO_STREAM(blob_msg.minx[0] << "\t" << 
		                  blob_msg.miny[0] << "\t" <<
		                  blob_msg.maxx[0] << "\t" <<
		                  blob_msg.maxy[0]);
	  
		    ROS_INFO_STREAM(blob_msg.count << "BLOBS FOUND !!");
		    cv::Rect myROI(blob_msg.minx[0],
				blob_msg.miny[0],
				blob_msg.maxx[0]-blob_msg.minx[0],
				blob_msg.maxy[0]-blob_msg.miny[0]);
		    croppedRef = cv::Mat(cv_img->image,myROI);
		    //croppedRef.copyTo(cropped11);
		    // Show the cropped image    
		//}
	    cv::imshow("test", croppedRef);
	    cv::waitKey(30);
	    //*/
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "preproc_node");
	ros::NodeHandle nhp("~");
	bool view_object;
	nhp.getParam("displayFirstObject", view_object);
	AttnWindow lpp(view_object);
	ros::spin();
	return 0;
}
