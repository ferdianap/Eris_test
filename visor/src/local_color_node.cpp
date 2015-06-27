/**
 * This class subscribes to:
 *    /detected_ent_img
 * and publishes an array of color info of detected entities
 *    /local_feat/color
 *
 * The published msg is the same using colorInfo.msg
 *    mean[0] for entity 1, mean[1] entity 2, etc
 *    var[]  
 * Therefore, the Array size for each mean and var is:
 *    no of detected entities
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <visor/local_color_node.hpp>

#include <visor/MPEG7FexLib_src_OpenCV22/Feature.h>


LocalColorFeat::LocalColorFeat() {
	sub_ = nh_.subscribe("/detected_ent_img", 1,
	&LocalColorFeat::localColorCb, this);
	pub_ = nh_.advertise<visor::localColor>("/local_feat/color",1);
	cv::namedWindow( "test", CV_WINDOW_AUTOSIZE);
}

LocalColorFeat::~LocalColorFeat() {
}

void LocalColorFeat::localColorCb(
	const visor::detEntityArray::ConstPtr& msg
){
	cv_bridge::CvImagePtr cv_ptr;
	
	lcolor_.object_color.clear();
	lcolor_.header.stamp = msg->header.stamp;
	lcolor_.object_count = msg->count;
	cinfo_.header.stamp = msg->header.stamp;
	cinfo_.size = 64; //msg->count;		
	for (int i=0; i<msg->count; i++) {
		try {
			cv_ptr = cv_bridge::toCvCopy(msg->entity[i], sensor_msgs::image_encodings::BGR8);
			// ROS_WARN("OBJECT %d: ", i);
			Frame* frame = new Frame( cv_ptr->image );
			XM::ColorStructureDescriptor* csd = Feature::getColorStructureD(frame, cinfo_.size);
			cinfo_.desc.clear();
			for(int i = 0; i < csd->GetSize(); i++) {
				// std::cout << (int)csd->GetElement(i) << " ";
				// Assign each element to cinfo_.desc
				cinfo_.desc.push_back((int)csd->GetElement(i));
			}
			// std::cout << std::endl;
			delete csd;
			lcolor_.object_color.push_back(cinfo_);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("LOCAL COLOR <- cv_bridge exception: %s", e.what());
			return;
		}
	}
	pub_.publish(lcolor_);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "local_color_feat");
	LocalColorFeat lcf;
	ros::spin();
	return 0;
}