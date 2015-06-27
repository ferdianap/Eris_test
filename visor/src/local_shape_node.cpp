/**
 * This node calculates the texture representing the texture(shape) of each entity
 * within a particular detected scene.
 *
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <visor/local_shape_node.hpp>
#include <visor/MPEG7FexLib_src_OpenCV22/Feature.h>

LocalShapeDescriptor::LocalShapeDescriptor() {
	sub_ = nh_.subscribe("/detected_ent_img", 1,
		&LocalShapeDescriptor::localShapeCb, this);
	ls_pub_ = nh_.advertise<visor::localTexture>("/local_feat/shape",1);
}

LocalShapeDescriptor::~LocalShapeDescriptor() {
}

void LocalShapeDescriptor::localShapeCb(
	const visor::detEntityArray::ConstPtr& msg
){
	cv_bridge::CvImagePtr cv_ptr;

	lt_.image.clear();
	lt_.pos_minx.clear();
	lt_.pos_miny.clear();
	lt_.pos_maxx.clear();
	lt_.pos_maxy.clear();
	
	lt_.header.stamp = msg->header.stamp;
	lt_.count = msg->count;
	lt_.object_texture.clear();	
	lt_.pos_minx = msg->pos_minx;
	lt_.pos_miny = msg->pos_miny;
	lt_.pos_maxx = msg->pos_maxx;
	lt_.pos_maxy = msg->pos_maxy;
	lt_.image = msg->entity;
	
	tinfo_.header.stamp = lt_.header.stamp;
	tinfo_.size = 80; // 80 default for CSD
	for (int i=0; i<msg->count; i++) { // For all detected objects,
		try {
			cv_ptr = cv_bridge::toCvCopy(msg->entity[i],  sensor_msgs::image_encodings::BGR8);
			Frame* frame = new Frame( cv_ptr->image );
			XM::EdgeHistogramDescriptor* ehd = Feature::getEdgeHistogramD(frame);
			tinfo_.desc.clear();//gr_.gist.clear();
			char* de = ehd->GetEdgeHistogramElement();
			for(int i=0; i < ehd->GetSize(); i++) {
				tinfo_.desc.push_back((int)de[i]);
			}
			delete ehd;
			lt_.object_texture.push_back(tinfo_);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("LOCAL SHAPE <- cv_bridge exception: %s", e.what());
			return;
		}
	}
	ls_pub_.publish(lt_);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "local_shape_descriptor");
	LocalShapeDescriptor lsd;
	ros::spin();
	return 0;
}