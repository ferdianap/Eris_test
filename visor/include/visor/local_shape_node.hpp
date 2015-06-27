#ifndef LOCAL_SHAPE_NODE_HPP_
#define LOCAL_SHAPE_NODE_HPP_

#include <sensor_msgs/image_encodings.h>
#include <visor/detEntityArray.h>
#include <visor/localTexture.h>
#include <visor/textureInfo.h>

class LocalShapeDescriptor {
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Publisher ls_pub_;
	visor::textureInfo tinfo_; // Message declared
	visor::localTexture lt_;
public:
	LocalShapeDescriptor();
	~LocalShapeDescriptor();

	void localShapeCb(const visor::detEntityArray::ConstPtr& msg);
};

#endif