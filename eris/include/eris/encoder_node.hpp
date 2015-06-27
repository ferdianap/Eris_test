#ifndef ENCODER_NODE_HPP_
#define ENCODER_NODE_HPP_

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
// Custom msgs
#include <visor/localColor.h>
#include <visor/localTexture.h>

class EncoderNode {
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	
	message_filters::Subscriber<visor::localTexture> localShape_sub_;
	message_filters::Subscriber<visor::localColor> 	localColor_sub_;

	typedef message_filters::sync_policies::ApproximateTime<
		visor::localTexture,
		visor::localColor
	> MySyncPolicy;

	message_filters::Synchronizer< MySyncPolicy > sync;

	//bool sampleObject_; // NOT USED AT THE MOMENT
	int colorThres_, textureThres_;

public:
	EncoderNode(bool sampling, int colorThres, int textureThres);
	~EncoderNode();

	void EncoderCb(
		const visor::localTexture::ConstPtr& 		s_msg,
		const visor::localColor::ConstPtr& 	c_msg
	);
};
#endif