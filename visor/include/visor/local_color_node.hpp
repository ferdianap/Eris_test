#ifndef LOCAL_COLOR_NODE_HPP_
#define LOCAL_COLOR_NODE_HPP_

#include <visor/detEntityArray.h>
 
#include <visor/colorInfo.h>
#include <visor/localColor.h>

class LocalColorFeat {
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Publisher pub_;
	
	visor::colorInfo cinfo_; // Message declared
	visor::localColor lcolor_;

public:
	LocalColorFeat();
	~LocalColorFeat();

	void localColorCb(const visor::detEntityArray::ConstPtr& msg);
};


#endif