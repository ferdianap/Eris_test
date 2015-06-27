/**
 * ENCODER NODE
 * This node currently encodes both SM and EM 
 */ 

#include <ros/ros.h>
#include <eris/encoder_node.hpp>
#include <eris/file_operations.hpp>
#include <eris/console_pretty.hpp>
// #include <typeinfo>
#include <eris/hricase1.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
EncoderNode::EncoderNode(bool sampling, int colorThres, int textureThres)
	: localShape_sub_(nh_, "/local_feat/shape",1),
	localColor_sub_(nh_, "/local_feat/color",1),
	sync( MySyncPolicy( 100 ), localShape_sub_, localColor_sub_ )
{	
	// sampleObject_ = sampling;
	colorThres_ = colorThres;
	textureThres_ = textureThres;
	sync.registerCallback( boost::bind(&EncoderNode::EncoderCb, this, _1, _2) );
}

EncoderNode::~EncoderNode() {
}

void EncoderNode::EncoderCb(
	const visor::localTexture::ConstPtr& 	s_msg,
	const visor::localColor::ConstPtr&  c_msg
){
	// Format conversion ----------------
	int objcnt = (int)s_msg->count;
	std::vector<std::vector<int> > colorFeats, textureFeats;
	std::vector<cv::Mat> objsimgs;
	// assign colorFeats and textureFeats
	int cDescSize = 64;//c_msg->object_color[0].size;
	int tDescSize = 80;//s_msg->object_texture[0].size;
	assert(cDescSize < tDescSize); // 64 < 80
	std::vector<int> minx, miny, maxx, maxy;
	for (int i = 0; i < objcnt; ++i) {// for every object
		std::vector<int> cfeatTMP, tfeatTMP;
		for(int j = 0; j < tDescSize; j++) {
			if (j < cDescSize) {
				cfeatTMP.push_back(c_msg->object_color[i].desc[j]);
			}
			tfeatTMP.push_back(s_msg->object_texture[i].desc[j]);
		}
		colorFeats.push_back(cfeatTMP);
		textureFeats.push_back(tfeatTMP);
		// convert img to cv::Mat
		cv_bridge::CvImagePtr cv_img = cv_bridge::toCvCopy(s_msg->image[i], sensor_msgs::image_encodings::BGR8);
		cv::Mat img = cv_img->image;
		// assign objimgs
		objsimgs.push_back(img);
		// assign vector of 2D pos
		minx.push_back(s_msg->pos_minx[i]);
		maxx.push_back(s_msg->pos_maxx[i]);
		miny.push_back(s_msg->pos_miny[i]);
		maxy.push_back(s_msg->pos_maxy[i]);
	}
	// Actual Module init and computation ---------------
	Case1 c1;
	c1.init();
	c1.loadInputs(
		objcnt,// load blobCnt_ (no need to load bblobs_)
		minx, 
		maxx,
		miny,
		maxy,// load objs2dPos_.push_back
		objsimgs,// load objImgs_
		colorFeats,// load colorFeats_
		textureFeats// load textureFeats_
	);
	c1.computeInputFamiliarity();
	c1.determineSceneFamiliarityFromObjs();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "SMEncoder_node");
	ros::NodeHandle nhp("~");
    bool sampling;
    int colorThres, textureThres;
    nhp.param(std::string("sampleSingleObject"), sampling, false);
    nhp.param(std::string("colorThres"), colorThres, 65);
    nhp.param(std::string("textureThres"), textureThres, 65);
	EncoderNode en(sampling, colorThres, textureThres);
	ros::spin();
	return 0;
}
