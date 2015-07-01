#include <ros/ros.h>
#include <eris/hricase1.hpp>
#include <iostream>
#include <visor/object_extractor.hpp>
#include <visor/MPEG7FexLib_src_OpenCV22/Feature.h>
#include <eris/encode_tmp_ffi.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eris/file_operations.hpp>
#include <cv_bridge/cv_bridge.h>

Case1::Case1()
	: colorThres_(70)
	, textureThres_(70)
{
	// 1. init vars for va members
	va.setMaxBlobCount(30);
	va.setEntropyRatio(0.66);
	va.setWeightIntensity(.0005);
	va.setWeightColor(.3);
	va.setWeightForm(.01);
	va.setChannels(3);
	va.setDepthBit(8);
	va.setMaskMin(3);
	va.setMaskMax(100);
	srand (time(NULL)); // Init random seed
}

Case1::~Case1() {
}

void Case1::init() {
	if (fileExist(MEMORY_DIR+INBOXFNAME)) {
		inboxTag_ = read_ffi_tag(INBOXFNAME);
	} else {
		inboxTag_.name = "INBOX";
	}
	scnres_.is_familiar = false;
	colorFeats_.clear();
	textureFeats_.clear();
	objs2dPos_.clear();
	objsFam_.clear();
}

Case1::object_pos_2dfov Case1::assignPosValue(
	const short int &minx,
	const short int &maxx, 
	const short int &miny,
	const short int &maxy
){
	object_pos_2dfov res;
	res.minx = (int)minx;
	res.maxx = (int)maxx;
	res.miny = (int)miny;
	res.maxy = (int)maxy;
	return res;
}

std::string Case1::answerQuestion(int q) {
	std::string ans("");
	switch (q) {
	case 2: {
		break;
		}
	default: {
		// Get the list of familiar objects
		ans += "\n  Object(s) is(are) recognized as: ";
		for (int i = 0; i < getNumOfFamiliarObjs(); ++i) { // for each object
			ans += "\n* "+computeTopLabel(getFamiliarObjectLabel(i));
		}
		// and the list of familiar scene label
		std::vector<std::string> fslabel = getFamiliarSceneLabels();
		ans += "\n  Scene is recognized as: ";
		for (int i = 0; i < fslabel.size(); ++i) {
			ans += "\n* "+fslabel[i];
		}
	}
	}
	return ans;
}

/**
 * Intended to be called from sm_encoder_node,
 * where it only loads the vector<vector<int> >
 * of color and texture features to be computed using
 * getInputFamiliarity()
 */
void Case1::loadInputs(
	int &cnt,
	std::vector<int> &minx,
	std::vector<int> &maxx,
	std::vector<int> &miny,
	std::vector<int> &maxy,
	std::vector<cv::Mat> &imgs,
	std::vector<std::vector<int> > cfeats,
	std::vector<std::vector<int> > tfeats
){
	blobCnt_ = cnt;
	for (int i = 0; i < blobCnt_; ++i) {
		objs2dPos_.push_back(assignPosValue(minx[i], maxx[i], miny[i], maxy[i]));
	}
	objImgs_ = imgs;
	colorFeats_ = cfeats;
	textureFeats_ = tfeats;
}

/**
 * Process objects features by the image processing class 
 * used by vattn load img and process saliency detection
 */
void Case1::processFeaturesFromInputScene() {
	const std::string fn("vision.JPG");
	const std::string RBTVIS_DIR("/home/ros/scene_snap/BaxterVision/");		
	cv::Mat imgBGR = cv::imread(RBTVIS_DIR+fn, CV_LOAD_IMAGE_COLOR);
	if (imgBGR.rows!=100 || imgBGR.cols!=160) {
		cv::resize(imgBGR,imgBGR,cv::Size(160,100));
	}
	cv::Mat imgGray;
	cv::cvtColor(imgBGR, imgGray, CV_BGR2GRAY);
	va.setCurrGray(imgGray);
	va.mainmethod(imgBGR);
	bblobs_.clear();
    bblobs_ = va.getFinalDetectedObj();
    blobCnt_ = bblobs_.size(); // bblobs_ is a vector<cv::Rect>
    // Then we use the method of attn_window_node to crop the
    // image based on bblobs_,
    printf("blobcnt = %d\n",blobCnt_);
    ObjectExtractor oe;
	oe.setBlobCount(blobCnt_);
	for (int i = 0; i < blobCnt_; ++i) {
		int maxxTMP = bblobs_[i].x + bblobs_[i].width;
		int maxyTMP = bblobs_[i].y + bblobs_[i].height;
		printf("%d, %d, %d, %d\n",bblobs_[i].x, bblobs_[i].y, maxxTMP, maxyTMP);
		oe.setBlobsManually(bblobs_[i].x, bblobs_[i].y, maxxTMP, maxyTMP);
		// also set objs2dPos_
		objs2dPos_.push_back(assignPosValue(bblobs_[i].x, maxxTMP, bblobs_[i].y, maxyTMP));
	}
	oe.setSourceImg(imgBGR); // should be cv::Mat
	oe.computeROIs();
	oe.processCroppedImgs();
	objImgs_ = oe.getCroppedImgs();
	const int cinfoSize = 64;
	for (int i=0; i<oe.getObjCount(); ++i) {
		if (oe.objectIsBigEnough(i)) {
			cv::Mat scrop = oe.getSingleCroppedImg(i);
			// and proceed to check color
			std::vector<int> cfeatTMP, tfeatTMP;
			Frame* frame = new Frame( scrop );
			XM::ColorStructureDescriptor* csd = Feature::getColorStructureD(frame, cinfoSize);
			for(int i = 0; i < csd->GetSize(); i++) {
				cfeatTMP.push_back((int)csd->GetElement(i));
			}
			delete csd;
			colorFeats_.push_back(cfeatTMP);
			// and texture
			XM::EdgeHistogramDescriptor* ehd = Feature::getEdgeHistogramD(frame);
			char* de = ehd->GetEdgeHistogramElement();
			for(int i=0; i < ehd->GetSize(); i++) {
				tfeatTMP.push_back((int)de[i]);
			}
			delete ehd;
			textureFeats_.push_back(tfeatTMP);
		}
	}
}

void Case1::encode_sm_entity(
	const std::string& fname,
	const std::string& label,
	ros::Time encTime,
	cv::Mat &img,
	std::vector<int> &ot,
	std::vector<int> &oc
){
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	eris::smEntity ent;
	ent.header.stamp = encTime;
	ent.label        = label;
	ent.image        = *msg;//sMsg->image[idx];
	for (int i=0;i<ot.size();++i) {
		ent.texture.push_back((short int)ot[i]);
	}
	for (int i = 0; i < oc.size(); ++i) {
		ent.color.push_back((short int)oc[i]);
	}
	// ROS_WARN_STREAM("Consolidating corresponding SM data...");
	// ent.texture      = ot;//sMsg->object_texture[idx].desc;
	// ent.color        = oc;//cMsg->object_color[idx].desc;
	write_sm_entity(fname, ent);
}

void Case1::encode_em_episode(
	const std::string& label,
	std::vector<std::string>& objectLabels,
	int objectCount,
	ros::Time sceneTime,
	std::vector<object_pos_2dfov> &pos
){
	eris::episode epi;
	const std::string fname = label;
	epi.header.stamp 		= sceneTime;
	epi.label 				= label;
	epi.obj_count 			= objectCount;
	epi.seq_count	 		= 0;// for now 0 TODO: fix this
	for (int i = 0; i < pos.size(); ++i) {
		epi.pos_minx.push_back(pos[i].minx);
		epi.pos_miny.push_back(pos[i].miny);
		epi.pos_maxx.push_back(pos[i].maxx);
		epi.pos_maxy.push_back(pos[i].maxy);
	}
	epi.obj_name 			= objectLabels;
	//epi.sequence.push_back(std::string(category));
	write_em(fname, epi);
}


/**
 * Check obj familiarity
 */
void Case1::computeInputFamiliarity() {
	for (int i = 0; i < blobCnt_; ++i) { 
		object_result objRes;
		objRes.is_familiar = false;
		const std::string fname(".sm");
		std::vector<std::string> smlist = getFileList(MEMORY_DIR, fname);
		std::vector<std::string>::iterator iter;
		const std::vector<int> color_sensed = colorFeats_[i];
		const std::vector<int> texture_sensed = textureFeats_[i];
		for (iter = smlist.begin(); iter != smlist.end(); ++iter) {
			bool is_color_familiar = false;
			bool is_texture_familiar = false;
			eris::smEntity smdata = read_sm_entity(*iter);
			// convert vec from short int to int
			std::vector<int> color_recalled(smdata.color.begin(), smdata.color.end());
			std::vector<int> texture_recalled(smdata.texture.begin(), smdata.texture.end());
			float color_similarity = color_check(color_sensed, color_recalled);
			if (color_similarity > colorThres_) is_color_familiar = true;
			float texture_similarity = texture_check(texture_sensed, texture_recalled);
			if (texture_similarity > textureThres_) is_texture_familiar = true;
			if (is_color_familiar && is_texture_familiar) {
				objRes.is_familiar = true;
				objRes.label.push_back(smdata.label);
				objRes.color_similarity.push_back(color_similarity);
				objRes.texture_similarity.push_back(texture_similarity);
			}
		}
		objsFam_.push_back(objRes);
	}
}

void Case1::determineSceneFamiliarityFromObjs() {
	std::vector<std::string> objectLabels; // contains labels to be encoded in the Episode
	for (int i = 0; i < blobCnt_; ++i) {
		std::string fname("");
		if (objsFam_[i].is_familiar) {
			fname = computeTopLabel(objsFam_[i]);
			printGreen("scene_status.is_familiar = "); std::cout << scnres_.is_familiar << "\n";
			computeSceneFamiliarity(fname, scnres_, objs2dPos_[i]);
		} else { // encode SM
			fname = gen_rand_name(5);
			// ROS_WARN_STREAM("Generating file named "+fname+".sm\n");
			inboxTag_.assoc_obj.push_back(fname);
			ros::Time encTime	 = getTimeNow();
			encode_sm_entity(OBJ+fname, fname, encTime, objImgs_[i], textureFeats_[i], colorFeats_[i]);
		}
		objectLabels.push_back(fname);
		printWhite("=== END of Processing OBJ "+intToString(i)+"===\n");
	}
	scnres_.is_familiar = determineSceneStatus(scnres_, blobCnt_);
	ROS_WARN("scene_status(final) = %s\n",boolToString(scnres_.is_familiar));
	if (scnres_.is_familiar) {
		ROS_WARN("ALL OBJECTS ARE FAMILIAR!");
	} else {  // Encode EM
		write_ffi_tag(INBOXFNAME, inboxTag_);
		ros::Time sceneTime   = getTimeNow();
		int objCount          = blobCnt_;
		std::string sceneName = "SE_"+gen_rand_name(5);
		encode_em_episode(sceneName, objectLabels, blobCnt_, sceneTime, objs2dPos_);
	}
}

bool Case1::determineSceneStatus(scene_result &scn, int &blobCnt) {
	bool res = false;
	// determine whether all objects are familiar but in a diff position
	// or the detected scene already exist in em database
	for (int i = 0; i < scn.familiar_obj_cnt.size(); i++) {
		std::cout << scn.label[i] <<"\n  Curr Det Obj= " << blobCnt << " with  Det Fam Obj= " << scn.familiar_obj_cnt[i] << " TOTAL= " << scn.recalled_obj_cnt[i] << "\n";
		if (blobCnt == scn.familiar_obj_cnt[i] &&
			blobCnt == scn.recalled_obj_cnt[i]) 
			res = true;
	}
	return res;
}

void Case1::computeSceneFamiliarity(
	std::string &objname,
	scene_result &scn,
	object_pos_2dfov &pos
){
	// check em data
	const std::string em("SE_");
	std::vector<std::string> scenelist = getFileList(MEMORY_DIR, em);
	std::vector<std::string>::iterator iter;
	// for all em data
	for (iter = scenelist.begin(); iter != scenelist.end(); ++iter) {
		eris::episode epi = read_em(*iter); // read each episode
		printWhite("Processing "+*iter+"\n");
		// check if objname exist in em data
		if (stringExistsInVect(objname, epi.obj_name)) {
			printRed(objname+" exist in "+epi.label+"\n");
			// get objname pos from scene
			std::vector<std::string>::iterator it = std::find(epi.obj_name.begin(), epi.obj_name.end(), objname);
			int idx = it - epi.obj_name.begin(); // get index from vector
			object_pos_2dfov recalled 	= assignPosValue(epi.pos_minx[idx], epi.pos_maxx[idx], epi.pos_miny[idx], epi.pos_maxy[idx]);
			object_pos_2dfov detected	= assignPosValue(pos.minx, pos.maxx, pos.miny, pos.maxy);
			if (isPositionSimilar(recalled, detected)) {
				// find the vector index
				std::vector<std::string>::iterator itSCN = std::find(scn.label.begin(), scn.label.end(), *iter);
				int idxSCN = itSCN - scn.label.begin(); 
				// check if scene name already exist in scn.label
				if (stringExistsInVect(*iter, scn.label)) {
					printYellow("scene name already exist in scn.label\n");
					// printYellow("scn.is_familiar = ");
					// std::cout << boolToString(scn.is_familiar) << "\n";
					scn.familiar_obj_cnt[idxSCN] += 1;
					// std::cout << "incrementing familiar_obj_cnt_ of "<< scn.label[idxSCN] <<" into" << scn.familiar_obj_cnt[idxSCN] << "\n";
				} else { // remember scene label
					scn.label.push_back(*iter);
					scn.recalled_obj_cnt.push_back(epi.obj_count);
					scn.familiar_obj_cnt.push_back(1);
					// std::cout << "push back familiar_obj_cnt_\n";
				}
			} else {
				scn.is_familiar = false;
				printRed("familiar = false\n");
			}
		}
	}
}

bool Case1::isPositionSimilar(
	object_pos_2dfov &pos_recalled,
	object_pos_2dfov &pos_detected
){
	int thres = 20; // pixel
	// compare with pos from s_msg
	bool minx = abs(pos_recalled.minx-pos_detected.minx) < thres ? true : false;
	// printBlue(intToString(pos_recalled.minx)+" ? "+intToString(pos_detected.minx)+"\n");
	bool miny = abs(pos_recalled.miny-pos_detected.miny) < thres ? true : false;
	// printBlue(intToString(pos_recalled.miny)+" ? "+intToString(pos_detected.miny)+"\n");
	bool maxx = abs(pos_recalled.maxx-pos_detected.maxx) < thres ? true : false;
	// printBlue(intToString(pos_recalled.maxx)+" ? "+intToString(pos_detected.maxx)+"\n");
	bool maxy = abs(pos_recalled.maxy-pos_detected.maxy) < thres ? true : false;
	// printBlue(intToString(pos_recalled.maxy)+" ? "+intToString(pos_detected.maxy)+"\n");
	return minx && miny && maxx && maxy;
}

/**
 * Retrieve a single string of the most familiar object from the list
 * @param  o object_result
 * @return   a string of most familiar object
 */
std::string Case1::computeTopLabel(object_result o) {
	int i_color = std::max_element(o.color_similarity.begin(), o.color_similarity.end()) - o.color_similarity.begin();
	int i_texture = std::max_element(o.texture_similarity.begin(), o.texture_similarity.end()) - o.texture_similarity.begin();
	std::string closest_by_color = o.label[i_color];
	std::string closest_by_texture = o.label[i_texture];
	std::cout << "Closest by color  : " << closest_by_color << std::endl
		<< "Closest by texture: " << closest_by_texture << std::endl;

	return o.color_similarity[i_color] > o.texture_similarity[i_texture] ? closest_by_color : closest_by_texture;
}