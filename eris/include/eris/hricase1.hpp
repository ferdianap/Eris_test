#ifndef HRICASE1_HPP_
#define HRICASE1_HPP_
#include <vector>
#include <string>

#include <visor/Vattn/vattn.hpp>
#include <eris/tag.h>
#include <eris/console_pretty.hpp>
/**
 * Case 1 - Image feed only
 */
class Case1 {
protected:
	struct object_result {
		bool is_familiar;
		std::vector<std::string> label;
		std::vector<int> color_similarity; // dist (the lower the better)
		std::vector<int> texture_similarity;
	};
	struct scene_result {
		bool is_familiar;
		std::vector<std::string> label;
		std::vector<int> recalled_obj_cnt; // representing the obj_cnt from a recalled scene
		std::vector<int> familiar_obj_cnt; // representing the familiar obj_cnt of currently detected scene
	};
	struct object_pos_2dfov {
		int minx;
		int maxx;
		int miny;
		int maxy;
	};
	Vattn va;
	scene_result scnres_;
	const float colorThres_, textureThres_;
	int blobCnt_;
	std::vector<cv::Rect> bblobs_;
	std::vector<std::vector<int> > colorFeats_, textureFeats_;
	// This contains the list of familiar objects based on SM data
	std::vector<object_result> objsFam_;

	std::vector<cv::Mat> objImgs_;
	eris::tag inboxTag_;
	std::vector<object_pos_2dfov> objs2dPos_;
public:
	Case1();
	~Case1();
	virtual void init();
	virtual void processFeaturesFromInputScene();
	virtual void loadInputs(
		int &cnt,
		std::vector<int> &minx,
		std::vector<int> &maxx,
		std::vector<int> &miny,
		std::vector<int> &maxy,
		std::vector<cv::Mat> &imgs,
		std::vector<std::vector<int> > cfeats,
		std::vector<std::vector<int> > tfeats
	);
	virtual void computeInputFamiliarity();
	virtual void determineSceneFamiliarityFromObjs();
	virtual void computeSceneFamiliarity(
		std::string &objname,
		scene_result &scn,
		object_pos_2dfov &pos
	);
	bool determineSceneStatus(scene_result &scn, int &blobCnt);
	std::string computeTopLabel(object_result o);
	object_pos_2dfov assignPosValue(
		const short int &minx,
		const short int &maxx, 
		const short int &miny,
		const short int &maxy
	);
	bool isPositionSimilar(
		object_pos_2dfov &pos_recalled,
		object_pos_2dfov &pos_detected
	);
	void encode_em_episode(
		const std::string& label,
		std::vector<std::string>& objectLabels,
		int objectCount,
		ros::Time sceneTime,
		std::vector<object_pos_2dfov> &pos
	);
	void encode_sm_entity(
		const std::string& fname,
		const std::string& label,
		ros::Time encTime,
		cv::Mat &img,
		std::vector<int> &ot,
		std::vector<int> &oc
	);
	// Setter Members for load inputs
	inline void setObjCount(int &c) { blobCnt_ = c; }
	// Getter Members
	inline std::vector<std::string> getFamiliarSceneLabels() { return scnres_.label; }
	inline object_result getFamiliarObjectLabel(int &idx) { return objsFam_[idx]; }
	inline int getNumOfFamiliarObjs() { return objsFam_.size(); }
	inline int getNumOfFamiliarScns() { return scnres_.label.size(); }
	std::string answerQuestion(int q);
	inline int getObjCount() { return blobCnt_; }

	inline std::vector<std::vector<int> > getColorFeats() { return colorFeats_; }
	inline std::vector<std::vector<int> > getTextureFeats() { return textureFeats_; }
	// TODO: add getTopObjRes, similar to computeTopLabel but return object_result with only 1 result

};
#endif