#ifndef HRICASE3_HPP_
#define HRICASE3_HPP_
#include <vector>
#include <string>

class Case3 {
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
	struct Condition {
		// bool atleast;
		std::string leftmost;
		std::string rightmost;
		std::string no_;
		// int nOther;
		// int nObjs;
		std::vector<std::string> objs;

	};
	Condition cond_;
	int blobCnt_;
	std::vector<std::vector<int> > colorFeats_, textureFeats_;
	std::vector<std::string> context_;
	std::vector<object_result> objsFam_;
	const float colorThres_, textureThres_;
	static std::vector<std::string> keywordsTemplate_;
public:
	Case3();
	~Case3();
	bool init(std::vector<std::string> &ctx);
	inline void setColorFeats(std::vector<std::vector<int> > cF) { colorFeats_ = cF; }
	inline void setTextureFeats(std::vector<std::vector<int> > tF) { textureFeats_ = tF; }
	inline void setObjCount(int c) { blobCnt_ = c; }

	std::string answerQuestion(int q);
	
	std::string processContextInputObject();
	std::string processContextInputScene();
};

#endif