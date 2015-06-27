#include <visor/object_extractor.hpp>

ObjectExtractor::ObjectExtractor()//int cnt): objcnt_(cnt)
// int cnt,
// 	std::vector<int> minx, std::vector<int> miny,
// 	std::vector<int> maxx, std::vector<int> maxy
// ): objcnt_(cnt), minx_(minx), miny_(miny), maxx_(maxx), maxy_(maxy)
{
}

ObjectExtractor::~ObjectExtractor() {}

void ObjectExtractor::computeROIs() {
	for (int i = 0; i < objcnt_; ++i) {
		int cwidth  = maxx_[i]-minx_[i];
		int cheight = maxy_[i]-miny_[i];
		cv::Rect myROI(minx_[i], miny_[i], cwidth, cheight);
		ROIs_.push_back(myROI);
	}
}

void ObjectExtractor::processCroppedImgs() {
	for (int i = 0; i < objcnt_; ++i) {
		if(objectIsBigEnough(i)) {
			cv::Mat croppedRef(srcimg_,ROIs_[i]);
			croppeds_.push_back(croppedRef);
		}
	}
}

bool ObjectExtractor::objectIsBigEnough(int &idx) {
	assert(idx<objcnt_);
	bool res = false;
	int cwidth  = maxx_[idx]-minx_[idx];
	int cheight = maxy_[idx]-miny_[idx];
	if (cwidth > 8 || cheight > 8) res = true;
	return res;
}

void ObjectExtractor::setBlobsManually(
	int minx, int miny, int maxx, int maxy
){
	minx_.push_back(minx);
	miny_.push_back(miny);
	maxx_.push_back(maxx);
	maxy_.push_back(maxy);
}

// TODO: make a setter for direct vector<int> of minx etc