#ifndef OBJEXTRACTOR_HPP_
#define OBJEXTRACTOR_HPP_

#include <opencv2/imgproc/imgproc.hpp>

/**
 * Example how to use this class:
 * ObjectExtractor oe(blob_msg.count, blob_msg.minx, blob_msg.miny, blob_msg.maxx, blob_msg.maxy);
 * cv::Mat srcimg = cv_img->image;
 * oe.setSourceImg(srcimg); // should be cv::Mat
 * oe.computeROIs();
 * oe.processCroppedImgs();
 * // do anything else
 */

class ObjectExtractor
{
	int objcnt_;
	cv::Mat srcimg_;
	std::vector<int> minx_, miny_, maxx_, maxy_;
	std::vector<cv::Rect> ROIs_;
	std::vector<cv::Mat> croppeds_;
public:
	ObjectExtractor();
	// 	int cnt,
	// 	std::vector<int> &minx,
	// 	std::vector<int> &miny,
	// 	std::vector<int> &maxx,
	// 	std::vector<int> &maxy
	// );
	~ObjectExtractor();
	void computeROIs(); // for loop computation
	void processCroppedImgs();
	// Getter Members
	inline int getObjCount() { return objcnt_; }
	inline std::vector<cv::Rect> getROIs() { return ROIs_; }
	inline cv::Mat getSingleCroppedImg(int &idx) { return croppeds_[idx]; }
	inline std::vector<cv::Mat> getCroppedImgs() { return croppeds_; }
	// Setter Members
	void setBlobCount(int cnt) { objcnt_ = cnt; }
	void setSourceImg(const cv::Mat &img) { srcimg_ = img; }
	bool objectIsBigEnough(int &idx);
	void setBlobsManually(int minx, int miny, int maxx, int maxy);
};


#endif