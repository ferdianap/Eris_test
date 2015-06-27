#ifndef VATTN_HPP_
#define VATTN_HPP_

#include <visor/Vattn/saliency_map.hpp>
#include <visor/Vattn/extract_data.hpp>
// Need to include opencv here
#include <opencv2/highgui/highgui.hpp>

class Vattn
{
	typedef std::vector<std::vector<cv::Point> > Contours;
	typedef std::vector<cv::Vec4i> Hierarchy;

	cv::RNG rng;
	cv::Scalar randomColor;
	bool firstIteration;

	int max_blob_cnt_;
	float entropy_ratio_;
	float weight_intensity_, weight_color_, weight_form_;
	int channels_, depth_bit_, mask_min_, mask_max_;

	// cv::Mat curr_bgr_; // We don't need this bc it's passed for mainmethod input
	cv::Mat curr_gray_, prev_gray_, diff_gray_;

	CSaliencyMap  saliency_map_;
	cv::Mat saliency_image_, blank_, mod_input_, still_mod_input_;

	CExtractData    		stExtractData;
	std::vector<cv::Rect>  	stSaliencyRect;
	std::vector<cv::Point> 	stSaliencyPoint;
	/************************************************************
	 * THIS IS AN ADDITIONAL ALGORITHM 
	 * TO MERGE OVERLAPPER SALIENCY BLOBS INTO SEPARATE ENTITIES
	 *
	 ************************************************************/
	Contours contours;
	Hierarchy hierarchy;

	std::vector<cv::Rect> boundRectBlobs;
	std::vector<cv::Rect> finalDetectedObj;
public:
	Vattn();
	~Vattn();

	void mainmethod(const cv::Mat bgr_img);
	cv::Mat getSalmapImg() { return saliency_image_; }
	cv::Mat getStillImg() { return still_mod_input_; }
	cv::Mat getImg() { return mod_input_; }
	void printExternalContours(
		cv::Mat img, 
		Contours const& contours, 
		Hierarchy const& hierarchy, 
		int const idx
	);
	void drawRect_one(
		cv::Mat img,
		cv::Rect rect,
		cv::Scalar color = cv::Scalar(255),
		int thickness = 1
		);
	void drawRect_all(
		cv::Mat img,
		std::vector<cv::Rect> rect_vect,
		cv::Scalar color = cv::Scalar(255),
		int thickness = 1
		);
	inline bool isObjectTooSmall(
		cv::Rect rect,
		int thres = 15 // default is 15
	){
		return rect.width <= thres || rect.height <= thres ? true : false;
	}
	inline bool sceneNoChangesDetected() {
		return contours.size()==0 ? true : false;
	}
	std::vector<cv::Rect> getFinalDetectedObj() { return finalDetectedObj; }
	// Setter
	void setMaxBlobCount(int n) { max_blob_cnt_ = n; }
	void setEntropyRatio(float n) { entropy_ratio_ = n; }
	void setWeightIntensity(float n) { weight_intensity_ = n; }
	void setWeightColor(float n) { weight_color_ = n; }
	void setWeightForm(float n) { weight_form_ = n; }
	void setChannels(int n) { channels_ = n; }
	void setDepthBit(int n) { depth_bit_ = n; }
	void setMaskMin(int n) { mask_min_ = n; }
	void setMaskMax(int n) { mask_max_ = n; }
	// Setter Image Member
	// void setCurrBGR(const cv::Mat &img) { curr_bgr_ = img; } // No need
	void setCurrGray(const cv::Mat &img) { curr_gray_ = img; }
	void setPrevGray(const cv::Mat &img) { prev_gray_ = img; }
	// Getter Image Member
	// cv::Mat getCurrBGR() { return curr_bgr_; } // No need as its var
	cv::Mat getCurrGray() { return curr_gray_; }
	cv::Mat getPrevGray() { return prev_gray_; }
	cv::Mat getDiffGray() { return diff_gray_; }
};

#endif