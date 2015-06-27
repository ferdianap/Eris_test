#include <visor/Vattn/vattn.hpp>

Vattn::Vattn()
	: firstIteration(true)
{
	rng = 12345;
    randomColor = cv::Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
}

Vattn::~Vattn(){}

void Vattn::mainmethod(const cv::Mat bgr_img) {
	if (firstIteration) {
		prev_gray_ = curr_gray_;
		diff_gray_ = curr_gray_;
		firstIteration = false;
	}
	cv::absdiff(prev_gray_, curr_gray_, diff_gray_);
    cv::blur(diff_gray_, diff_gray_, cv::Size(3,3));
    cv::threshold( diff_gray_, diff_gray_, 40, 255, CV_THRESH_BINARY);
    cv::findContours( diff_gray_, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE), cv::Point(0,0);
    // Draw contours
	for (int i=0; i<(signed int)contours.size(); i++) {
		cv::drawContours(diff_gray_, contours, i, randomColor, 2, 8, hierarchy, 0, cv::Point());
	}

	mod_input_ = bgr_img.clone();
	saliency_map_.setData(bgr_img); // this is the saliency
	//TODO: make an assertion that setData is executed before Execute method !
	saliency_map_.Execute(weight_intensity_, weight_color_, weight_form_);
	saliency_image_ = saliency_map_.SaliencyMap();
	// always clear to prevent remaining prev data!
	finalDetectedObj.clear();
	boundRectBlobs.clear();
	stExtractData.SetData(saliency_image_);
	stExtractData.Execute(max_blob_cnt_, mask_min_, mask_max_, entropy_ratio_);
	blank_ = cv::Mat::zeros(bgr_img.rows,bgr_img.cols,CV_8UC1);
	if (stExtractData.Count() > 0) {
		for (int i=0; i<stExtractData.Count(); i++) {
			stSaliencyPoint.push_back(cv::Point(0,0));
			stSaliencyRect.push_back(cv::Rect(0,0,0,0));
			stSaliencyPoint[i] = stExtractData.SaliencyPoint(i);
			stSaliencyRect[i]  = stExtractData.SaliencyRect(i);
			// Draw rect in blank for contour detection
			drawRect_one(blank_, stSaliencyRect[i]);
			// UNCOMMENT BELOW TO PRINT THE SALIENT AREAS
			drawRect_one(mod_input_, stSaliencyRect[i], cv::Scalar(0,255,0,0));
		}
		cv::findContours(blank_, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
		printExternalContours(mod_input_, contours, hierarchy, 0);
		for (int i=0; i<(signed int)boundRectBlobs.size(); ++i) {
			//printf("width: %d, Height: %d\n",boundRectBlobs[i].width,boundRectBlobs[i].height);
			if (!isObjectTooSmall(boundRectBlobs[i])) {
				finalDetectedObj.push_back(boundRectBlobs[i]);
			}
		}
		//printf("now final vect has %d element\n",finalDetectedObj.size());
		// Print the Final Detected Object Region
		drawRect_all(mod_input_, finalDetectedObj, cv::Scalar(0,255,255,0), 2);
		still_mod_input_ = mod_input_.clone();
	}
	prev_gray_ = curr_gray_;
}

void Vattn::drawRect_one(
	cv::Mat img,
	cv::Rect rect,
	cv::Scalar color,
	int thickness
){
	cv::Point p1, p2;
	p1.x = rect.x;
	p1.y = rect.y;
	p2.x = p1.x + rect.width;
	p2.y = p1.y + rect.height;
	cv::rectangle(img, p1, p2, color, thickness);
}

void Vattn::drawRect_all(
	cv::Mat img,
	std::vector<cv::Rect> rect_vect,
	cv::Scalar color,
	int thickness
){
	for (int i=0; i<(signed int)rect_vect.size(); i++) {
		drawRect_one(img, rect_vect[i], color, thickness);
	}
}

void Vattn::printExternalContours(
	cv::Mat img, 
	Contours const& contours, 
	Hierarchy const& hierarchy, 
	int const idx
){
    // For every contour of the same hierarchy level
    for (int i = idx; i >= 0; i = hierarchy[i][0]) {
        // Print it using either drawContour or rectangle (NOT USED)
        //cv::drawContours(img, contours, i, cv::Scalar(255));
        //cv::rectangle(img, cv::boundingRect(contours[i]), cv::Scalar(255), 2);
        boundRectBlobs.push_back(cv::boundingRect(contours[i]));

        // For every of its internal contours
        for (int j = hierarchy[i][2]; j >= 0; j = hierarchy[j][0]) {
            //recursively print the external contours of its children
            printExternalContours(img, contours, hierarchy, hierarchy[j][2]);
        }
    }
}