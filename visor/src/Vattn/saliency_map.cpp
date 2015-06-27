#include <visor/Vattn/saliency_map.hpp>
#include <iostream>

//#define	TH_LEVEL	(80.0 / 255.0)
const double CColorFeature::TH_LEVEL = 0/255.0;// NO Filter

CColorFeature::CColorFeature() {
}

CColorFeature::~CColorFeature() {
}

void CColorFeature::Allocate(int flag) {
	red = cv::Mat(input_img.rows, input_img.cols, CV_32F, cv::Scalar(0.));
	green = red.clone();
	blue = red.clone();
	intensity = red.clone();
		/*m_pRed = cvCreateMat( m_Height, m_Width, CV_32F );
		m_pGreen = cvCreateMat( m_Height, m_Width, CV_32F );
		m_pBlue = cvCreateMat( m_Height, m_Width, CV_32F );
		m_pIntensity = cvCreateMat( m_Height, m_Width, CV_32F );*/

	if (condition1(flag)) {
		realRed = red.clone();
		realGreen = red.clone();
		realBlue = red.clone();
		realYellow = red.clone();
			/*m_pRealRed = cvCreateMat( m_Height, m_Width, CV_32F );
			m_pRealGreen = cvCreateMat( m_Height, m_Width, CV_32F );
			m_pRealBlue = cvCreateMat( m_Height, m_Width, CV_32F );
			m_pRealYellow = cvCreateMat( m_Height, m_Width, CV_32F );*/

		opponentRG = red.clone();
		opponentBY = red.clone();
			/*m_pOpponentRG = cvCreateMat( m_Height, m_Width, CV_32F );
			m_pOpponentBY = cvCreateMat( m_Height, m_Width, CV_32F );*/
	}

	if (condition2(flag)) {
		hue = red.clone();
		sat = red.clone();
		val = red.clone();
			/*m_pHue = cvCreateMat( m_Height, m_Width, CV_32F );
			m_pSaturation = cvCreateMat( m_Height, m_Width, CV_32F );
			m_pValue = cvCreateMat( m_Height, m_Width, CV_32F );*/
	}
}


void CColorFeature::setData(const cv::Mat bgr_img) {
	input_img = bgr_img.clone();
	SetColorWeight(1,1,1,1);
}

void CColorFeature::SetColorWeight(double red, double green, double blue, double yellow) {
	weightBlue = blue;
	weightRed = red;
	weightGreen = green;
	weightYellow = yellow;
}

bool CColorFeature::Execute( int flag ) {
	//assert(flag==OP_OPPONENT);
 	Allocate(flag);
 
 	int widthStep = input_img.step[0];

	int cn = input_img.channels();
 
 	assert(input_img.cols*cn==widthStep);

 	for (int row=0; row<input_img.rows; row++) {
 		for (int col=0; col<input_img.cols; col++) {
 			double b = (double)(unsigned char)input_img.data[(row*widthStep)+col*3+0]/255;
 			double g = (double)(unsigned char)input_img.data[(row*widthStep)+col*3+1]/255;
 			double r = (double)(unsigned char)input_img.data[(row*widthStep)+col*3+2]/255;
 			double i= (r+g+b)/3;

 			red.at<float>(row,col) = r;
 			green.at<float>(row,col) = g;
 			blue.at<float>(row,col) = b;
 			intensity.at<float>(row,col) = i;

 			
 			if (condition1(flag)) {
 				double rred 	= r - (g+b)/2.;
 				double rgreen 	= g - (b+r)/2.;
 				double rblue    = b - (g+r)/2.;
 				double ryellow  = (r+g)/2. -b - fabs(r-g)/2.;
 //				std::cout << rred << "      " << rgreen << std::endl;

				if (rred	< TH_LEVEL) rred = 0;
				if (rgreen	< TH_LEVEL) rgreen = 0;
				if (rblue	< TH_LEVEL) rblue = 0;
				if (ryellow	< TH_LEVEL) ryellow = 0;
				//std::cout << rred << "      " << rgreen << std::endl;
				//std::cout << weightRed << "      " << weightGreen << std::endl;

				rred	*= weightRed;   
				rgreen	*= weightGreen; 
				rblue	*= weightBlue;  
				ryellow	*= weightYellow;

 				if (rred < 0)		rred = 0.;
 				if (rgreen < 0)		rgreen = 0.;
 				if (rblue < 0)		rblue = 0.;
 				if (ryellow < 0)	ryellow = 0.;
 
 				double oppRG = fabs(rred-rgreen);
 				double oppBY = fabs(rblue-ryellow);
 //				std::cout << oppRG << "      " << oppBY << std::endl;
 				
 				realRed.at<float>(row,col) = rred;
 				realGreen.at<float>(row,col) = rgreen;
 				realBlue.at<float>(row,col) = rblue;
 				realYellow.at<float>(row,col) = ryellow;

 				opponentRG.at<float>(row,col) = oppRG;
 				opponentBY.at<float>(row,col) = oppBY;
 			}
 
 			if (condition2(flag)) {
 				double h,s,v;
 				double max = r;
 				if (max < b) max = b;
 				if (max < g) max = g;
 
 				double min = r;
 				if (min > b) min = b;
 				if (min > g) min = g;
 
 				v = max;
 
 				if (max == min) {
 					s = 0;
 					h = 0;
 				} else {
 					s = (max - min)/max;
 
 					if (max == r)			h = (g-b)*60/(max-min);
 					else if (max == g)		h = 120.0+(b-r)*60/(max-min);
 					else if (max == b)		h = 240.0+(r-g)*60/(max-min);
 
 					if (h<0)	h = h+360.0;
 
 				}
 
 				h = h/360;
 
 				hue.at<float>(row,col) = h;
 				sat.at<float>(row,col) = s;
 				val.at<float>(row,col) = v;
 			}
 		}
 	}
 	return true;
}


//============================================================

CFormFeature::CFormFeature() {
	colorFeat = new CColorFeature();		
}

CFormFeature::~CFormFeature() {
	delete colorFeat;		
}

void CFormFeature::Allocate(int flag) {
	if (condition1(flag)) {
		sobelRedX = cv::Mat(input_img.rows, input_img.cols, CV_32F, cv::Scalar(0.));
		sobelRedY = sobelRedX.clone();
		sobelGreenX = sobelRedX.clone();
		sobelGreenY = sobelRedX.clone();
		sobelBlueX = sobelRedX.clone();
		sobelBlueY = sobelRedX.clone();

		sobelRed = sobelRedX.clone();
		sobelGreen = sobelRedX.clone();
		sobelBlue = sobelRedX.clone();

		sobelWinner = sobelRedX.clone();
	}

	if (condition2(flag)) {
		sobelIntensityX = sobelRedX.clone();
		sobelIntensityY = sobelRedX.clone();
		sobelIntensity = sobelRedX.clone();
	}
}


void CFormFeature::setData(const cv::Mat bgr_img) {
	input_img = bgr_img.clone();
	colorFeat->setData(bgr_img);
}

bool CFormFeature::Execute(int flag) {
	Allocate(flag);

	colorFeat->Execute(CColorFeature::OP_RGBI);

	cv::Mat red = colorFeat->getRedChannel();
	cv::Mat green = colorFeat->getGreenChannel();
	cv::Mat blue = colorFeat->getBlueChannel();
	cv::Mat intensity = colorFeat->getIntensityChannel();


	if (condition1(flag)) {
		cv::Sobel(red, sobelRedX, -1, 1, 0); // -1 refers to src (red.depth())
		cv::Sobel(red, sobelRedY, -1, 0, 1);
		cv::Sobel(green, sobelGreenX, -1, 1, 0);
		cv::Sobel(green, sobelGreenY, -1, 0, 1);
		cv::Sobel(blue, sobelBlueX, -1, 1, 0);
		cv::Sobel(blue, sobelBlueY, -1, 0, 1);
	}

	if (condition2(flag)) {
		cv::Sobel(intensity, sobelIntensityX, -1, 1, 0);
		cv::Sobel(intensity, sobelIntensityY, -1, 0, 1);
	}

	for (int row=0; row<input_img.rows; row++) {
 		for (int col=0; col<input_img.cols; col++) {
			if (condition1(flag)) {
				double rX = fabs(sobelRedX.at<float>(row,col));
				double rY = fabs(sobelRedY.at<float>(row,col));
				double gX = fabs(sobelGreenX.at<float>(row,col));
				double gY = fabs(sobelGreenY.at<float>(row,col));
				double bX = fabs(sobelBlueX.at<float>(row,col));
				double bY = fabs(sobelBlueY.at<float>(row,col));
				

				double r = (rX+rY)/2.0;
				double g = (gX+gY)/2.0;
				double b = (bX+bY)/2.0;

				double w = r;
				if (w < g)	w = g;
				if (w < b)	w = b;

				sobelRed.at<float>(row,col) = r;
				sobelGreen.at<float>(row,col) = g;
				sobelBlue.at<float>(row,col) = b;
				sobelWinner.at<float>(row,col) = w;
			}

			if (condition2(flag)) {
				double iX = fabs(sobelIntensityX.at<float>(row,col));
				double iY = fabs(sobelIntensityY.at<float>(row,col));
				double i = (iX+iY)/2.0;
				sobelIntensity.at<float>(row,col) = i;
			}
		}
	}
	return true;
}

//===========================================================

CSaliencyMap::CSaliencyMap() {
	weightIntensity = 0.0;
	weightColor = 0.0;
	weightForm = 0.0;
	colorFeat = new CColorFeature();
	formFeat = new CFormFeature();
}

CSaliencyMap::~CSaliencyMap() {
}

void CSaliencyMap::Allocate() {
	int widthPyramid = input_img.cols;//m_Width;
	int heightPyramid = input_img.rows;//m_Height;
	cv::Mat empty(input_img.rows, input_img.cols, CV_32F);

	for (int idx=0; idx<N_PYRAMID; idx++) {
		pyramidStore.push_back(cv::Mat(heightPyramid, widthPyramid, CV_32F, cv::Scalar(0.)));
		widthPyramid = (int)ceil(widthPyramid/2.0);
		heightPyramid = (int)ceil(heightPyramid/2.0);

	}

	for (int idx=0; idx<N_ACTIVE_PYRAMID; idx++) {
		pyramidIntensity.push_back(empty);
		pyramidRGcolor.push_back(empty);
		pyramidBYcolor.push_back(empty);
		pyramidEdge.push_back(empty);
	}

	csdnStore = empty.clone();

	for (int idx=0; idx<N_CSDN; idx++) {
		csdnIntensity.push_back(empty);
		csdnRGcolor.push_back(empty);
		csdnBYcolor.push_back(empty);
		csdnEdge.push_back(empty);
	}

	intensityFeatMap = empty.clone();

	RGcolorFeatMap = empty.clone();
	BYcolorFeatMap = empty.clone();
	formFeatMap = empty.clone();
	saliencyMap = empty.clone();
}

std::vector<cv::Mat> CSaliencyMap::getPyramid(cv::Mat input) {
	std::vector<cv::Mat> pyramid;//5

	cv::Mat empty(input_img.rows, input_img.cols, CV_32F);
	//std::vector<float> pyramid;

	
	pyramidStore[0] = input.clone(); //pyramidStore[0] = input;
	//std::cout << "BEFORE\n" << std::endl;
	//std::cout << pyramidStore[0].at<float>(60,80) << " " << pyramidStore[0].at<float>(61,81) << std::endl;
	cv::blur(pyramidStore[0], pyramidStore[0], cv::Size(3,3));
	/*cv::GaussianBlur( pyramidStore[0], 
	                  pyramidStore[0], 
		    	      cv::Size(3,3),
	                  0, 
	                  0, 
	                  cv::BORDER_DEFAULT
	                  );
	*/
	for (int idxDown=1; idxDown<N_PYRAMID; idxDown++) {
		//std::cout << pyramidStore[0].size() << "   " << pyramidStore[1].size()<< std::endl;
  		cv::resize(pyramidStore[idxDown-1], //src
  				   pyramidStore[idxDown],   //dst
  				   pyramidStore[idxDown].size()
  				   );
  		cv::blur(pyramidStore[0], pyramidStore[0], cv::Size(3,3));
		/*cv::GaussianBlur(pyramidStore[idxDown], 
	                  	 pyramidStore[idxDown], 
		    	      	 cv::Size(3,3),
	                  	 0, 
	                  	 0, 
	                  	 cv::BORDER_DEFAULT
	                  	 );
 		*/
  		if (idxDown>=2) {
			for (int idxUp=idxDown; idxUp>0; idxUp--) {
				cv::resize(pyramidStore[idxUp], 
  				   		   pyramidStore[idxUp-1], 
  				   		   pyramidStore[idxUp-1].size()
  				   		   );
				cv::blur(pyramidStore[0], pyramidStore[0], cv::Size(3,3));
				/*cv::GaussianBlur(pyramidStore[idxUp-1], 
	                  	 		 pyramidStore[idxUp-1], 
		    	      	 		 cv::Size(3,3),
	                  	 		 0, 
	                  	 		 0, 
	                  	 		 cv::BORDER_DEFAULT
	                  	 		 );*/
			}
			//std::cout << "AFTER\n" << std::endl;
			//std::cout << pyramidStore[0].at<float>(60,80) << " " << pyramidStore[0].at<float>(61,81) << std::endl;
  			pyramid.push_back(pyramidStore[0].clone());
  			//pyramid.assign();//pyramidStore[0].copyTo(pyramid[2-idxDown]);//pyramid.push_back(pyramidStore[0]); //pyramid[idxDown-2] = pyramidStore[0];
  			
  		}
 	}
 	//std::cout << "[0] =" << pyramid[0].at<float>(60,80) << " " << pyramid[0].at<float>(61,81) << std::endl;
 	//std::cout << "[1] =" << pyramid[1].at<float>(60,80) << " " << pyramid[1].at<float>(61,81) << std::endl;
 	//std::cout << "[2] =" << pyramid[2].at<float>(60,80) << " " << pyramid[2].at<float>(61,81) << std::endl;
 	//std::cout << "[3] =" << pyramid[3].at<float>(60,80) << " " << pyramid[3].at<float>(61,81) << std::endl;
 	return pyramid;
}

std::vector<cv::Mat> CSaliencyMap::getCSDN(std::vector<cv::Mat> pyramid) {
	std::vector<cv::Mat> csdn;
	cv::Mat empty(input_img.rows, input_img.cols, CV_32F);

	cv::Mat csdn0 = empty.clone();
	cv::Mat csdn1 = empty.clone();
	cv::Mat csdn2 = empty.clone();
	cv::Mat csdn3 = empty.clone();

	//std::cout << pyramid[0].at<float>(60,80) << "   " << pyramid[0].at<float>(61,81) << std::endl;
	//std::cout << pyramid[2].at<float>(60,80) << "   " << pyramid[2].at<float>(61,81) << std::endl << std::endl;

	for (int row=0; row<input_img.rows; row++) {
		for (int col=0 ; col<input_img.cols; col++) {
			//assert(pyramid[0].at<float>(row,col)!=pyramid[1].at<float>(row,col));
			double l2 = pyramid[0].at<float>(row,col);
			double l3 = pyramid[1].at<float>(row,col);
			double l4 = pyramid[2].at<float>(row,col);
			double l5 = pyramid[3].at<float>(row,col);
			double l6 = pyramid[4].at<float>(row,col);

			csdn0.at<float>(row,col) = fabs(l2-l4);
			csdn1.at<float>(row,col) = fabs(l2-l5);
			csdn2.at<float>(row,col) = fabs(l3-l5);
			csdn3.at<float>(row,col) = fabs(l3-l6);
			

			//std::cout << "l2 = " << l2 << ", l3 = " << l3 << ", l4 = " << l4 << ", l5 = " << l5 << ", l6 = " << l6 << std::endl;
			//std::cout << csdn0.at<float>(row,col) << "  " << csdn1.at<float>(row,col) << "  " << csdn2.at<float>(row,col)<< "  " << csdn3.at<float>(row,col) << std::endl;
			//std::cout << fabs(l2-l4) << "  " << fabs(l2-l5) << "  " << fabs(l3-l5) << "  " << fabs(l2-l6) << std::endl;
			//std::cout << l3 << " - " << l5 << " = " <<  fabs(l3-l5) << std::endl;
		}
		//std::cout << pyramid[0].at<float>(row,0) << "  " << pyramid[1].at<float>(row,0) << "  " << pyramid[2].at<float>(row,0)<< "  " << pyramid[3].at<float>(row,0) << std::endl;
	}

	cv::GaussianBlur(csdn0, csdn0, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
	cv::normalize(csdn0,csdn0, 0, 1, CV_MINMAX); // PROBLEMS

	cv::GaussianBlur(csdn1, csdn1, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
	cv::normalize(csdn1,csdn1, 0, 1, CV_MINMAX);
	
	cv::GaussianBlur(csdn2, csdn2, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
	cv::normalize(csdn2,csdn2, 0, 1, CV_MINMAX);
	
	cv::GaussianBlur(csdn3, csdn3, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
	cv::normalize(csdn3,csdn3, 0, 1, CV_MINMAX);

	/*for (int i = 0; i<4; i++) {
		cv::GaussianBlur(csdn[i], csdn[i], cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
		cv::normalize(csdn[i],csdn[i], 0, 1, CV_MINMAX, csdn[i].channels());
	}*/

	csdn.push_back(csdn0);
	csdn.push_back(csdn1);
	csdn.push_back(csdn2);
	csdn.push_back(csdn3);

	//std::cout << csdn0; // HERE
	return csdn;
}

cv::Mat CSaliencyMap::getFeatureMap(std::vector<cv::Mat> csdn) {
	cv::Mat featureMap(input_img.rows, input_img.cols, CV_32F);

	cv::Mat csdn0 = csdn[0].clone();
	cv::Mat csdn1 = csdn[1].clone();
	cv::Mat csdn2 = csdn[2].clone();
	cv::Mat csdn3 = csdn[3].clone();

	for (int row=0 ; row<input_img.rows; row++) {
		for(int col=0 ; col<input_img.cols; col++) {
			double t = 	csdn0.at<float>(row,col) + csdn1.at<float>(row,col) +
						csdn2.at<float>(row,col) + csdn3.at<float>(row,col);
			csdnStore.at<float>(row,col) = t;
			//std::cout << csdn0.at<float>(row,col) << " + " << csdn1.at<float>(row,col) << " + " << csdn2.at<float>(row,col) << " + " << csdn3.at<float>(row,col) << std::endl;
			//std::cout << csdn[0].at<float>(row,col) << " + " << csdn[1].at<float>(row,col) << " + " << csdn[2].at<float>(row,col) << " + " << csdn[3].at<float>(row,col) << std::endl;
			//std::cout << t << std::endl;
		}
	}

	cv::normalize(csdnStore, csdnStore, 0, 1, CV_MINMAX);
	//*
	assert(csdnStore.cols==160);
	//assert(featureMap.cols==160);
	assert(csdnStore.rows==100);
	//assert(featureMap.rows==120);
	//assert(csdnStore.cols==featureMap.cols);
	//*/
	cv::Mat b;
	csdnStore.convertTo(b, CV_32F);
	cv::resize(b, featureMap, featureMap.size()); // put the result in feature map
	return featureMap;
}

//void CSaliencyMap::GetSaliencyMap( CvMat *intensity, CvMat *rgColor, CvMat *byColor, CvMat *form, CvMat *saliencyMap ) {
cv::Mat CSaliencyMap::getSaliencyMap(
	cv::Mat intensity,
	cv::Mat rgcolor,
	cv::Mat bycolor,
	cv::Mat form
){
	cv::Mat saliencyMap(input_img.rows, input_img.cols, CV_32F);
	for (int row=0 ; row<input_img.rows; row++) {
		for (int col=0 ; col<input_img.cols; col++ ) {
 			double t = 0.0;
			t += weightIntensity*intensity.at<float>(row,col);
			t += weightColor*rgcolor.at<float>(row,col);
			t += weightColor*bycolor.at<float>(row,col);
			t += weightForm*form.at<float>(row,col);
			
			saliencyMap.at<float>(row,col) = t;
		}
	}
	cv::normalize(saliencyMap, saliencyMap, 0, 1, CV_MINMAX);
	return saliencyMap;
}

void CSaliencyMap::setData(const cv::Mat bgr_img) {
	input_img = bgr_img.clone();
	colorFeat->setData(bgr_img);
	formFeat->setData(bgr_img);
}

void CSaliencyMap::SetColorWeight(
	double red, 
	double green, 
	double blue, 
	double yellow
){
	weightBlue = blue;
	weightRed = red;
	weightGreen = green;
	weightYellow = yellow;
}

bool CSaliencyMap::Execute(
	double weight_intensity,
	double weight_color,
	double weight_form
){
	weightIntensity = weight_intensity;
	weightColor = weight_color/2.0;
	weightForm = weight_form;
	Allocate();
	colorFeat->SetColorWeight( 1, 1, 1, 1);
	colorFeat->Execute( CColorFeature::OP_OPPONENT );
	formFeat->Execute( CFormFeature::OP_SOBEL_WINNER_EDGE );

	cv::Mat intensity(colorFeat->getIntensityChannel());
	cv::Mat rgColor(colorFeat->getOpponentRGchannel());
	cv::Mat byColor(colorFeat->getOpponentBYchannel());
	cv::Mat edge(formFeat->getSobelWinnerChannel());
	
	
	pyramidIntensity = getPyramid(intensity); // ERROR HERE
	pyramidRGcolor = getPyramid(rgColor);
	pyramidBYcolor = getPyramid(byColor);
	pyramidEdge = getPyramid(edge);

 
	csdnIntensity = getCSDN(pyramidIntensity);//GetCSDN( m_ppPyramidIntensity, m_ppCsdnIntensity );

	csdnRGcolor = getCSDN(pyramidRGcolor);//GetCSDN( m_ppPyramidRGColor, m_ppCsdnRGColor );
 	csdnBYcolor = getCSDN(pyramidBYcolor);//GetCSDN( m_ppPyramidBYColor, m_ppCsdnBYColor );
 	csdnEdge = getCSDN(pyramidEdge);//GetCSDN( m_ppPyramidEdge, m_ppCsdnEdge );
  

  	// EMPTY !!!
 	intensityFeatMap = getFeatureMap(csdnIntensity);//GetFeatureMap( m_ppCsdnIntensity, m_pIntensityFeatureMap );

    
    RGcolorFeatMap = getFeatureMap(csdnRGcolor);//GetFeatureMap( m_ppCsdnRGColor, m_pRGColorFeatureMap );
 	
	BYcolorFeatMap = getFeatureMap(csdnBYcolor);//GetFeatureMap( m_ppCsdnBYColor, m_pBYColorFeatureMap );

  	formFeatMap = getFeatureMap(csdnEdge);//GetFeatureMap( m_ppCsdnEdge, m_pFormFeatureMap );

	saliencyMap = getSaliencyMap(intensityFeatMap, 
								 RGcolorFeatMap, 
								 BYcolorFeatMap, 
								 formFeatMap
								 );
		//GetSaliencyMap( m_pIntensityFeatureMap, m_pRGColorFeatureMap, m_pBYColorFeatureMap, m_pFormFeatureMap, m_pSaliencyMap );
//*/
	return true;
}
