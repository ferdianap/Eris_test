#ifndef SALIENCY_MAP_HPP_
#define SALIENCY_MAP_HPP_

#include <opencv2/imgproc/imgproc.hpp>

class CColorFeature {
	cv::Mat input_img;

	cv::Mat red, green, blue, intensity;
	cv::Mat realRed, realGreen, realBlue, realYellow;
	cv::Mat opponentRG, opponentBY;
	cv::Mat hue, sat, val;

	double	weightBlue;
	double	weightRed;
	double	weightGreen;
	double	weightYellow;

	static const double TH_LEVEL;

	void Allocate(int flag);

	inline bool condition1(int flag) { // or static const ?
		return (flag & OP_OPPONENT) == OP_OPPONENT ? true : false;
	}

	inline bool condition2(int flag) {
		return  (flag & OP_HSV) == OP_HSV ? true : false;
	}

public:
	CColorFeature();
	virtual ~CColorFeature();

	void setData(const cv::Mat bgr_img);
	void SetColorWeight(double red, double green, double blue, double yellow);
	bool Execute( int flag );

	cv::Mat getInputImg() { return input_img; }

	cv::Mat getRedChannel()  { return red; }
	cv::Mat getGreenChannel()  { return green; }
	cv::Mat getBlueChannel()  { return blue; }
	cv::Mat getIntensityChannel()  { return intensity; }

	cv::Mat getRealRedChannel()  { return realRed; }
	cv::Mat getRealGreenChannel()  { return realGreen; }
	cv::Mat getRealBlueChannel()  { return realBlue; }
	cv::Mat getRealYellowChannel()  { return realYellow; }

	cv::Mat getOpponentRGchannel()  { return opponentRG; }
	cv::Mat getOpponentBYchannel()  { return opponentBY; }

	cv::Mat getHueChannel()  { return hue; }
	cv::Mat getSaturationChannel()  { return sat; }
	cv::Mat getValueChannel()  { return val; }

	static const int OP_RGBI		= 0x1;
	static const int OP_HSV			= OP_RGBI << 1;
	static const int OP_OPPONENT	= OP_HSV << 1;
};

//============================================================


class CFormFeature {
	cv::Mat input_img;

	CColorFeature *colorFeat;

	cv::Mat sobelRedX, sobelRedY, sobelRed;
	cv::Mat sobelGreenX, sobelGreenY, sobelGreen;
	cv::Mat sobelBlueX, sobelBlueY, sobelBlue;

	cv::Mat sobelWinner, sobelIntensityX, sobelIntensityY, sobelIntensity;

	void Allocate(int flag);

	inline bool condition1(int flag) { // or static const ?
		return (flag & OP_SOBEL_WINNER_EDGE) == OP_SOBEL_WINNER_EDGE ?
			true : false;
	}

	inline bool condition2(int flag) {
		return (flag & OP_SOBEL_INTENSITY_EDGE ) == OP_SOBEL_INTENSITY_EDGE ?
			true : false;
	}

public:
	CFormFeature();
	virtual ~CFormFeature();

	void setData (const cv::Mat bgr_img);
	bool Execute( int flag );

	cv::Mat getInputImg() { return input_img; }

	CColorFeature *ColorFeature() const { return colorFeat; }

	cv::Mat getSobelRed()  { return sobelRed; }
	cv::Mat getSobelRedX()  { return sobelRedX; }

	cv::Mat getSobelRedY()  { return sobelRedY; }

	cv::Mat getSobelWinner()  { return sobelWinner; }

	/*CvMat *SobelRedX()			const { return m_pSobelRedX; }
	CvMat *SobelRedY()			const { return m_pSobelRedY; }
	CvMat *SobelGreenX()		const { return m_pSobelGreenX; }
	CvMat *SobelGreenY()		const { return m_pSobelGreenY; }
	CvMat *SobelBlueX()			const { return m_pSobelBlueX; }
	CvMat *SobelBlueY()			const { return m_pSobelBlueY; }

	CvMat *SobelRed()			const { return m_pSobelRed; }
	CvMat *SobelGreen()			const { return m_pSobelGreen; }
	CvMat *SobelBlue()			const { return m_pSobelBlue; }

	CvMat *SobelWinner()		const { return m_pSobelWinner; }

	CvMat *SobelIntensityX()	const { return m_pSobelIntensityX; }
	CvMat *SobelIntensityY()	const { return m_pSobelIntensityY; }

	CvMat *SobelIntensity()		const { return m_pSobelIntensity; }*/

	cv::Mat getSobelWinnerChannel() { return sobelWinner; }

	static const int OP_SOBEL_WINNER_EDGE		= 0x1;
	static const int OP_SOBEL_INTENSITY_EDGE	= OP_SOBEL_WINNER_EDGE << 1;
};

//============================================================

class CSaliencyMap {
	cv::Mat input_img;

	CColorFeature *colorFeat;
	CFormFeature *formFeat;

	double weightIntensity;
	double weightColor;
	double weightForm;

	std::vector<cv::Mat> pyramidStore, pyramidEdge, pyramidIntensity;
	std::vector<cv::Mat> pyramidRGcolor, pyramidBYcolor;

	cv::Mat csdnStore;

	std::vector<cv::Mat> csdnIntensity, csdnRGcolor, csdnBYcolor, csdnEdge;

	cv::Mat intensityFeatMap, RGcolorFeatMap, BYcolorFeatMap, formFeatMap, saliencyMap;

	double	weightRed;
	double	weightGreen;
	double	weightBlue;
	double	weightYellow;

	void Allocate(); 

	std::vector<cv::Mat> getPyramid(cv::Mat input);
		//void GetPyramid( CvMat *input, CvMat **pyramid );
	std::vector<cv::Mat> getCSDN(std::vector<cv::Mat> pyramid);
		//void GetCSDN( CvMat **pyramid, CvMat **csdn );
	cv::Mat getFeatureMap(std::vector<cv::Mat> csdn);
		//void GetFeatureMap( CvMat **csdn, CvMat *featureMap );
	cv::Mat getSaliencyMap(cv::Mat intensity, // change get into calc
						   cv::Mat rgcolor,
						   cv::Mat bycolor,
						   cv::Mat form
						   );
		//void GetSaliencyMap( CvMat *intensity, CvMat *rgColor, CvMat *byColor, CvMat *form, CvMat *saliencyMap );

public:
	CSaliencyMap();
	virtual ~CSaliencyMap();

	void setData(const cv::Mat bgr_img);
	void SetColorWeight(double red, double green, double blue, double yellow);
	bool Execute( double weightIntensity, double weightColor, double weightForm );

	cv::Mat getInputImg() { return input_img; }

	cv::Mat getIntensityFeatMap() { return intensityFeatMap; }

	//CColorFeature * ColorFeature() const { return m_pColorFeature; }
	//CFormFeature * FormFeature() const { return m_pFormFeature; }
/*
	CvMat *PyramidStore( int idx )		const { return m_ppPyramidStore[idx]; }

	CvMat *PyramidIntensity( int idx )	const { return m_ppPyramidIntensity[idx]; }
	CvMat *PyramidRGColor( int idx )	const { return m_ppPyramidRGColor[idx]; }
	CvMat *PyramidBYColor( int idx )	const { return m_ppPyramidBYColor[idx]; }
	CvMat *PyramidEdge( int idx )		const { return m_ppPyramidEdge[idx]; }

	CvMat *CsdnStore()					const { return m_pCsdnStore; }

	CvMat *CsdnIntensity( int idx )		const { return m_ppCsdnIntensity[idx]; }
	CvMat *CsdnRGColor( int idx )		const { return m_ppCsdnRGColor[idx]; }
	CvMat *CsdnBYColor( int idx )		const { return m_ppCsdnBYColor[idx]; }
	CvMat *CsdnEdge( int idx )			const { return m_ppCsdnEdge[idx]; }

	CvMat *IntensityFeatureMap()		const { return m_pIntensityFeatureMap; }
	CvMat *RGColorFeatureMap()			const { return m_pRGColorFeatureMap; }
	CvMat *BYColorFeatureMap()			const { return m_pBYColorFeatureMap; }
	CvMat *FormFeatureMap()				const { return m_pFormFeatureMap; }

	CvMat *SaliencyMap()				const { return m_pSaliencyMap; }
*/

	cv::Mat SaliencyMap() { return saliencyMap; }
	static const int N_PYRAMID			= 7;
	static const int N_ACTIVE_PYRAMID	= 5;
	static const int N_CSDN				= 4;
};

#endif