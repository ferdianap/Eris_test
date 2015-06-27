#ifndef EXTRACT_DATA_HPP_
#define EXTRACT_DATA_HPP_

#include <opencv2/imgproc/imgproc.hpp>

class CExtractData {
	cv::Mat input_img;//CvMat *m_pInput;

	/*int m_Width;
	int m_Height;*/
	int m_RGB;
	int m_MaskMin;
	int m_MaskMax;

	double m_EntropyRatio;

	std::vector<double> colorAvg;
		//double **m_ppColorAverage;
	//double *m_pColorAverage; //NOT USED


	double m_nObjSalThreshold;

	cv::Mat blurData;
		//CvMat *m_pBlurData;
	//bool m_WasSetData;	
	//bool m_WasExecuted;	

	void Allocate(); 
	void Deallocate();

public:
	CExtractData(void);
	virtual ~CExtractData(void);

	void SetData(cv::Mat salmap_img);
		//void SetData( CvMat *input );	
	bool Execute( int totalnum, int maskMin = 13, int maskMax = 45, double entropyRatio = 0.4 );		
	void SaliencyObjects();

	
	//CvMat *Input() const { return m_pInput; }

	//CvMat *BlurData()					const { return m_pBlurData; }
	cv::Point SaliencyPoint(int idx)  { return saliencyPoint[idx];}
	cv::Rect SaliencyRect(int idx)  { return saliencyRect[idx];}
		//CvPoint *SaliencyPoint( int idx )	const { return m_ppSaliencyPoint[idx]; }
		//CvRect *SaliencyRect( int idx )		const { return m_ppSaliencyRect[idx]; }

	int Count() const { return m_Count; }
	int m_Count;
		//CvPoint **m_ppSaliencyPoint;
		//CvRect **m_ppSaliencyRect;

	std::vector<cv::Point> saliencyPoint;
	std::vector<cv::Rect> saliencyRect;
	std::vector<double> SMavg;//double *m_pSMaverage; 
};


#endif