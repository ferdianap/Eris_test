#include <visor/Vattn/extract_data.hpp>
#include "cv.h"

CExtractData::CExtractData(void)
{
	m_Count = 0;
	m_RGB = 3;
	m_MaskMin = 0;
	m_MaskMax = 0;
	m_nObjSalThreshold = 0.5;

	m_EntropyRatio = 0.0;
}

CExtractData::~CExtractData(void) {
}

void CExtractData::Allocate()
{
	blurData = cv::Mat(input_img.rows, input_img.cols, CV_32F);

	cv::Point emptyPoint(0,0);
	cv::Rect emptyRect(0,0,0,0);
	for (int i=0; i<m_Count; i++) {
		saliencyPoint.push_back(emptyPoint);
		saliencyRect.push_back(emptyRect);
	}
		//m_ppSaliencyPoint = new CvPoint*[m_Count];
		//m_ppSaliencyRect = new CvRect*[m_Count];
	
	colorAvg.push_back(0.);
	colorAvg.push_back(0.);
	colorAvg.push_back(0.); // 3x
		//m_ppColorAverage = new double*[m_RGB];
	//SMavg.push_back(0.0);
		//m_pSMaverage = new double [m_Count];
		//m_pColorAverage = new double [m_Count];// NOT USED

	/*for( int idx=0;idx<m_Count;idx++) {
		m_ppSaliencyPoint[idx] = new CvPoint();
		m_ppSaliencyRect[idx] = new CvRect();
	}*/

	/*for( int i=0;i<m_RGB;i++) {
		m_ppColorAverage[i] = new double[m_Count];
	}*/
}

void CExtractData::Deallocate() {
}

void CExtractData::SetData(cv::Mat salmap_img) {
	input_img = salmap_img.clone();
}

bool CExtractData::Execute( int totalnum, int maskMin, int maskMax, double entropyRatio) {
	m_Count = totalnum;
	m_MaskMin = maskMin;
	m_MaskMax = maskMax;
	m_EntropyRatio = entropyRatio;

	Allocate();
	//std::cout << input_img.at<float>(60,80) << std::endl;
	cv::blur(input_img, blurData, cv::Size(3,3));
	//std::cout << blurData.at<float>(60,80) << std::endl;
	for (int idx=0; idx<m_Count; idx++) {
		double maxValue = 0.0;
		for (int y=0;y<input_img.rows-m_MaskMin;y++) {
			for (int x=0;x<input_img.cols-m_MaskMin;x++) {
				double tempSum=0;
				for (int i=0;i<m_MaskMin;i++) {
					for (int j=0;j<m_MaskMin;j++) {
						tempSum += blurData.at<float>(y+i, x+j);
							//cvmGet( m_pBlurData, y+i, x+j);
					}
				}
				tempSum /= (m_MaskMin*m_MaskMin);
				if (maxValue < tempSum) {
					maxValue = tempSum;
					saliencyPoint[idx].x = x+int(m_MaskMin/2);
					saliencyPoint[idx].y = y+int(m_MaskMin/2);
						//m_ppSaliencyPoint[idx]->x = x+int(m_MaskMin/2);
						//m_ppSaliencyPoint[idx]->y = y+int(m_MaskMin/2);
				}
			}
		}

		double ratio, sum, pastAvg;
		int size = m_MaskMin;
		int px = saliencyPoint[idx].x;//m_ppSaliencyPoint[idx]->x;
		int py = saliencyPoint[idx].y;//m_ppSaliencyPoint[idx]->y;
		//std::cout << px << ", " << py << std::endl;
		pastAvg = maxValue;

		for (int tp = 0; tp<(m_MaskMax-m_MaskMin)/2; tp++) {
			if (py-size/2 <= 0)  				py = size/2;
			if (py+size/2>=input_img.rows-1)    py = input_img.rows-1-size/2;
			if (px-size/2 <= 0)  				px = size/2;
			if (px+size/2>=input_img.cols-1)    px = input_img.cols-1-size/2;
			
			SMavg.push_back(0.0);//m_pSMaverage[idx] = 0.0;
			sum = 0.0;
			int cnt = 0;
			
			for (int y=py-size/2; y<=py+size/2; y++) {
				for (int x=px-size/2; x<=px+size/2; x++) {
					sum += blurData.at<float>(y,x);
					cnt = cnt++;
				}
			}

			SMavg[idx]=sum/(size*size);//m_pSMaverage[idx] = sum/(size*size);

			//std::cout << SMavg[idx] <<  "    "<<(sum/(size*size)) << std::endl;
			ratio = SMavg[idx]/pastAvg;//m_pSMaverage[idx]/pastAvg;
			//std::cout <<ratio << "  " << (sum/(size*size))/maxValue << "       " << sum/(size*size) << "  " << maxValue << std::endl;
			if (ratio < m_EntropyRatio) {
				if (sum / cnt > .30) {
					break;
				} else {
					size = size + 2;
					break;
				}
			} else {
				size = size+2;
			}
			//*/
		}
		//std::cout << size << std::endl;
//*
		saliencyRect[idx].width = size;
		saliencyRect[idx].height = size;//m_ppSaliencyRect[idx]->width = m_ppSaliencyRect[idx]->height = size;
		saliencyRect[idx].x = px - size/2;//m_ppSaliencyRect[idx]->x = px - size/2;
		saliencyRect[idx].y = py - size/2;//m_ppSaliencyRect[idx]->y = py - size/2;

		for (int y=py-size/2; y<=py+size/2; y++) {
			for (int x=px-size/2; x<=px+size/2; x++) {
				if (y >= 0 && y < blurData.rows && x >= 0 && x < blurData.cols )
					blurData.at<float>(y,x) = 0.;
			}
		}
		//*/
	}

	return true;
}

void CExtractData::SaliencyObjects()
{
	CvPoint startPoint;
	CvPoint endPoint;
	int k=0; 

	double *buff1;	// value
	CvRect **buff2;	// sp

	buff1 = new double [m_Count];
	buff2 = new CvRect *[m_Count];

	for(int idx=0;idx<m_Count;idx++)
	{
		buff1[idx] = SMavg[idx];//m_pSMaverage[idx];
		buff2[idx] = new CvRect();
		buff2[idx]->x = saliencyRect[idx].x;//m_ppSaliencyRect[idx]->x;
		buff2[idx]->y = saliencyRect[idx].y;//m_ppSaliencyRect[idx]->y;
		buff2[idx]->width = saliencyRect[idx].width;//m_ppSaliencyRect[idx]->width;
		buff2[idx]->height = saliencyRect[idx].height;//m_ppSaliencyRect[idx]->height;
		SMavg[idx]=0;//m_pSMaverage[idx] = 0;
	}

	for(int idx=0;idx<m_Count;idx++)
	{

		if(buff1[idx]>m_nObjSalThreshold)
		{
			startPoint.x = saliencyRect[idx].x;//m_ppSaliencyRect[idx]->x;
			startPoint.y = saliencyRect[idx].y;//m_ppSaliencyRect[idx]->y;

			endPoint.x = startPoint.x + saliencyRect[idx].width;//m_ppSaliencyRect[idx]->width;
			endPoint.y = startPoint.y + saliencyRect[idx].height;//m_ppSaliencyRect[idx]->height;

			saliencyRect[idx].x = buff2[idx]->x;//m_ppSaliencyRect[idx]->x= buff2[idx]->x;
			saliencyRect[idx].y = buff2[idx]->y;//m_ppSaliencyRect[idx]->y= buff2[idx]->y;
			saliencyRect[idx].width = buff2[idx]->width;//m_ppSaliencyRect[idx]->width= buff2[idx]->width;
			saliencyRect[idx].height = buff2[idx]->height;//m_ppSaliencyRect[idx]->height= buff2[idx]->height;

			SMavg[k]=buff1[idx];//m_pSMaverage[k] = buff1[idx];
			k++;
		}

	}
	delete buff1;
	for(int i=0;i<m_Count;i++)
		delete buff2[i];
	delete buff2;
	m_Count = k;
}

