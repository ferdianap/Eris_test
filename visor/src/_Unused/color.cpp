/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Ferdian Pratama
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <visor/color.hpp>

globalColorInfo extractGlobalColorInfo(
	cv_bridge::CvImageConstPtr ImagePtr,
	const int nblocks
){
	cv::Mat hsv_channels[3];
	split(ImagePtr->image,hsv_channels);
	cv::Mat h_image = hsv_channels[0];

	float sum[16]= {0}, sumsquared[16]={0};
	//  nblocks is normally = 4
	int imgRows = ImagePtr->image.rows; // 120
	int imgCols = ImagePtr->image.cols; // 160
	int imgUnitRow = imgRows/nblocks; // 120/4   = 30
	int imgUnitCol = imgCols/nblocks; // 160/4   = 40
	/***********************************************
	The block is divided as such
	0                    640 width
    ---------------------
    |  1 |  2 |  3 |  4 |
    ---------------------
    |  5 |  6 |  7 |  8 |
    ---------------------
    |  9 | 10 | 11 | 12 |
    ---------------------
    | 13 | 14 | 15 | 16 |
    ---------------------
	480 height

	***********************************************/
	unsigned char *input = (unsigned char*)(h_image.data);
	for (int row=0; row<imgRows; row++) { // traverse all row
		for (int col=0; col<imgCols; col++) { // all cols
			float pxldata = input[h_image.step * row + col];
			pxldata = pxldata/255;
			float pxldatasquared = pxldata * pxldata;
			if (col < imgUnitCol) {
				if (row < imgUnitRow) { // sum[] block 1
					sum[0] += pxldata;
					sumsquared[0] += pxldatasquared;
					//input[h_image.step * row + col] = 255; // to test each region
				} else if (row < imgUnitRow*2) { // sum[] block 5
					sum[4] += pxldata;
					sumsquared[4] += pxldatasquared;
				} else if (row < imgUnitRow*3) { // sum[] block 9
					sum[8] += pxldata;
					sumsquared[8] += pxldatasquared;
        		} else if (row < imgUnitRow*4) { // sum[] block 13
          			sum[12] += pxldata;
          			sumsquared[12] += pxldatasquared;
        		}
      		}
      		if (col > imgUnitCol-1 && col <imgUnitCol*2) {
        		if (row < imgUnitRow) { // sum[] block 2
          			sum[1] += pxldata;
          			sumsquared[1] += pxldatasquared;
        		} else if (row < imgUnitRow*2) { // sum[] block 6
          			sum[5] += pxldata;
          			sumsquared[5] += pxldatasquared;
        		} else if (row < imgUnitRow*3) { // sum[] block 10
          			sum[9] += pxldata;
          			sumsquared[9] += pxldatasquared;
        		} else if (row < imgUnitRow*4) { // sum[] block 14
          			sum[13] += pxldata;
          			sumsquared[13] += pxldatasquared;
        		}
      		}
      		if (col > imgUnitCol*2-1 && col < imgUnitCol*3) {
        		if (row < imgUnitRow) { // sum[] block 3
          			sum[2] += pxldata;
          			sumsquared[2] += pxldatasquared;
        		} else if (row < imgUnitRow*2) { // sum[] block 7
          			sum[6] += pxldata;
          			sumsquared[6] += pxldatasquared;
        		} else if (row < imgUnitRow*3) { // sum[] block 11
          			sum[10] += pxldata;
          			sumsquared[10] += pxldatasquared;
        		} else if (row < imgUnitRow*4) { // sum[] block 15
          			sum[14] += pxldata;
          			sumsquared[14] += pxldatasquared;
        		}
      		}
      		if (col > imgUnitCol*3-1 && col < imgUnitCol*4) {
        		if (row < imgUnitRow) { // sum[] block 4
        	  		sum[3] += pxldata;
          			sumsquared[3] += pxldatasquared;
        		} else if (row < imgUnitRow*2) { // sum[] block 8
          			sum[7] += pxldata;
          			sumsquared[7] += pxldatasquared;
        		} else if (row < imgUnitRow*3) { // sum[] block 12
          			sum[11] += pxldata;
          			sumsquared[11] += pxldatasquared;
        		} else if (row < imgUnitRow*4) { // sum[] block 16
          			sum[15] += pxldata;
          			sumsquared[15] += pxldatasquared;
        		}
      		}
    	}
  	}

  	// Computing mean & var, and then assign to struct and return
  	globalColorInfo ci; // initialize
  	for (int i=0; i<sizeof(sum)/sizeof(sum[0]); i++) {
		ci.mean[i] = sum[i]/(imgUnitCol*imgUnitRow);
  		ci.var[i] = sumsquared[i]/(imgUnitCol*imgUnitRow);
  	}

	/*    
	ROS_INFO_STREAM("block 1  = " << sum[0] << "\t 2  = " << sum[1] << "\t 3  = " <<sum[2] << "\t 4  = " <<sum[3]);
	ROS_INFO_STREAM("block 5  = " << sum[4] << "\t 6  = " << sum[5] << "\t 7  = " <<sum[6] << "\t 8  = " <<sum[7]);
	ROS_INFO_STREAM("block 9  = " << sum[8] << "\t 10 = " << sum[9] << "\t 11 = " <<sum[10] << "\t 12 = " <<sum[11]);
	ROS_INFO_STREAM("block 13 = " << sum[12] << "\t 14 = " << sum[13] << "\t 15 = " <<sum[14] << "\t 16 = " <<sum[15]);
	ROS_INFO_STREAM("============================");
	  
	ROS_INFO_STREAM("mean 1  = " << ci.mean[0] << "\t 2  = " << ci.mean[1] << "\t 3  = " <<ci.mean[2] << "\t 4  = " <<ci.mean[3]);
	ROS_INFO_STREAM("mean 5  = " << ci.mean[4] << "\t 6  = " << ci.mean[5] << "\t 7  = " <<ci.mean[6] << "\t 8  = " <<ci.mean[7]);
	ROS_INFO_STREAM("mean 9  = " << ci.mean[8] << "\t 10 = " << ci.mean[9] << "\t 11 = " <<ci.mean[10] << "\t 12 = " <<ci.mean[11]);
	ROS_INFO_STREAM("mean 13 = " << ci.mean[12] << "\t 14 = " << ci.mean[13] << "\t 15 = " <<ci.mean[14] << "\t 16 = " <<ci.mean[15]);
	ROS_INFO_STREAM("============================");
	ROS_INFO_STREAM("var 1  = " << ci.var[0] << "\t 2  = " << ci.var[1] << "\t 3  = " <<ci.var[2] << "\t 4  = " <<ci.var[3]);
	ROS_INFO_STREAM("var 5  = " << ci.var[4] << "\t 6  = " << ci.var[5] << "\t 7  = " <<ci.var[6] << "\t 8  = " <<ci.var[7]);
	ROS_INFO_STREAM("var 9  = " << ci.var[8] << "\t 10 = " << ci.var[9] << "\t 11 = " <<ci.var[10] << "\t 12 = " <<ci.var[11]);
	ROS_INFO_STREAM("var 13 = " << ci.var[12] << "\t 14 = " << ci.var[13] << "\t 15 = " <<ci.var[14] << "\t 16 = " <<ci.var[15]);
	ROS_INFO_STREAM("============================");
	//*/
	return ci;
}


localColorInfo extractLocalColorInfo (cv_bridge::CvImageConstPtr ImagePtr) {
	cv::Mat hsv_channels[3];
	split(ImagePtr->image,hsv_channels);
	cv::Mat h_image = hsv_channels[0];

	float sum=0, sumsquared=0;
	//  nblocks for local feat = 1
	int imgRows = ImagePtr->image.rows;
	int imgCols = ImagePtr->image.cols;

	unsigned char *input = (unsigned char*)(h_image.data);
	for (int row=0; row<imgRows; row++) { // traverse all row
    	for (int col=0; col<imgCols; col++) { // all cols
    		float pxldata = input[h_image.step * row + col];
    		pxldata /= 255;
    		float pxldatasquared = pxldata * pxldata;
    		sum += pxldata;
    		sumsquared += pxldatasquared;
		}
	}

	// Computing mean & var, and then assign to struct and return
	localColorInfo ci; // initialize
	ci.mean = sum/(imgCols*imgRows);
	ci.var = sumsquared/(imgCols*imgRows);

	return ci;
}