/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/

/*
 * TLD.h
 *
 *  Created on: Nov 17, 2011
 *      Author: Georg Nebehay
 */

#ifndef MTF_TLD_H_
#define MTF_TLD_H_

#include <opencv/cv.h>

#include "MedianFlowTracker.h"
#include "DetectorCascade.h"
#include "mtf/TrackerBase.h"

struct TLDParams{
	bool trackerEnabled;
	bool detectorEnabled;
	bool learningEnabled;
	bool alternating;
	int init_border_size;

	TLDParams(bool _trackerEnabled, bool _detectorEnabled,
		bool _learningEnabled, bool _alternating);
	TLDParams(const TLDParams *params = nullptr);
};

namespace tld
{

	class TLD : public mtf::TrackerBase
	{
		void storeCurrentData();
		void fuseHypotheses();
		void learn();
		void initialLearning();
	public:
		typedef TLDParams ParamType;
		ParamType params;

		bool trackerEnabled;
		bool detectorEnabled;
		bool learningEnabled;
		bool alternating;

		MedianFlowTracker *medianFlowTracker;
		DetectorCascade *detectorCascade;
		NNClassifier *nnClassifier;
		bool valid;
		bool wasValid;
		cv::Mat prevImg;
		cv::Mat currImg;
		cv::Rect *prevBB;
		cv::Rect *currBB;
		float currConf;
		bool learning;

		cv::Mat cv_img;

		TLD();
		TLD(const ParamType *tld_params = nullptr);
		virtual ~TLD();

		void release();
		void selectObject(const cv::Mat &img, cv::Rect *bb);
		void processImage(const cv::Mat &img);
		void writeToFile(const char *path);
		void readFromFile(const char *path);

		void initialize(const cv::Mat& cv_corners) override;
		void setImage(const cv::Mat &img) override{ cv_img = img; }
		int inputType() const  override{ return CV_8UC1; }
		void update() override{
			processImage(cv_img);
			updateCVCorners();
		}
		void updateCVCorners();
	};

} /* namespace tld */
#endif /* TLD_H_ */
