/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include <iostream>
#include <ctime>

#include "mtf/ThirdParty/MIL/object_tracker.h"

namespace cv
{

	//---------------------------------------------------------------------------
	MILTrackerParams::MILTrackerParams()
	{
		// By default, use online boosting
		algorithm_ = CV_ONLINEBOOSTING;

		// Default number of classifiers/selectors for boosting algorithms
		num_classifiers_ = 100;

		// Default search region parameters for boosting algorithms
		overlap_ = 0.99f;
		search_factor_ = 2.0f;

		// Defaults for MIL tracker
		pos_radius_train_ = 4.0f;
		neg_num_train_ = 65;
		num_features_ = 250;
	}

	//---------------------------------------------------------------------------
	MILTrackerParams::MILTrackerParams(const int algorithm, const int num_classifiers, const float overlap,
		const float search_factor, const float pos_radius_train,
		const int neg_num_train, const int num_features)
	{
		// Make sure a valid algorithm flag is used before storing it
		if((algorithm != CV_ONLINEBOOSTING) && (algorithm != CV_SEMIONLINEBOOSTING) && (algorithm != CV_ONLINEMIL)
			&& (algorithm != CV_LINEMOD))
		{
			// Use CV_ERROR?
			std::cerr << "MILTrackerParams::MILTrackerParams(...) -- ERROR!  Invalid algorithm choice.\n";
			exit(-1);
		}
		// Store it
		algorithm_ = algorithm;

		// Store the number of weak classifiers (any error checking done here?)
		num_classifiers_ = num_classifiers;

		// Store information about the searching 
		overlap_ = overlap;
		search_factor_ = search_factor;

		// MIL parameters
		pos_radius_train_ = pos_radius_train;
		neg_num_train_ = neg_num_train;
		num_features_ = num_features;
	}

	//
	//
	//

	//---------------------------------------------------------------------------
	MILTrackingAlgorithm::MILTrackingAlgorithm()
	{
	}

	//---------------------------------------------------------------------------
	MILTrackingAlgorithm::~MILTrackingAlgorithm()
	{
	}

	//
	//
	//

	//---------------------------------------------------------------------------
	MILOnlineBoostingAlgorithm::MILOnlineBoostingAlgorithm()
		:
		MILTrackingAlgorithm(),
		tracker_(NULL),
		cur_frame_rep_(NULL),
		tracker_lost_(false)
	{
	}

	//---------------------------------------------------------------------------
	MILOnlineBoostingAlgorithm::~MILOnlineBoostingAlgorithm()
	{
		if(tracker_ != NULL)
		{
			delete tracker_;
			tracker_ = NULL;
		}

		if(cur_frame_rep_ != NULL)
		{
			delete cur_frame_rep_;
			cur_frame_rep_ = NULL;
		}
	}

	//---------------------------------------------------------------------------
	bool
		MILOnlineBoostingAlgorithm::initialize(const cv::Mat & image, const MILTrackerParams& params,
		const CvRect& init_bounding_box)
	{
		// Import the image
		import_image(image);

		// If the boosting tracker has already been allocated, first de-allocate it
		if(tracker_ != NULL)
		{
			std::cerr
				<< "OnlineBoostingAlgorithm::initialize(...) -- WARNING!  Boosting tracker already initialized.  Resetting now...\n"
				<< std::endl;
			delete tracker_;
			tracker_ = NULL;
		}

		// Do the same for the image frame representation
		if(cur_frame_rep_ != NULL)
		{
			std::cerr
				<< "OnlineBoostingAlgorithm::initialize(...) -- WARNING!  Boosting tracker already initialized.  Resetting now...\n"
				<< std::endl;
			delete cur_frame_rep_;
			cur_frame_rep_ = NULL;
		}

		// (Re-)Initialize the boosting tracker
		cv::Size imageSize(image_.cols, image_.rows);
		cur_frame_rep_ = new boosting::ImageRepresentation(image_, imageSize);
		cv::Rect wholeImage(0, 0, imageSize.width, imageSize.height);
		cv::Rect tracking_rect = init_bounding_box;
		tracking_rect_size_ = cv::Size(tracking_rect.width, tracking_rect.height);
		tracker_ = new boosting::BoostingTracker(cur_frame_rep_, tracking_rect, wholeImage, params.num_classifiers_);

		// Initialize some useful tracking debugging information
		tracker_lost_ = false;

		// Return success
		return true;
	}

	//---------------------------------------------------------------------------
	bool
		MILOnlineBoostingAlgorithm::update(const cv::Mat & image, const MILTrackerParams& params, cv::Rect & track_box)
	{
		// Import the image
		import_image(image);

		// Make sure the tracker has already been successfully initialized
		if(tracker_ == NULL)
		{
			std::cerr
				<< "OnlineBoostingAlgorithm::update(...) -- ERROR!  Trying to call update without properly initializing the tracker!\n"
				<< std::endl;
			return false;
		}
		if(cur_frame_rep_ == NULL)
		{
			std::cerr
				<< "OnlineBoostingAlgorithm::update(...) -- ERROR!  Trying to call update without properly initializing the tracker!\n"
				<< std::endl;
			return false;
		}

		// Calculate the patches within the search region
		cv::Size imageSize(image_.cols, image_.rows);
		cv::Rect wholeImage(0, 0, imageSize.width, imageSize.height);
		boosting::Patches *trackingPatches;
		cv::Rect searchRegion = tracker_->getTrackingROI(params.search_factor_);
		trackingPatches = new boosting::PatchesRegularScan(searchRegion, wholeImage, tracking_rect_size_, params.overlap_);

		cur_frame_rep_->setNewImageAndROI(image_, searchRegion);

		if(!tracker_->track(cur_frame_rep_, trackingPatches))
		{
			tracker_lost_ = true;
		} else
		{
			tracker_lost_ = false;
		}

		delete trackingPatches;

		// Save the new tracking ROI
		track_box = tracker_->getTrackedPatch();
		std::cout << "\rTracking confidence = " << tracker_->getConfidence();

		// Return success or failure based on whether or not the tracker has been lost
		return !tracker_lost_;
	}

	//---------------------------------------------------------------------------
	void
		MILOnlineBoostingAlgorithm::import_image(const cv::Mat & image)
	{
		// We want the internal version of the image to be gray-scale, so let's
		// do that here.  We'll handle cases where the input is either RGB, RGBA,
		// or already gray-scale.  I assume it's already 8-bit.  If not then 
		// an error is thrown.  I'm not going to deal with converting properly
		// from every data type since that shouldn't be happening.

		// Make sure the input image pointer is valid
		if(image.empty())
		{
			std::cerr << "OnlineBoostingAlgorithm::import_image(...) -- ERROR!  Input image pointer is NULL!\n" << std::endl;
			exit(0); // <--- CV_ERROR?
		}

		// Now copy it in appropriately as a gray-scale, 8-bit image
		if(image.channels() == 4)
		{
			cv::cvtColor(image, image_, CV_RGBA2GRAY);
		} else if(image.channels() == 3)
		{
			cv::cvtColor(image, image_, CV_RGB2GRAY);
		} else if(image.channels() == 1)
		{
			image.copyTo(image_);
		} else
		{
			std::cerr << "OnlineBoostingAlgorithm::import_image(...) -- ERROR!  Invalid number of channels for input image!\n"
				<< std::endl;
			exit(0);
		}
	}
	//
	//
	//

	//---------------------------------------------------------------------------
	MILSemiOnlineBoostingAlgorithm::MILSemiOnlineBoostingAlgorithm()
		:
		MILTrackingAlgorithm(),
		tracker_(0),
		cur_frame_rep_(0),
		tracker_lost_(false)
	{
	}

	//---------------------------------------------------------------------------
	MILSemiOnlineBoostingAlgorithm::~MILSemiOnlineBoostingAlgorithm()
	{
		if(tracker_ != NULL)
		{
			delete tracker_;
			tracker_ = NULL;
		}

		if(cur_frame_rep_ != NULL)
		{
			delete cur_frame_rep_;
			cur_frame_rep_ = NULL;
		}
	}

	//---------------------------------------------------------------------------
	bool
		MILSemiOnlineBoostingAlgorithm::initialize(const cv::Mat & image, const MILTrackerParams& params,
		const CvRect& init_bounding_box)
	{
		// Import the image
		import_image(image);

		// If the boosting tracker has already been allocated, first de-allocate it
		if(tracker_ != NULL)
		{
			std::cerr
				<< "OnlineBoostingAlgorithm::initialize(...) -- WARNING!  Boosting tracker already initialized.  Resetting now...\n"
				<< std::endl;
			delete tracker_;
			tracker_ = NULL;
		}

		// Do the same for the image frame representation
		if(cur_frame_rep_ != NULL)
		{
			std::cerr
				<< "OnlineBoostingAlgorithm::initialize(...) -- WARNING!  Boosting tracker already initialized.  Resetting now...\n"
				<< std::endl;
			delete cur_frame_rep_;
			cur_frame_rep_ = NULL;
		}

		// (Re-)Initialize the boosting tracker
		cv::Size imageSize(image_.cols, image_.rows);
		cur_frame_rep_ = new boosting::ImageRepresentation(image_, imageSize);
		cv::Rect wholeImage = cv::Rect(0, 0, imageSize.width, imageSize.height);
		cv::Rect tracking_rect = init_bounding_box;
		tracking_rect_size_ = cv::Size(tracking_rect.width, tracking_rect.height);
		tracker_ = new boosting::SemiBoostingTracker(cur_frame_rep_, tracking_rect, wholeImage, params.num_classifiers_);

		// Initialize some useful tracking debugging information
		tracker_lost_ = false;

		// Return success
		return true;
	}

	//---------------------------------------------------------------------------
	bool
		MILSemiOnlineBoostingAlgorithm::update(const cv::Mat & image, const MILTrackerParams& params, cv::Rect & track_box)
	{
		// Import the image
		import_image(image);

		// Make sure the tracker has already been successfully initialized
		if(tracker_ == NULL)
		{
			std::cerr
				<< "OnlineBoostingAlgorithm::update(...) -- ERROR!  Trying to call update without properly initializing the tracker!\n"
				<< std::endl;
			return false;
		}
		if(cur_frame_rep_ == NULL)
		{
			std::cerr
				<< "OnlineBoostingAlgorithm::update(...) -- ERROR!  Trying to call update without properly initializing the tracker!\n"
				<< std::endl;
			return false;
		}

		// Calculate the patches within the search region
		cv::Size imageSize(image_.cols, image_.rows);
		cv::Rect wholeImage = cv::Rect(0, 0, imageSize.width, imageSize.height);
		boosting::Patches *trackingPatches;
		cv::Rect searchRegion = tracker_->getTrackingROI(params.search_factor_);
		trackingPatches = new boosting::PatchesRegularScan(searchRegion, wholeImage, tracking_rect_size_, params.overlap_);

		cur_frame_rep_->setNewImageAndROI(image_, searchRegion);

		if(!tracker_->track(cur_frame_rep_, trackingPatches))
		{
			tracker_lost_ = true;
		} else
		{
			tracker_lost_ = false;
		}

		delete trackingPatches;

		// Save the new tracking ROI
		track_box = tracker_->getTrackedPatch();
		std::cout << "\rTracking confidence = " << tracker_->getConfidence();

		// Return success or failure based on whether or not the tracker has been lost
		return !tracker_lost_;
	}

	//---------------------------------------------------------------------------
	void
		MILSemiOnlineBoostingAlgorithm::import_image(const cv::Mat & image)
	{
		// We want the internal version of the image to be gray-scale, so let's
		// do that here.  We'll handle cases where the input is either RGB, RGBA,
		// or already gray-scale.  I assume it's already 8-bit.  If not then 
		// an error is thrown.  I'm not going to deal with converting properly
		// from every data type since that shouldn't be happening.

		// Make sure the input image pointer is valid
		if(image.empty())
		{
			std::cerr << "OnlineBoostingAlgorithm::import_image(...) -- ERROR!  Input image pointer is NULL!\n" << std::endl;
			exit(0); // <--- CV_ERROR?
		}

		// Now copy it in appropriately as a gray-scale, 8-bit image
		if(image.channels() == 4)
		{
			cv::cvtColor(image, image_, CV_RGBA2GRAY);
		} else if(image.channels() == 3)
		{
			cv::cvtColor(image, image_, CV_RGB2GRAY);
		} else if(image.channels() == 1)
		{
			image.copyTo(image_);
		} else
		{
			std::cerr << "OnlineBoostingAlgorithm::import_image(...) -- ERROR!  Invalid number of channels for input image!\n"
				<< std::endl;
			exit(0);
		}
	}

	//
	//
	//

	//---------------------------------------------------------------------------
	MILOnlineMILAlgorithm::MILOnlineMILAlgorithm()
		:
		MILTrackingAlgorithm(),
		is_initialized(false)
	{
		cv::mil::RandomGenerator::initialize((int)time(0));
		clfparams_ = new cv::mil::ClfMilBoostParams();
		ftrparams_ = &haarparams_;
		clfparams_->_ftrParams = ftrparams_;
	}

	//---------------------------------------------------------------------------
	MILOnlineMILAlgorithm::~MILOnlineMILAlgorithm()
	{
		delete clfparams_;
	}

	//---------------------------------------------------------------------------
	bool
		MILOnlineMILAlgorithm::initialize(const cv::Mat & image, const MILTrackerParams& params,
		const CvRect& init_bounding_box)
	{
		import_image(image);

		((cv::mil::ClfMilBoostParams*) clfparams_)->_numSel = params.num_classifiers_;
		((cv::mil::ClfMilBoostParams*) clfparams_)->_numFeat = params.num_features_;
		tracker_params_._posradtrain = params.pos_radius_train_;
		tracker_params_._negnumtrain = params.neg_num_train_;

		// Tracking parameters
		tracker_params_._init_negnumtrain = 65;
		tracker_params_._init_postrainrad = 3.0f;
		tracker_params_._initstate[0] = (float)init_bounding_box.x;
		tracker_params_._initstate[1] = (float)init_bounding_box.y;
		tracker_params_._initstate[2] = (float)init_bounding_box.width;
		tracker_params_._initstate[3] = (float)init_bounding_box.height;
		tracker_params_._srchwinsz = 25;
		tracker_params_._negsamplestrat = 1;
		tracker_params_._initWithFace = false;
		tracker_params_._debugv = false;
		tracker_params_._disp = false; // set this to true if you want to see video output (though it slows things down)

		clfparams_->_ftrParams->_width = (cv::mil::uint) init_bounding_box.width;
		clfparams_->_ftrParams->_height = (cv::mil::uint) init_bounding_box.height;

		tracker_.init(image_, tracker_params_, clfparams_);

		// Return success
		is_initialized = true;
		return true;
	}

	//---------------------------------------------------------------------------
	bool
		MILOnlineMILAlgorithm::update(const cv::Mat & image, const MILTrackerParams& params, cv::Rect & track_box)
	{
		if(!is_initialized)
		{
			std::cerr << "OnlineMILAlgorithm::update() -- Error!  Did not intialize algorithm!\n" << std::endl;
			return false;
		}

		import_image(image);

		// Update tracker
		tracker_.track_frame(image_);

		// Save output
		tracker_.getTrackBox(track_box);

		// Return success
		return true;
	}

	//---------------------------------------------------------------------------
	void
		MILOnlineMILAlgorithm::import_image(const cv::Mat & image)
	{
		// We want the internal version of the image to be gray-scale, so let's
		// do that here.  We'll handle cases where the input is either RGB, RGBA,
		// or already gray-scale.  I assume it's already 8-bit.  If not then 
		// an error is thrown.  I'm not going to deal with converting properly
		// from every data type since that shouldn't be happening.

		// Make sure the input image pointer is valid
		if(image.empty())
		{
			std::cerr << "OnlineBoostingAlgorithm::import_image(...) -- ERROR!  Input image pointer is NULL!\n" << std::endl;
			exit(0); // <--- CV_ERROR?
		}

		// Now copy it in appropriately as a gray-scale, 8-bit image
		if(image.channels() == 4)
		{
			cv::cvtColor(image, image_, CV_RGBA2GRAY);
		} else if(image.channels() == 3)
		{
			cv::cvtColor(image, image_, CV_RGB2GRAY);
		} else if(image.channels() == 1)
		{
			image.copyTo(image_);
		} else
		{
			std::cerr << "OnlineBoostingAlgorithm::import_image(...) -- ERROR!  Invalid number of channels for input image!\n"
				<< std::endl;
			exit(0);
		}
	}

	//
	//
	//

	//---------------------------------------------------------------------------
	LINEMODAlgorithm::LINEMODAlgorithm()
		:
		MILTrackingAlgorithm()
	{
	}

	//---------------------------------------------------------------------------
	LINEMODAlgorithm::~LINEMODAlgorithm()
	{
	}

	//---------------------------------------------------------------------------
	bool
		LINEMODAlgorithm::initialize(const cv::Mat & image, const MILTrackerParams& params,
		const CvRect& init_bounding_box)
	{
		// Return success
		return true;
	}

	//---------------------------------------------------------------------------
	bool
		LINEMODAlgorithm::update(const cv::Mat & image, const MILTrackerParams& params, cv::Rect & track_box)
	{
		// Return success
		return true;
	}

	//---------------------------------------------------------------------------
	void
		LINEMODAlgorithm::import_image(const cv::Mat & image)
	{
	}

	//
	//
	//

	//---------------------------------------------------------------------------
	MILTracker::MILTracker(const MILTrackerParams& params)
		:
		initialized_(false),
		tracker_(NULL)
	{
		// Store configurable parameters internally
		set_params(params);

		// Allocate the proper tracking algorithm (note: error-checking that a valid
		// tracking algorithm parameter is used is done in the MILTrackerParams
		// constructor, so at this point we are confident it's valid).
		switch(params.algorithm_)
		{
		case MILTrackerParams::CV_ONLINEBOOSTING:
			tracker_ = new MILOnlineBoostingAlgorithm();
			break;
		case MILTrackerParams::CV_SEMIONLINEBOOSTING:
			tracker_ = new MILSemiOnlineBoostingAlgorithm();
			break;
		case MILTrackerParams::CV_ONLINEMIL:
			tracker_ = new MILOnlineMILAlgorithm();
			break;
		case MILTrackerParams::CV_LINEMOD:
			tracker_ = new LINEMODAlgorithm();
			break;
		default:
			// By default, if an invalid choice somehow gets through lets use online boosting?
			// Or throw an error and don't continue?
			tracker_ = new MILOnlineBoostingAlgorithm();
			break;
		}
	}

	//---------------------------------------------------------------------------
	MILTracker::~MILTracker()
	{
		// Delete the tracking algorithm object, if it was allocated properly
		if(tracker_ != NULL)
		{
			delete tracker_;
		}
	}

	//---------------------------------------------------------------------------
	bool
		MILTracker::initialize(const cv::Mat & image, const CvRect& bounding_box)
	{
		// Initialize the tracker and if it works, set the flag that we're now initialized
		// to true so that update() can work properly.
		//printf("Using MIL tracker with:\n");
		//printf("algorithm: %d\n", tracker_params_.algorithm_);
		//printf("num_classifiers: %d\n", tracker_params_.num_classifiers_);
		//printf("overlap: %f\n", tracker_params_.overlap_);
		//printf("search_factor: %f\n", tracker_params_.search_factor_);
		//printf("pos_radius_train: %f\n", tracker_params_.pos_radius_train_);
		//printf("neg_num_train: %d\n", tracker_params_.neg_num_train_);
		//printf("num_features: %d\n", tracker_params_.num_features_);
		//printf("\n");
		bool success = tracker_->initialize(image, tracker_params_, bounding_box);
		if(success)
		{
			initialized_ = true;
		} else
		{
			initialized_ = false;
		}

		// Return success or failure
		return success;
	}

	//---------------------------------------------------------------------------
	bool
		MILTracker::update(const cv::Mat & image, cv::Rect & track_box)
	{
		// First make sure we have already initialized.  Otherwise we can't continue.
		if(!initialized_)
		{
			std::cerr << "ObjectTracker::update() -- ERROR! The ObjectTracker needs to be initialized before updating.\n";
			return false;
		}

		// Update the tracker and return whether or not it succeeded
		bool success = tracker_->update(image, tracker_params_, track_box);

		// Return success or failure
		return success;
	}

	//---------------------------------------------------------------------------
	void
		MILTracker::set_params(const MILTrackerParams& params)
	{
		tracker_params_ = params;
	}

}
