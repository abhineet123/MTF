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

#ifndef __OPENCV_OBJECT_TRACKER_H__
#define __OPENCV_OBJECT_TRACKER_H__

#include <opencv2/core/core.hpp>

#include "cv_onlineboosting.h"
#include "cv_onlinemil.h"
//#include "cv_linemod.h"

namespace cv
{

  // Various parameters used by the ObjectTracker class.  This may also
  // be passed to the specific tracking algorithm classes, and so we'll
  // keep the configurable variable names generic enough to be used across
  // different algorithms
  //
  // :TODO: to be filled-in as needed...
  //
  struct ObjectTrackerParams
  {
    enum
    {
      CV_ONLINEBOOSTING = 100, CV_SEMIONLINEBOOSTING, CV_ONLINEMIL, CV_LINEMOD
    };

    ObjectTrackerParams();
    ObjectTrackerParams(const int algorithm, const int num_classifiers, const float overlap, const float search_factor,
                        const float pos_radius_train, const int neg_num_train, const int num_features);

    int algorithm_; // CV_ONLINEBOOSTING, CV_SEMIONLINEBOOSTING, CV_ONLINEMIL, CV_LINEMOD
    int num_classifiers_; // the number of classifiers to use in a given boosting algorithm (OnlineBoosting, MIL)
    float overlap_; // search region parameters to use in a given boosting algorithm (OnlineBoosting, MIL)
    float search_factor_; // search region parameters to use in a given boosting algorithm (OnlineBoosting, MIL)

    // The following are specific to the MIL algorithm
    float pos_radius_train_; // radius for gathering positive instances
    int neg_num_train_; // # negative samples to use during training
    int num_features_;
  };

  //
  //
  //

  // The base (abstract) tracking algorithm class, to define the interface
  // for all other specific object tracking algorithm classes.
  class TrackingAlgorithm
  {
  public:
    TrackingAlgorithm();
    virtual
    ~TrackingAlgorithm();

    virtual bool
    initialize(const cv::Mat & image, const ObjectTrackerParams& params, const CvRect& init_bounding_box) = 0;

    virtual bool
    update(const cv::Mat & image, const ObjectTrackerParams& params, cv::Rect & track_box) = 0;

  protected:
    // A method to import an image to the type desired for the current algorithm
    virtual void
    import_image(const cv::Mat & image) = 0;

    // A local image holder (can be gray-scale, color, depth image 16-bit, whatever
    // you want...)
    cv::Mat image_;
  };

  //
  //
  //

  // A speficic instance of a tracking algorithm: Online Boosting as described
  // in the following paper:
  //
  // H. Grabner, M. Grabner, and H. Bischof.  "Real-time Tracking via On-line Boosting", 
  // In Proceedings British Machine Vision Conference (BMVC), volume 1, pages 47-56, 2006.
  //
  class OnlineBoostingAlgorithm: public TrackingAlgorithm
  {
  public:
    OnlineBoostingAlgorithm();
    ~OnlineBoostingAlgorithm();

    virtual bool
    initialize(const cv::Mat & image, const ObjectTrackerParams& params, const CvRect& init_bounding_box);

    virtual bool
    update(const cv::Mat & image, const ObjectTrackerParams& params, cv::Rect & track_box);

  protected:
    // A method to import an image to the type desired for the current algorithm
    virtual void
    import_image(const cv::Mat & image);

  private:
    // The main boosting tracker object
    boosting::BoostingTracker* tracker_;

    // The main image frame representation, useful for the boosting tracker
    boosting::ImageRepresentation* cur_frame_rep_;

    // The overall tracking rectangle region-of-interesting
    cv::Size tracking_rect_size_;

    // Keep track of whether or not the tracker has been lost on a given frame
    bool tracker_lost_;
  };

  //
  //
  //

  // A speficic instance of a tracking algorithm: Semi-Online Boosting as described
  // in the following paper:
  //
  // H. Grabner, C. Leistner, and H. Bischof.  "Semi-supervised On-line Boosting for 
  // Robust Tracking", In Proceedings European Conference on Computer Vision (ECCV), 
  // 2008.
  //
  class SemiOnlineBoostingAlgorithm: public TrackingAlgorithm
  {
  public:
    SemiOnlineBoostingAlgorithm();
    ~SemiOnlineBoostingAlgorithm();

    virtual bool
    initialize(const cv::Mat & image, const ObjectTrackerParams& params, const CvRect& init_bounding_box);

    virtual bool
    update(const cv::Mat & image, const ObjectTrackerParams& params, cv::Rect & track_box);

  protected:
    // A method to import an image to the type desired for the current algorithm
    virtual void
    import_image(const cv::Mat & image);

  private:
    // The main boosting tracker object
    boosting::SemiBoostingTracker* tracker_;

    // The main image frame representation, useful for the boosting tracker
    boosting::ImageRepresentation* cur_frame_rep_;

    // The overall tracking rectangle region-of-interesting
    cv::Size tracking_rect_size_;

    // Keep track of whether or not the tracker has been lost on a given frame
    bool tracker_lost_;
  };

  //
  //
  //

  // A speficic instance of a tracking algorithm: Online Multiple Instance Learning
  // as described in the following paper:
  //
  // B. Babenko, M.H. Yang, and S. Belongie.  "Visual Tracking with Online Multiple 
  // Instance Learning", CVPR 2009, Miami, Florida.
  //
  class OnlineMILAlgorithm: public TrackingAlgorithm
  {
  public:
    OnlineMILAlgorithm();
    ~OnlineMILAlgorithm();

    virtual bool
    initialize(const cv::Mat & image, const ObjectTrackerParams& params, const CvRect& init_bounding_box);

    virtual bool
    update(const cv::Mat & image, const ObjectTrackerParams& params, cv::Rect & track_box);

  protected:
    // A method to import an image to the type desired for the current algorithm
    virtual void
    import_image(const cv::Mat & image);

  private:
    // The main Online MIL tracker
    cv::mil::SimpleTracker tracker_;
    cv::mil::SimpleTrackerParams tracker_params_;
    cv::mil::ClfStrongParams* clfparams_;

    // Feature parameters
    cv::mil::FtrParams* ftrparams_;
    cv::mil::HaarFtrParams haarparams_;

    bool is_initialized;
  };

  //
  //
  //

  // A speficic instance of a tracking algorithm: LINEMOD as described in the 
  // following paper:
  //
  // ... fill-in when paper is accepted and we have permission ...
  //
  class LINEMODAlgorithm: public TrackingAlgorithm
  {
  public:
    LINEMODAlgorithm();
    ~LINEMODAlgorithm();

    virtual bool
    initialize(const cv::Mat & image, const ObjectTrackerParams& params, const CvRect& init_bounding_box);

    virtual bool
    update(const cv::Mat & image, const ObjectTrackerParams& params, cv::Rect & track_box);

  protected:
    // A method to import an image to the type desired for the current algorithm
    virtual void
    import_image(const cv::Mat & image);
  };

  //
  //
  //

  // The main Object Tracking class, which implements different object tracking
  // algorithms.  See the specific class instances above.  This is the class
  // that the user will use.  We use an inheritance hierarchy to determine which
  // tracking algorithm to implement, based on the 'algorithm' field
  // in the ObjectTrackerParams.
  //
  class ObjectTracker
  {
  public:
    // Default constructor--performs initializations of the class and memory.
    // Input the tracking parameters (which are defaulted if you don't give
    // any).
    //
    ObjectTracker(const ObjectTrackerParams& params = ObjectTrackerParams());

    // Destructor--performs cleanup of memory 
    virtual
    ~ObjectTracker();

    // Initialize the tracker (on the first frame only!) with an image
    // and a bounding box representing the region of the object to be tracked.
    // 
    // If initialization has already occurred, don't do anything and return false.
    // If for some reason the initialization fails (boundary conditions, etc) then return false.
    // Otherwise return true for successful initialization.
    //
    // :TODO:       adr: 05/23/2011
    // -- What image types are allowed?  Only gray-scale, or will we accept color too?
    //    How about byte size?  Normally I'd say only IPL_DEPTH_8U, but if we want to allow
    //    depth images we'll probably also allow IPL_DEPTH_16S.
    // -- Be sure to do boundary checks on 'boundingBox' to account for user error.
    //
    virtual bool
    initialize(const cv::Mat & image, const CvRect& bounding_box);

    // Update the state of the tracker.  This assumes that initialization has already occurred.
    // If not, false will immediately be returned.  The new tracking bounding box will be 
    // returned via a pointer to 'trackBox' of type CvRect, giving the upper-left corner of the
    // new track box as well as the width/height.  Some trackers may or may not update the size
    // of the bounding box from what was used in initialization, but just in case leave this in.
    // Otherwise I'd say just return a CvPoint* indicating a new centroid.
    //
    // Optionally return a likelihood image, which is the same image dimensions as the input images,
    // but of type IPL_DEPTH_32F of floating point values of confidences-per-pixel of the object
    // being tracked.
    //
    // Finally, return true if the tracker succeeded and false otherwise.
    //
    // :TODO:       adr: 05/23/2011
    // -- Do we want to take any other inputs to this function?
    //
    virtual bool
    update(const cv::Mat & image, cv::Rect & track_box);

    // Store the input parameters internally
    void
    set_params(const ObjectTrackerParams& params);

  private:
    // A flag indicating whether or not this tracker has been initialized yet.
    // It's important to keep track of so the user doesn't try to track
    // without initializing first!
    bool initialized_;

    // The object tracking algorithm parameters
    ObjectTrackerParams tracker_params_;

    // The actual tracking algorithm
    TrackingAlgorithm* tracker_;
  };

}

#endif  // #ifndef __OPENCV_OBJECT_TRACKER_H__
/* End of file. */
