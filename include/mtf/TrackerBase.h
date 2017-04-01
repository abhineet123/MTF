#ifndef MTF_TRACKER_BASE_H
#define MTF_TRACKER_BASE_H

#include "opencv2/core/core.hpp"
#include "mtf/Utilities/excpUtils.h"

//! base class for all trackers
namespace mtf {
	class TrackerBase{
	protected:
		/**
		x,y coordinates of points lying along the boundary of the region
		representing the current location of the tracker
		*/
		cv::Mat cv_corners_mat;
	public:
		//! name of the tracker
		std::string name;

		TrackerBase(){}
		virtual ~TrackerBase(){}
		/**
		set the image on which subsequent calls to initialize/update/setRegion will operate;
		the idea is to call this once and then read in the following images in the sequence
		to the same memory location before calling the functions to process them;		
		*/
		virtual void setImage(const cv::Mat &img) = 0;
		/** 
		initialize the tracker to track the object present in the given location
		*/
		virtual void initialize(const cv::Mat &corners) = 0;
		/**
		find the object location in the latest image
		*/
		virtual void update() = 0;
		/**
		modify the tracker's internal state so that the tracked object is 
		placed at the given location instead of where it was after the 
		latest call to initialize/update
		*/
		virtual void setRegion(const cv::Mat& corners){
			throw mtf::utils::FunctonNotImplemented(cv::format("%s :: setRegion :: Not implemented Yet", name.c_str()));
		}
		/**
		overloaded variants for convenience
		*/
		virtual void initialize(const cv::Mat &img, const cv::Mat &corners){
			setImage(img);
			initialize(corners);
		}
		virtual void update(const cv::Mat &img){
			setImage(img);
			update();
		}
		virtual void setRegion(const cv::Mat &img, const cv::Mat &corners){
			setImage(img);
			setRegion(corners);
		}
		/**
		return the trackers's current location in the same format as provided to initialize/setRegion;
		usually a 2x4 matrix containing the corner x, y coordinates
		*/
		virtual const cv::Mat& getRegion() { return cv_corners_mat; }

		/**
		return the type of OpenCV Mat image the tracker requires as input; 
		typically either CV_8UC3 or CV_32FC1 for RGB and grayscale respectively
		*/
		virtual int inputType() const = 0;
	};
}
#endif
