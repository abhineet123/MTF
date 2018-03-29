#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/excpUtils.h"
#include "opencv2/highgui/highgui.hpp"
#if CV_MAJOR_VERSION == 3
#include "opencv2/imgproc/imgproc.hpp"
#endif
#ifndef DISABLE_VISP
//#if defined _WIN32
//#define VISP_HAVE_X11
//#endif
#include <visp3/core/vpImagePoint.h>
#include <visp3/gui/vpDisplayX.h>
#endif
#include <fstream>
#include <string>
#ifdef _WIN32
#include <sstream>
#endif
#include <ctime>

#define mat_to_pt(vert, id) cv::Point2d(vert.at<double>(0, id), vert.at<double>(1, id))
#define eig_to_pt(vert, id) cv::Point2d(vert(0, id), vert(1, id))


_MTF_BEGIN_NAMESPACE

namespace utils{

	void drawCorners(cv::Mat &img, const cv::Point2d(&corners)[4],
		const cv::Scalar corners_col, const std::string label){
		line(img, corners[0], corners[1], corners_col, 2);
		line(img, corners[1], corners[2], corners_col, 2);
		line(img, corners[2], corners[3], corners_col, 2);
		line(img, corners[3], corners[0], corners_col, 2);
		putText(img, label, corners[0],
			cv::FONT_HERSHEY_SIMPLEX, 0.5, corners_col);
	}
	// mask a vector, i.e. retain only those entries where the given mask is true
	void maskVector(VectorXd &masked_vec, const VectorXd &in_vec,
		const VectorXb &mask, int masked_size, int in_size){
		assert(in_vec.size() == mask.size());
		assert(mask.array().count() == masked_size);

		masked_vec.resize(masked_size);
		int mask_id = 0;
		for(int i = 0; i < in_size; i++){
			if(!mask(i)){ masked_vec(mask_id++) = in_vec(i); }
		}
	}
	// returning version
	VectorXd maskVector(const VectorXd &in_vec,
		const VectorXb &mask, int masked_size, int in_size){
		assert(in_vec.size() == mask.size());
		assert(mask.array().count() == masked_size);

		VectorXd masked_vec(masked_size);
		maskVector(masked_vec, in_vec, mask, masked_size, in_size);
		return masked_vec;
	}
	//! specialization with loop unrolling for copying a warp matrix from OpenCV to Eigen
	template<>
	void copyCVToEigen<double, Matrix3d>(Matrix3d &eig_mat, const cv::Mat &cv_mat){
		assert(eig_mat.rows() == cv_mat.rows && eig_mat.cols() == cv_mat.cols);
		eig_mat(0, 0) = cv_mat.at<double>(0, 0);
		eig_mat(0, 1) = cv_mat.at<double>(0, 1);
		eig_mat(0, 2) = cv_mat.at<double>(0, 2);

		eig_mat(1, 0) = cv_mat.at<double>(1, 0);
		eig_mat(1, 1) = cv_mat.at<double>(1, 1);
		eig_mat(1, 2) = cv_mat.at<double>(1, 2);

		eig_mat(2, 0) = cv_mat.at<double>(2, 0);
		eig_mat(2, 1) = cv_mat.at<double>(2, 1);
		eig_mat(2, 2) = cv_mat.at<double>(2, 2);
	}
	//! specialization for copying corners
	template<>
	void copyCVToEigen<double, CornersT>(CornersT &corners_eig, const cv::Mat &corners_cv){
		assert(corners_eig.rows() == corners_cv.rows && corners_eig.cols() == corners_cv.cols);
		for(int i = 0; i < 4; i++){
			corners_eig(0, i) = corners_cv.at<double>(0, i);
			corners_eig(1, i) = corners_cv.at<double>(1, i);
		}
	}

	//! specialization for copying corners
	template<>
	void copyEigenToCV<double, CornersT>(cv::Mat &cv_mat,
		const CornersT &eig_mat){
		assert(cv_mat.rows == 2 && cv_mat.cols == 4);
		cv_mat.at<double>(0, 0) = eig_mat(0, 0);
		cv_mat.at<double>(1, 0) = eig_mat(1, 0);
		cv_mat.at<double>(0, 1) = eig_mat(0, 1);
		cv_mat.at<double>(1, 1) = eig_mat(1, 1);
		cv_mat.at<double>(0, 2) = eig_mat(0, 2);
		cv_mat.at<double>(1, 2) = eig_mat(1, 2);
		cv_mat.at<double>(0, 3) = eig_mat(0, 3);
		cv_mat.at<double>(1, 3) = eig_mat(1, 3);
	}
	double writeTimesToFile(vector<double> &proc_times,
		vector<char*> &proc_labels, char *time_fname, int iter_id){
		MatrixXd iter_times(proc_times.size(), 2);
		for(unsigned int proc_id = 0; proc_id < proc_times.size(); proc_id++){
			iter_times(proc_id, 0) = proc_times[proc_id];
		}
		double total_iter_time = iter_times.col(0).sum();
		iter_times.col(1) = (iter_times.col(0) / total_iter_time) * 100;
		utils::printScalarToFile(iter_id, "iteration", time_fname, "%6d", "a");
		//char **row_label_ptr = &proc_labels[0];
		utils::printMatrixToFile(iter_times, "iter_times", time_fname, "%15.9f", "a",
			"\t", "\n", &proc_labels[0]);
		utils::printScalarToFile(total_iter_time, "total_iter_time", time_fname, "%15.9f", "a");
		return total_iter_time;
	}
	template<typename ValT>
	cv::Rect_<ValT>  getBestFitRectangle(const cv::Mat &corners,
		int _img_width, int _img_height, int border_size){
		double center_x = (corners.at<double>(0, 0) + corners.at<double>(0, 1) +
			corners.at<double>(0, 2) + corners.at<double>(0, 3)) / 4;
		double center_y = (corners.at<double>(1, 0) + corners.at<double>(1, 1) +
			corners.at<double>(1, 2) + corners.at<double>(1, 3)) / 4;

		double mean_half_width = (abs(corners.at<double>(0, 0) - center_x) + abs(corners.at<double>(0, 1) - center_x)
			+ abs(corners.at<double>(0, 2) - center_x) + abs(corners.at<double>(0, 3) - center_x)) / 4.0;
		double mean_half_height = (abs(corners.at<double>(1, 0) - center_y) + abs(corners.at<double>(1, 1) - center_y)
			+ abs(corners.at<double>(1, 2) - center_y) + abs(corners.at<double>(1, 3) - center_y)) / 4.0;

		cv::Rect_<ValT> best_fit_rect;
		best_fit_rect.x = static_cast<ValT>(center_x - mean_half_width);
		best_fit_rect.y = static_cast<ValT>(center_y - mean_half_height);
		best_fit_rect.width = static_cast<ValT>(2 * mean_half_width);
		best_fit_rect.height = static_cast<ValT>(2 * mean_half_height);

		return _img_width > 0 && _img_height > 0 ? getBoundedRectangle<ValT>(best_fit_rect,
			_img_width, _img_height, border_size) : best_fit_rect;
	}
	template cv::Rect_<int> getBestFitRectangle<int>(const cv::Mat &corners, int _img_width, int _img_height, int border_size);
	template cv::Rect_<double> getBestFitRectangle<double>(const cv::Mat &corners, int _img_width, int _img_height, int border_size);
	template cv::Rect_<float> getBestFitRectangle<float>(const cv::Mat &corners, int _img_width, int _img_height, int border_size);

	template<typename ValT>
	cv::Rect_<ValT> getBoundedRectangle(const cv::Rect_<ValT> &_in_rect,
		int _img_width, int _img_height, int border_size){
		cv::Rect_<ValT> bounded_rect;
		int min_x = border_size, max_x = _img_width - border_size - 1;
		int min_y = border_size, max_y = _img_height - border_size - 1;
		//printScalar(_img_width, "_img_width", "%d");
		//printScalar(_img_height, "_img_height", "%d");
		//printScalar(_in_rect.x, "_in_rect.x", "%d");
		//printScalar(_in_rect.y, "_in_rect.y", "%d");
		//printScalar(_in_rect.height, "_in_rect.height", "%d");
		//printScalar(_in_rect.width, "_in_rect.width", "%d");
		bounded_rect.x = _in_rect.x < min_x ? min_x : _in_rect.x;
		bounded_rect.y = _in_rect.y < min_y ? min_y : _in_rect.y;
		bounded_rect.width = bounded_rect.x + _in_rect.width > max_x ? max_x - bounded_rect.x : _in_rect.width;
		bounded_rect.height = bounded_rect.y + _in_rect.height > max_y ? max_y - bounded_rect.y : _in_rect.height;
		//printScalar(bounded_rect.x, "bounded_rect.x", "%d");
		//printScalar(bounded_rect.y, "bounded_rect.y", "%d");
		//printScalar(bounded_rect.height, "bounded_rect.height", "%d");
		//printScalar(bounded_rect.width, "bounded_rect.width", "%d");
		return bounded_rect;
	}
	template cv::Rect_<int> getBoundedRectangle(const cv::Rect_<int> &_in_rect, int _img_width, int _img_height,
		int border_size);
	template cv::Rect_<double> getBoundedRectangle(const cv::Rect_<double> &_in_rect, int _img_width, int _img_height,
		int border_size);
	template cv::Rect_<float> getBoundedRectangle(const cv::Rect_<float> &_in_rect, int _img_width, int _img_height,
		int border_size);


	void drawRegion(cv::Mat &img, const cv::Mat &vertices, cv::Scalar col,
		int line_thickness, const char *label, double font_size,
		bool show_corner_ids, bool show_label, int line_type){
		std::vector<cv::Point2d> vertices_p2d(vertices.cols);
		for(int vert_id = 0; vert_id < vertices.cols; ++vert_id) {
			vertices_p2d[vert_id] = cv::Point2d(vertices.at<double>(0, vert_id), vertices.at<double>(1, vert_id));
		}
		if(line_type == 0){ line_type = CV_AA; }
		for(int vert_id = 0; vert_id < vertices.cols; ++vert_id) {
			cv::line(img, vertices_p2d[vert_id], vertices_p2d[(vert_id + 1) % vertices.cols],
				col, line_thickness, line_type);
			if(show_corner_ids){
#ifdef _WIN32
				putText(img, utils::to_string(vert_id), vertices_p2d[vert_id],
					cv::FONT_HERSHEY_SIMPLEX, font_size, col);
#else
				putText(img, std::to_string(vert_id), vertices_p2d[vert_id],
					cv::FONT_HERSHEY_SIMPLEX, font_size, col);
#endif
			}
		}
		if(label && show_label){
			putText(img, label, vertices_p2d[0], cv::FONT_HERSHEY_SIMPLEX, font_size, col);
		}
	}

#ifndef DISABLE_VISP
	void drawRegion(vpImage<vpRGBa> &img, const cv::Mat &vertices, vpColor col,
		int line_thickness, const char *label, double font_size,
		bool show_corner_ids, bool show_label, int line_type){
		std::vector<cv::Point2d> vertices_p2d(vertices.cols);
		for(int vert_id = 0; vert_id < vertices.cols; ++vert_id) {
			vertices_p2d[vert_id] = cv::Point2d(vertices.at<double>(0, vert_id), vertices.at<double>(1, vert_id));
		}
		if(line_type == 0){ line_type = CV_AA; }
		for(int vert_id = 0; vert_id < vertices.cols; ++vert_id) {
			vpImagePoint pt1(vertices_p2d[vert_id].y, vertices_p2d[vert_id].x);
			vpImagePoint pt2(vertices_p2d[(vert_id + 1) % vertices.cols].y, vertices_p2d[(vert_id + 1) % vertices.cols].x);

			vpDisplay::displayLine(img, pt1, pt2, col, line_thickness);			
			if(show_corner_ids){
				//vpDisplay::setFont(img, "FONT_HERSHEY_SIMPLEX");
				
#ifdef _WIN32
				vpDisplay::displayText(img, pt1, utils::to_string(vert_id), col);
#else
				vpDisplay::displayText(img, pt1, std::to_string(vert_id), col);
#endif
			}
		}
		if(label && show_label){
			//vpDisplay::setFont(img, "FONT_HERSHEY_SIMPLEX");
			vpDisplay::displayText(img, vpImagePoint(vertices_p2d[0].y, vertices_p2d[0].x),
				label, col);
		}
		vpDisplay::flush(img);
	}
#endif

	void drawGrid(cv::Mat &img, const PtsT &grid_pts, int res_x, int res_y, cv::Scalar col, int thickness){
		//draw vertical lines
		for(int x_id = 0; x_id < res_x; ++x_id){
			for(int y_id = 0; y_id < res_y - 1; ++y_id){
				int pt_id1 = (y_id * res_x + x_id);
				int pt_id2 = ((y_id + 1) * res_x + x_id);
				cv::Point p1(int(grid_pts(0, pt_id1)), int(grid_pts(1, pt_id1)));
				cv::Point p2(int(grid_pts(0, pt_id2)), int(grid_pts(1, pt_id2)));
				cv::line(img, p1, p2, col, thickness);
			}
		}
		//draw horizontal lines
		for(int y_id = 0; y_id < res_y; ++y_id){
			for(int x_id = 0; x_id < res_x - 1; ++x_id){
				int pt_id1 = (y_id * res_x + x_id);
				int pt_id2 = (y_id  * res_x + x_id + 1);
				cv::Point p1(int(grid_pts(0, pt_id1)), int(grid_pts(1, pt_id1)));
				cv::Point p2(int(grid_pts(0, pt_id2)), int(grid_pts(1, pt_id2)));
				cv::line(img, p1, p2, col, thickness);
			}
		}
	}
	template<typename ImgValT, typename PatchValT>
	void drawPatch(cv::Mat &img, const cv::Mat &patch, int n_channels,
		int start_x, int start_y){
		if((patch.channels() > 1) && (patch.channels() != img.channels())){
			throw InvalidArgument(
				cv::format("No. of channels in the image: %d does not match that in the patch: %d",
				img.channels(), patch.channels()));
		}

		for(int row_id = 0; row_id < patch.rows - 1; ++row_id){
			for(int col_id = 0; col_id < patch.cols; ++col_id){
				switch(n_channels){
				case 1:
					img.at<ImgValT>(start_y + row_id, start_x + col_id) =
						patch.at<PatchValT>(row_id, col_id);
					break;
				case 3:
					img.at< cv::Vec<ImgValT, 3> >(start_y + row_id, start_x + col_id) =
						patch.at< cv::Vec<PatchValT, 3> >(row_id, col_id);
					break;
				default:
					throw FunctonNotImplemented(
						cv_format("drawPatch :: %d channel images are not supported", n_channels));
				}
			}
		}
	}
	template void drawPatch<uchar, uchar>(cv::Mat &img, const cv::Mat &patch, int n_channels,
		int start_x, int start_y);
	template<typename ScalarT>
	void drawPts(cv::Mat &img, const cv::Mat &pts, cv::Scalar col, int radius, int thickness) {
		for(int pt_id = 0; pt_id < pts.rows; ++pt_id) {
			cv::Point pt(int(pts.at<ScalarT>(pt_id, 0)), int(pts.at<ScalarT>(pt_id, 1)));
			cv::circle(img, pt, radius, col, thickness);
		}
	}
	template void drawPts<float>(cv::Mat &img, const cv::Mat &pts, cv::Scalar col, int radius, int thickness);
	template void drawPts<double>(cv::Mat &img, const cv::Mat &pts, cv::Scalar col, int radius, int thickness);

	void writeCorners(FILE *out_fid, const cv::Mat &corners,
		int frame_id, bool write_header){
		if(write_header){
			fprintf(out_fid, "frame ulx uly urx ury lrx lry llx lly\n");
		}
		fprintf(out_fid, "frame%05d.jpg %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f %15.6f\n", frame_id + 1,
			corners.at<double>(0, 0), corners.at<double>(1, 0),
			corners.at<double>(0, 1), corners.at<double>(1, 1),
			corners.at<double>(0, 2), corners.at<double>(1, 2),
			corners.at<double>(0, 3), corners.at<double>(1, 3));
	}

	const char* toString(TrackErrT _er_type){
		switch(_er_type){
		case TrackErrT::MCD:
			return "Mean Corner Distance";
		case TrackErrT::CL:
			return "Center Location";
		case TrackErrT::Jaccard:
			return "Jaccard";
		default:
			throw InvalidArgument(
				cv::format("Invalid tracking error type provided: %d\n",
				_er_type));
		}
	}
	template<>
	double getTrackingError<TrackErrT::MCD>(const cv::Mat &gt_corners, const cv::Mat &tracker_corners){
		// compute the mean corner distance between the tracking result and the ground truth
		double tracking_err = 0;
		for(int corner_id = 0; corner_id < 4; ++corner_id){
			double x_diff = gt_corners.at<double>(0, corner_id) - tracker_corners.at<double>(0, corner_id);
			double y_diff = gt_corners.at<double>(1, corner_id) - tracker_corners.at<double>(1, corner_id);
			//printf("x_diff: %f\n", x_diff);
			//printf("y_diff: %f\n", y_diff);
			tracking_err += sqrt((x_diff*x_diff) + (y_diff*y_diff));
		}
		tracking_err /= 4.0;
		//printMatrix<double>(gt_corners, "gt_corners");
		//printMatrix<double>(tracker_corners, "tracker_corners");
		//printScalar(tracking_err, "tracking_err");
		return tracking_err;
	}
	template<>
	double getTrackingError<TrackErrT::CL>(const cv::Mat &gt_corners, const cv::Mat &tracker_corners){
		// compute the center location error between the tracking result and the ground truth
		cv::Mat corner_diff = gt_corners - tracker_corners;
		double mean_x_diff = (corner_diff.at<double>(0, 0) + corner_diff.at<double>(0, 1) +
			corner_diff.at<double>(0, 2) + corner_diff.at<double>(0, 3)) / 4.0;
		double mean_y_diff = (corner_diff.at<double>(1, 0) + corner_diff.at<double>(1, 1) +
			corner_diff.at<double>(1, 2) + corner_diff.at<double>(1, 3)) / 4.0;
		double tracking_err = sqrt((mean_x_diff*mean_x_diff) + (mean_y_diff*mean_y_diff));
		return tracking_err;
	}
	template<>
	double getTrackingError<TrackErrT::Jaccard>(const cv::Mat &gt_corners, const cv::Mat &tracker_corners){
		// compute the Jaccard index error between the tracking result and the ground truth		
		double gt_min_x, gt_max_x, gt_min_y, gt_max_y;
		cv::minMaxLoc(gt_corners.row(0), &gt_min_x, &gt_max_x);
		cv::minMaxLoc(gt_corners.row(1), &gt_min_y, &gt_max_y);
		double tracker_min_x, tracker_max_x, tracker_min_y, tracker_max_y;
		cv::minMaxLoc(tracker_corners.row(0), &tracker_min_x, &tracker_max_x);
		cv::minMaxLoc(tracker_corners.row(1), &tracker_min_y, &tracker_max_y);

		int min_x = static_cast<int>(gt_min_x < tracker_min_x ? gt_min_x : tracker_min_x);
		int min_y = static_cast<int>(gt_min_y < tracker_min_y ? gt_min_y : tracker_min_y);

		int max_x = static_cast<int>(gt_max_x > tracker_max_x ? gt_max_x : tracker_max_x);
		int max_y = static_cast<int>(gt_max_y > tracker_max_y ? gt_max_y : tracker_max_y);

		if(max_x < min_x || max_y < min_y){
			throw invalid_argument("getJaccardError::Invalid GT and/or tracker corners provided\n");
		}

		//! minimum and maximum allowed values for the corner coordinates to avoid numerical issues due to
		//! unreasonable low/high values produced by failing trackers
		const int min_thresh = 0, max_thresh = 2000;
		if(min_x < min_thresh){ min_x = min_thresh; }
		if(min_y < min_thresh){ min_y = min_thresh; }

		if(max_x > max_thresh){ max_x = max_thresh; }
		if(max_y > max_thresh){ max_y = max_thresh; }

		//! a border is added around the regions for better numerical stability
		const int border_size = 100;

		int img_height = max_y - min_y + 2 * border_size + 1;
		int img_width = max_x - min_x + 2 * border_size + 1;

		cv::Point2i gt_corners_int[4], tracker_corners_int[4];
		Corners(gt_corners, border_size - min_x, border_size - min_y).points(gt_corners_int);
		Corners(tracker_corners, border_size - min_x, border_size - min_y).points(tracker_corners_int);

		cv::Mat gt_img(img_height, img_width, CV_8UC1, cv::Scalar(0));
		cv::Mat tracker_img(img_height, img_width, CV_8UC1, cv::Scalar(0));
		fillConvexPoly(gt_img, gt_corners_int, 4, cv::Scalar(255), CV_AA);
		fillConvexPoly(tracker_img, tracker_corners_int, 4, cv::Scalar(255), CV_AA);

		cv::Mat intersection_img(img_height, img_width, CV_8UC1, cv::Scalar(0));
		cv::Mat union_img(img_height, img_width, CV_8UC1, cv::Scalar(0));
		cv::bitwise_and(gt_img, tracker_img, intersection_img);
		cv::bitwise_or(gt_img, tracker_img, union_img);

		double n_intersection = cv::sum(intersection_img)[0];
		double n_union = cv::sum(union_img)[0];
		double tracking_err = 1.0 - n_intersection / n_union;

		//cv::Mat intersection_img_resized, union_img_resized;
		//cv::resize(intersection_img, intersection_img_resized,
		//	cv::Size(img_width * 3, img_height * 3));
		//cv::resize(union_img, union_img_resized,
		//	cv::Size(img_width * 3, img_height * 3));

		//cv::imshow("gt_img fast", gt_img);
		//cv::imshow("tracker_img fast", tracker_img);
		//cv::imshow("intersection_img fast", intersection_img);
		//cv::imshow("union_img fast", union_img);
		//printf("new :: n_intersection: %f\t n_union: %f\n", n_intersection, n_union);

		return tracking_err;
	}
	double getJaccardError(const cv::Mat &gt_corners, const cv::Mat &tracker_corners,
		int img_width, int img_height){
		// compute the Jaccard index error between the tracking result and the ground truth
		cv::Point2i gt_corners_int[4], tracker_corners_int[4];
		Corners(gt_corners).points(gt_corners_int);
		Corners(tracker_corners).points(tracker_corners_int);
		cv::Mat gt_img(img_height, img_width, CV_8UC1, cv::Scalar(0));
		cv::Mat tracker_img(img_height, img_width, CV_8UC1, cv::Scalar(0));
		fillConvexPoly(gt_img, gt_corners_int, 4, cv::Scalar(255), CV_AA);
		fillConvexPoly(tracker_img, tracker_corners_int, 4, cv::Scalar(255), CV_AA);
		cv::Mat intersection_img(img_height, img_width, CV_8UC1, cv::Scalar(0));
		cv::Mat union_img(img_height, img_width, CV_8UC1, cv::Scalar(0));
		cv::bitwise_and(gt_img, tracker_img, intersection_img);
		cv::bitwise_or(gt_img, tracker_img, union_img);


		double n_intersection = cv::sum(intersection_img)[0];
		double n_union = cv::sum(union_img)[0];
		double tracking_err = 1.0 - n_intersection / n_union;

		//cv::imshow("gt_img", gt_img);
		//cv::imshow("tracker_img", tracker_img);
		//cv::imshow("union_img", union_img);
		//cv::imshow("intersection_img", intersection_img);
		//printf("old :: n_intersection: %f\t n_union: %f\n", n_intersection, n_union);

		return tracking_err;
	}

	double getTrackingError(TrackErrT tracking_err_type, const cv::Mat &gt_corners,
		const cv::Mat &tracker_corners, FILE *out_fid, int frame_id,
		int img_width, int img_height){
		if(out_fid){
			//! compute and write all errors to the provided file
			double mcd_err = getTrackingError<TrackErrT::MCD>(gt_corners, tracker_corners);
			double cl_err = getTrackingError<TrackErrT::CL>(gt_corners, tracker_corners);
			double jacc_err = getTrackingError<TrackErrT::Jaccard>(gt_corners, tracker_corners);

			fprintf(out_fid, "frame%05d.jpg\t %15.6f\t %15.6f\t %15.6f", frame_id + 1,
				mcd_err, cl_err, jacc_err);

			if(img_width > 0 && img_height > 0){
				double jacc_err_2 = getJaccardError(gt_corners, tracker_corners,
					img_width, img_height);
				fprintf(out_fid, "\t %15.6f", jacc_err_2);
				if(jacc_err != 0){
					double frac_jacc_diff = fabs(jacc_err - jacc_err_2) / jacc_err;
					printf("frac_jacc_diff: %f\n", frac_jacc_diff);
				}
			}

			fprintf(out_fid, "\n");
			//! check the type of error needed
			switch(tracking_err_type){
			case TrackErrT::MCD:
				return mcd_err;
			case TrackErrT::CL:
				return cl_err;
			case TrackErrT::Jaccard:
				return jacc_err;
			default:
				throw invalid_argument(
					cv::format("getTrackingError::Invalid tracking error type provided: %d\n",
					tracking_err_type));
			}
		}
		//! check the type of error needed
		switch(tracking_err_type){
		case TrackErrT::MCD:
			return getTrackingError<TrackErrT::MCD>(gt_corners, tracker_corners);
		case TrackErrT::CL:
			return getTrackingError<TrackErrT::CL>(gt_corners, tracker_corners);
		case TrackErrT::Jaccard:
			return getTrackingError<TrackErrT::Jaccard>(gt_corners, tracker_corners);
		default:
			throw invalid_argument(
				cv::format("getTrackingError::Invalid tracking error type provided: %d\n",
				tracking_err_type));
		}
	}

	cv::Mat readTrackerLocation(const std::string &file_path){
		std::ifstream fin(file_path);
		float ulx, uly, urx, ury, lrx, lry, llx, lly;
		fin >> ulx >> uly >> urx >> ury >> lrx >> lry >> llx >> lly;
		cv::Mat tracker_location(2, 4, CV_64FC1);
		tracker_location.at<double>(0, 0) = ulx;
		tracker_location.at<double>(0, 1) = urx;
		tracker_location.at<double>(0, 2) = lrx;
		tracker_location.at<double>(0, 3) = llx;
		tracker_location.at<double>(1, 0) = uly;
		tracker_location.at<double>(1, 1) = ury;
		tracker_location.at<double>(1, 2) = lry;
		tracker_location.at<double>(1, 3) = lly;
		return tracker_location;
	}
	cv::Mat getFrameCorners(const cv::Mat &img, int borner_size){
		cv::Mat uav_img_corners(2, 4, CV_64FC1);
		uav_img_corners.at<double>(0, 0) = borner_size;
		uav_img_corners.at<double>(0, 1) = img.cols - borner_size - 1;
		uav_img_corners.at<double>(0, 2) = img.cols - borner_size - 1;
		uav_img_corners.at<double>(0, 3) = borner_size;
		uav_img_corners.at<double>(1, 0) = borner_size;
		uav_img_corners.at<double>(1, 1) = borner_size;
		uav_img_corners.at<double>(1, 2) = img.rows - borner_size - 1;
		uav_img_corners.at<double>(1, 3) = img.rows - borner_size - 1;
		return uav_img_corners;
	}
	mtf::PtsT getFramePts(const cv::Mat &img, int borner_size){
		int min_x = borner_size, min_y = borner_size;
		int max_x = img.cols - borner_size - 1, max_y = img.rows - borner_size - 1;
		int n_pts = (max_x - min_x + 1)*(max_y - min_y + 1);
		mtf::PtsT pts;
		pts.resize(Eigen::NoChange, n_pts);

		int pt_id = 0;
		for(int x = min_x; x <= max_x; x++){
			for(int y = min_y; y <= max_y; y++){
				pts(0, pt_id) = x;
				pts(1, pt_id) = y;
				++pt_id;
			}
		}
		return pts;
	}
	cv::Point2d getCentroid(const cv::Mat &corners){
		cv::Mat centroid;
		cv::reduce(corners, centroid, 1, CV_REDUCE_AVG);
		//printMatrix<double>(corners, "getCentroid :: corners");
		//printMatrix<double>(centroid, "getCentroid :: centroid");
		return cv::Point2d(centroid.at<double>(0, 0), centroid.at<double>(1, 0));
	}
	std::vector<int>  rearrangeIntoRegion(const cv::Mat &region_corners){
		cv::Mat region_rect = utils::Corners(utils::getBestFitRectangle<double>(region_corners)).mat();
		std::vector<int> rearrange_idx(region_corners.cols);
		for(int corner_id = 0; corner_id < region_corners.cols; ++corner_id){
			double min_dist = std::numeric_limits<double>::infinity();
			int min_idx = 0;
			double x1 = region_corners.at<double>(0, corner_id);
			double y1 = region_corners.at<double>(1, corner_id);
			for(int rect_id = 0; rect_id < region_rect.cols; ++rect_id){
				double x2 = region_rect.at<double>(0, rect_id);
				double y2 = region_rect.at<double>(1, rect_id);
				double dx = x1 - x2, dy = y1 - y2;
				double dist = dx*dx + dy*dy;
				if(dist < min_dist){
					min_dist = dist;
					min_idx = rect_id;
				}
			}
			rearrange_idx[corner_id] = min_idx;
		}
		return rearrange_idx;
	}
	cv::Mat concatenate(const cv::Mat img_list[], int n_images, int axis){
		cv::Mat out_img;
		if(axis == 1){
			cv::hconcat(img_list, n_images, out_img);
		} else{
			cv::vconcat(img_list, n_images, out_img);
		}
		return out_img;
	}
	cv::Mat stackImages(const std::vector<cv::Mat> &img_list, int stack_order){
		int n_images = img_list.size();
		cv::Size img_size = img_list[0].size();
		int img_type = img_list[0].type();
		int grid_size = int(ceil(sqrt(n_images)));
		cv::Mat stacked_img;
		bool list_ended = false;
		int inner_axis = 1 - stack_order;
		for(int row_id = 0; row_id < grid_size; ++row_id){
			int start_id = grid_size * row_id;
			cv::Mat curr_row;
			for(int col_id = 0; col_id < grid_size; ++col_id){
				int img_id = start_id + col_id;
				cv::Mat curr_img;
				if(img_id >= n_images){
					curr_img.create(img_size, img_type);
					list_ended = true;
				} else{
					curr_img = img_list[img_id];
				}
				if(img_id == n_images - 1){
					list_ended = true;
				}
				if(curr_row.empty()){
					curr_row = curr_img;
				} else{
					const cv::Mat curr_list[] = { curr_row, curr_img };
					curr_row = concatenate(curr_list, 2, inner_axis);
				}
			}
			if(stacked_img.empty()){
				stacked_img = curr_row;
			} else{
				const cv::Mat curr_list[] = { stacked_img, curr_row };
				stacked_img = concatenate(curr_list, 2, stack_order);
			}
			if(list_ended){
				break;
			}
		}
		return stacked_img;
	}
	std::string getDateTime() {
		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];

		time(&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(buffer, sizeof(buffer), "%y%m%d_%H%M%S", timeinfo);
		return std::string(buffer);
	}
}

_MTF_END_NAMESPACE
