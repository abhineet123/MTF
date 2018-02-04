#ifndef MTF_OBJ_UTILS_H
#define MTF_OBJ_UTILS_H

#include<sstream>
#include<memory>
#include <map>

#include "opencv2/core/core.hpp"
#include "mtf/Utilities/inputUtils.h"
#ifndef DISABLE_VISP
#include <visp3/core/vpColor.h>
#endif

using namespace std;

_MTF_BEGIN_NAMESPACE
namespace utils{
	//extern const std::map<std::string, cv::Scalar> col_rgb;
	/**
	simple structure to store an object read from ground truth or selected by the user;
	requires that the object location be representable by a quadrilateral
	*/
	struct ObjStruct {
		cv::Point2d min_point;
		cv::Point2d max_point;

		double size_x;
		double size_y;
		double pos_x;
		double pos_y;

		cv::Mat corners;

		ObjStruct();
		void updateCornerMat();
		void updateCornerPoints();
		void operator*=(double resize_factor);
	};

	typedef std::shared_ptr<ObjStruct> ObjStructPtr;

	/* callback function for mouse clicks */
	inline void getClickedPoint(int mouse_event, int x, int y, int flags, void* param);

	/**
	utility functions to obtain initial object location for tracking -
	either read from a ground truth file or selected interactively by the user
	*/
	class ObjUtils {
	public:
		ObjUtils(double _resize_factor = 1.0);
		~ObjUtils();
		const cv::Scalar &getObjCol(int col_id){
			return obj_cols[col_id % no_of_cols];
		}
		/**
		allows the user to select a rectangle by clicking on its opposite corners
		*/
		bool addRectObject(InputBase *input, string selection_window,
			int line_thickness = 2, int patch_size = 0);
		/**
		allows the user to select a quadrilateral by clicking on its 4 corners
		*/
		bool addQuadObject(InputBase *input, string selection_window,
			int line_thickness = 2);
		//! overloaded variant for non-live input feed
		bool selectObjects(const cv::Mat &img, int no_of_objs,
			int patch_size = 0, int line_thickness = 1, int write_objs = 0, bool sel_quad_obj = false,
			const char* filename = "selected_objects.txt");
		bool selectObjects(InputBase *input, int no_of_objs, int patch_size = 0,
			int line_thickness = 1, int write_objs = 0, bool sel_quad_obj = false,
			const char* filename = "selected_objects.txt");
#ifndef DISABLE_VISP
		bool addRectObjectVP(InputBase *input, string selection_window,
			int line_thickness = 2, int patch_size = 0);
		bool addQuadObjectVP(InputBase *input, string selection_window,
			int line_thickness = 2);
		//! overloaded variant for non-live input feed
		bool selectObjectsVP(const cv::Mat &img, int no_of_objs,
			int patch_size = 0, int line_thickness = 1, int write_objs = 0, bool sel_quad_obj = false,
			const char* filename = "selected_objects.txt");
		bool selectObjectsVP(InputBase *input, int no_of_objs, int patch_size = 0,
			int line_thickness = 1, int write_objs = 0, bool sel_quad_obj = false,
			const char* filename = "selected_objects.txt");
#endif
		void writeObjectsToFile(int no_of_objs,
			const char* filename = "sel_objs/selected_objects.txt");
		bool readObjectFromGT(string source_name, string source_path, int n_frames,
			int _init_frame_id = 0, bool use_opt_gt = false, string opt_gt_ssm = "2",
			bool _use_reinit_gt = false, bool _invert_seq = false, int debug_mode = 0);
		bool getObjStatus() const{ return !init_objects.empty(); }
		const ObjStruct &getObj(int obj_id = 0) const{
			return init_objects.size() == 1 ? init_objects[0] : init_objects[obj_id];
		}
		const cv::Mat &getGT(int frame_id, int _reinit_frame_id = -1);
		const cv::Mat &getReinitGT(int frame_id, int _reinit_frame_id = -1);
		const vector<cv::Mat> &getGT(){ return ground_truth; }
		int getGTSize(){
			return use_reinit_gt ? ground_truth.size() + reinit_frame_id : ground_truth.size();
		}
		bool readGT(string source_name, string source_path,
			int n_frames = 1, int init_frame_id = 0, int debug_mode = 0,
			bool use_opt_gt = false, string opt_gt_ssm = "2");
		bool readReinitGT(string source_name, string source_path,
			int _reinit_frame_id, int _n_frames, bool use_opt_gt = false,
			string opt_gt_ssm = "2");
		bool readObjectsFromFile(int no_of_objs,
			const char* filename = "sel_objs/selected_objects.txt", int debug_mode = 0);
		cv::Point getMeanPoint(cv::Point *pt_array, int no_of_points);
		void cornersToPoint2D(cv::Point2d(&cv_corners)[4], const cv::Mat &cv_corners_mat);
		
	private:
		vector<ObjStruct> init_objects;
		vector<cv::Mat> ground_truth;
		vector<cv::Mat> reinit_ground_truth;
		int init_frame_id, reinit_frame_id;
		int reinit_n_frames;
		bool use_reinit_gt;
		string reinit_gt_filename;
		vector<cv::Scalar> obj_cols;
		int no_of_cols;
#ifndef DISABLE_VISP
		vector<vpColor> obj_cols_vp;
		int no_of_cols_vp;
#endif

		double resize_factor;
		bool invert_seq;
		//! read reinit GT for a specific frame
		void readReinitGT(int _reinit_frame_id);
		const vector<cv::Mat> &getReinitGT(){ return reinit_ground_truth; }
	};
}
_MTF_END_NAMESPACE
#endif
