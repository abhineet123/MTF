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

	std::map<std::string, cv::Scalar> col_rgb = {
		{ "snow", cv::Scalar(250, 250, 255) },
		{ "snow_2", cv::Scalar(233, 233, 238) },
		{ "snow_3", cv::Scalar(201, 201, 205) },
		{ "snow_4", cv::Scalar(137, 137, 139) },
		{ "ghost_white", cv::Scalar(255, 248, 248) },
		{ "white_smoke", cv::Scalar(245, 245, 245) },
		{ "gainsboro", cv::Scalar(220, 220, 220) },
		{ "floral_white", cv::Scalar(240, 250, 255) },
		{ "old_lace", cv::Scalar(230, 245, 253) },
		{ "linen", cv::Scalar(230, 240, 240) },
		{ "antique_white", cv::Scalar(215, 235, 250) },
		{ "antique_white_2", cv::Scalar(204, 223, 238) },
		{ "antique_white_3", cv::Scalar(176, 192, 205) },
		{ "antique_white_4", cv::Scalar(120, 131, 139) },
		{ "papaya_whip", cv::Scalar(213, 239, 255) },
		{ "blanched_almond", cv::Scalar(205, 235, 255) },
		{ "bisque", cv::Scalar(196, 228, 255) },
		{ "bisque_2", cv::Scalar(183, 213, 238) },
		{ "bisque_3", cv::Scalar(158, 183, 205) },
		{ "bisque_4", cv::Scalar(107, 125, 139) },
		{ "peach_puff", cv::Scalar(185, 218, 255) },
		{ "peach_puff_2", cv::Scalar(173, 203, 238) },
		{ "peach_puff_3", cv::Scalar(149, 175, 205) },
		{ "peach_puff_4", cv::Scalar(101, 119, 139) },
		{ "navajo_white", cv::Scalar(173, 222, 255) },
		{ "moccasin", cv::Scalar(181, 228, 255) },
		{ "cornsilk", cv::Scalar(220, 248, 255) },
		{ "cornsilk_2", cv::Scalar(205, 232, 238) },
		{ "cornsilk_3", cv::Scalar(177, 200, 205) },
		{ "cornsilk_4", cv::Scalar(120, 136, 139) },
		{ "ivory", cv::Scalar(240, 255, 255) },
		{ "ivory_2", cv::Scalar(224, 238, 238) },
		{ "ivory_3", cv::Scalar(193, 205, 205) },
		{ "ivory_4", cv::Scalar(131, 139, 139) },
		{ "lemon_chiffon", cv::Scalar(205, 250, 255) },
		{ "seashell", cv::Scalar(238, 245, 255) },
		{ "seashell_2", cv::Scalar(222, 229, 238) },
		{ "seashell_3", cv::Scalar(191, 197, 205) },
		{ "seashell_4", cv::Scalar(130, 134, 139) },
		{ "honeydew", cv::Scalar(240, 255, 240) },
		{ "honeydew_2", cv::Scalar(224, 238, 244) },
		{ "honeydew_3", cv::Scalar(193, 205, 193) },
		{ "honeydew_4", cv::Scalar(131, 139, 131) },
		{ "mint_cream", cv::Scalar(250, 255, 245) },
		{ "azure", cv::Scalar(255, 255, 240) },
		{ "alice_blue", cv::Scalar(255, 248, 240) },
		{ "lavender", cv::Scalar(250, 230, 230) },
		{ "lavender_blush", cv::Scalar(245, 240, 255) },
		{ "misty_rose", cv::Scalar(225, 228, 255) },
		{ "white", cv::Scalar(255, 255, 255) },
		{ "black", cv::Scalar(0, 0, 0) },
		{ "dark_slate_gray", cv::Scalar(79, 79, 49) },
		{ "dim_gray", cv::Scalar(105, 105, 105) },
		{ "slate_gray", cv::Scalar(144, 138, 112) },
		{ "light_slate_gray", cv::Scalar(153, 136, 119) },
		{ "gray", cv::Scalar(190, 190, 190) },
		{ "light_gray", cv::Scalar(211, 211, 211) },
		{ "midnight_blue", cv::Scalar(112, 25, 25) },
		{ "navy", cv::Scalar(128, 0, 0) },
		{ "cornflower_blue", cv::Scalar(237, 149, 100) },
		{ "dark_slate_blue", cv::Scalar(139, 61, 72) },
		{ "slate_blue", cv::Scalar(205, 90, 106) },
		{ "medium_slate_blue", cv::Scalar(238, 104, 123) },
		{ "light_slate_blue", cv::Scalar(255, 112, 132) },
		{ "medium_blue", cv::Scalar(205, 0, 0) },
		{ "royal_blue", cv::Scalar(225, 105, 65) },
		{ "blue", cv::Scalar(255, 0, 0) },
		{ "dodger_blue", cv::Scalar(255, 144, 30) },
		{ "deep_sky_blue", cv::Scalar(255, 191, 0) },
		{ "sky_blue", cv::Scalar(250, 206, 135) },
		{ "light_sky_blue", cv::Scalar(250, 206, 135) },
		{ "steel_blue", cv::Scalar(180, 130, 70) },
		{ "light_steel_blue", cv::Scalar(222, 196, 176) },
		{ "light_blue", cv::Scalar(230, 216, 173) },
		{ "powder_blue", cv::Scalar(230, 224, 176) },
		{ "pale_turquoise", cv::Scalar(238, 238, 175) },
		{ "dark_turquoise", cv::Scalar(209, 206, 0) },
		{ "medium_turquoise", cv::Scalar(204, 209, 72) },
		{ "turquoise", cv::Scalar(208, 224, 64) },
		{ "cyan", cv::Scalar(255, 255, 0) },
		{ "light_cyan", cv::Scalar(255, 255, 224) },
		{ "cadet_blue", cv::Scalar(160, 158, 95) },
		{ "medium_aquamarine", cv::Scalar(170, 205, 102) },
		{ "aquamarine", cv::Scalar(212, 255, 127) },
		{ "dark_green", cv::Scalar(0, 100, 0) },
		{ "dark_olive_green", cv::Scalar(47, 107, 85) },
		{ "dark_sea_green", cv::Scalar(143, 188, 143) },
		{ "sea_green", cv::Scalar(87, 139, 46) },
		{ "medium_sea_green", cv::Scalar(113, 179, 60) },
		{ "light_sea_green", cv::Scalar(170, 178, 32) },
		{ "pale_green", cv::Scalar(152, 251, 152) },
		{ "spring_green", cv::Scalar(127, 255, 0) },
		{ "lawn_green", cv::Scalar(0, 252, 124) },
		{ "chartreuse", cv::Scalar(0, 255, 127) },
		{ "medium_spring_green", cv::Scalar(154, 250, 0) },
		{ "green_yellow", cv::Scalar(47, 255, 173) },
		{ "lime_green", cv::Scalar(50, 205, 50) },
		{ "yellow_green", cv::Scalar(50, 205, 154) },
		{ "forest_green", cv::Scalar(34, 139, 34) },
		{ "olive_drab", cv::Scalar(35, 142, 107) },
		{ "dark_khaki", cv::Scalar(107, 183, 189) },
		{ "khaki", cv::Scalar(140, 230, 240) },
		{ "pale_goldenrod", cv::Scalar(170, 232, 238) },
		{ "light_goldenrod_yellow", cv::Scalar(210, 250, 250) },
		{ "light_yellow", cv::Scalar(224, 255, 255) },
		{ "yellow", cv::Scalar(0, 255, 255) },
		{ "gold", cv::Scalar(0, 215, 255) },
		{ "light_goldenrod", cv::Scalar(130, 221, 238) },
		{ "goldenrod", cv::Scalar(32, 165, 218) },
		{ "dark_goldenrod", cv::Scalar(11, 134, 184) },
		{ "rosy_brown", cv::Scalar(143, 143, 188) },
		{ "indian_red", cv::Scalar(92, 92, 205) },
		{ "saddle_brown", cv::Scalar(19, 69, 139) },
		{ "sienna", cv::Scalar(45, 82, 160) },
		{ "peru", cv::Scalar(63, 133, 205) },
		{ "burlywood", cv::Scalar(135, 184, 222) },
		{ "beige", cv::Scalar(220, 245, 245) },
		{ "wheat", cv::Scalar(179, 222, 245) },
		{ "sandy_brown", cv::Scalar(96, 164, 244) },
		{ "tan", cv::Scalar(140, 180, 210) },
		{ "chocolate", cv::Scalar(30, 105, 210) },
		{ "firebrick", cv::Scalar(34, 34, 178) },
		{ "brown", cv::Scalar(42, 42, 165) },
		{ "dark_salmon", cv::Scalar(122, 150, 233) },
		{ "salmon", cv::Scalar(114, 128, 250) },
		{ "light_salmon", cv::Scalar(122, 160, 255) },
		{ "orange", cv::Scalar(0, 165, 255) },
		{ "dark_orange", cv::Scalar(0, 140, 255) },
		{ "coral", cv::Scalar(80, 127, 255) },
		{ "light_coral", cv::Scalar(128, 128, 240) },
		{ "tomato", cv::Scalar(71, 99, 255) },
		{ "orange_red", cv::Scalar(0, 69, 255) },
		{ "red", cv::Scalar(0, 0, 255) },
		{ "hot_pink", cv::Scalar(180, 105, 255) },
		{ "deep_pink", cv::Scalar(147, 20, 255) },
		{ "pink", cv::Scalar(203, 192, 255) },
		{ "light_pink", cv::Scalar(193, 182, 255) },
		{ "pale_violet_red", cv::Scalar(147, 112, 219) },
		{ "maroon", cv::Scalar(96, 48, 176) },
		{ "medium_violet_red", cv::Scalar(133, 21, 199) },
		{ "violet_red", cv::Scalar(144, 32, 208) },
		{ "violet", cv::Scalar(238, 130, 238) },
		{ "plum", cv::Scalar(221, 160, 221) },
		{ "orchid", cv::Scalar(214, 112, 218) },
		{ "medium_orchid", cv::Scalar(211, 85, 186) },
		{ "dark_orchid", cv::Scalar(204, 50, 153) },
		{ "dark_violet", cv::Scalar(211, 0, 148) },
		{ "blue_violet", cv::Scalar(226, 43, 138) },
		{ "purple", cv::Scalar(240, 32, 160) },
		{ "medium_purple", cv::Scalar(219, 112, 147) },
		{ "thistle", cv::Scalar(216, 191, 216) },
		{ "green", cv::Scalar(0, 255, 0) },
		{ "magenta", cv::Scalar(255, 0, 255) }
	};

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
