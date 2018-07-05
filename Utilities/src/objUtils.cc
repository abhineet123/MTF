#include "mtf/Utilities/objUtils.h"
#include "mtf/Utilities/miscUtils.h"
#include "mtf/Utilities/excpUtils.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#ifndef DISABLE_VISP
//#if defined _WIN32
//#define VISP_HAVE_X11
//#endif
#include <visp3/core/vpImagePoint.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>
#endif

#include<iostream>
#include<fstream>
#include <stdio.h>
#include <stdlib.h>

#define colors(type) {\
	{ "snow", type(250, 250, 255) },\
	{ "snow_2", type(233, 233, 238) },\
	{ "snow_3", type(201, 201, 205) },\
	{ "snow_4", type(137, 137, 139) },\
	{ "ghost_white", type(255, 248, 248) },\
	{ "white_smoke", type(245, 245, 245) },\
	{ "gainsboro", type(220, 220, 220) },\
	{ "floral_white", type(240, 250, 255) },\
	{ "old_lace", type(230, 245, 253) },\
	{ "linen", type(230, 240, 240) },\
	{ "antique_white", type(215, 235, 250) },\
	{ "antique_white_2", type(204, 223, 238) },\
	{ "antique_white_3", type(176, 192, 205) },\
	{ "antique_white_4", type(120, 131, 139) },\
	{ "papaya_whip", type(213, 239, 255) },\
	{ "blanched_almond", type(205, 235, 255) },\
	{ "bisque", type(196, 228, 255) },\
	{ "bisque_2", type(183, 213, 238) },\
	{ "bisque_3", type(158, 183, 205) },\
	{ "bisque_4", type(107, 125, 139) },\
	{ "peach_puff", type(185, 218, 255) },\
	{ "peach_puff_2", type(173, 203, 238) },\
	{ "peach_puff_3", type(149, 175, 205) },\
	{ "peach_puff_4", type(101, 119, 139) },\
	{ "navajo_white", type(173, 222, 255) },\
	{ "moccasin", type(181, 228, 255) },\
	{ "cornsilk", type(220, 248, 255) },\
	{ "cornsilk_2", type(205, 232, 238) },\
	{ "cornsilk_3", type(177, 200, 205) },\
	{ "cornsilk_4", type(120, 136, 139) },\
	{ "ivory", type(240, 255, 255) },\
	{ "ivory_2", type(224, 238, 238) },\
	{ "ivory_3", type(193, 205, 205) },\
	{ "ivory_4", type(131, 139, 139) },\
	{ "lemon_chiffon", type(205, 250, 255) },\
	{ "seashell", type(238, 245, 255) },\
	{ "seashell_2", type(222, 229, 238) },\
	{ "seashell_3", type(191, 197, 205) },\
	{ "seashell_4", type(130, 134, 139) },\
	{ "honeydew", type(240, 255, 240) },\
	{ "honeydew_2", type(224, 238, 244) },\
	{ "honeydew_3", type(193, 205, 193) },\
	{ "honeydew_4", type(131, 139, 131) },\
	{ "mint_cream", type(250, 255, 245) },\
	{ "azure", type(255, 255, 240) },\
	{ "alice_blue", type(255, 248, 240) },\
	{ "lavender", type(250, 230, 230) },\
	{ "lavender_blush", type(245, 240, 255) },\
	{ "misty_rose", type(225, 228, 255) },\
	{ "white", type(255, 255, 255) },\
	{ "black", type(0, 0, 0) },\
	{ "dark_slate_gray", type(79, 79, 49) },\
	{ "dim_gray", type(105, 105, 105) },\
	{ "slate_gray", type(144, 138, 112) },\
	{ "light_slate_gray", type(153, 136, 119) },\
	{ "gray", type(190, 190, 190) },\
	{ "light_gray", type(211, 211, 211) },\
	{ "midnight_blue", type(112, 25, 25) },\
	{ "navy", type(128, 0, 0) },\
	{ "cornflower_blue", type(237, 149, 100) },\
	{ "dark_slate_blue", type(139, 61, 72) },\
	{ "slate_blue", type(205, 90, 106) },\
	{ "medium_slate_blue", type(238, 104, 123) },\
	{ "light_slate_blue", type(255, 112, 132) },\
	{ "medium_blue", type(205, 0, 0) },\
	{ "royal_blue", type(225, 105, 65) },\
	{ "blue", type(255, 0, 0) },\
	{ "dodger_blue", type(255, 144, 30) },\
	{ "deep_sky_blue", type(255, 191, 0) },\
	{ "sky_blue", type(250, 206, 135) },\
	{ "light_sky_blue", type(250, 206, 135) },\
	{ "steel_blue", type(180, 130, 70) },\
	{ "light_steel_blue", type(222, 196, 176) },\
	{ "light_blue", type(230, 216, 173) },\
	{ "powder_blue", type(230, 224, 176) },\
	{ "pale_turquoise", type(238, 238, 175) },\
	{ "dark_turquoise", type(209, 206, 0) },\
	{ "medium_turquoise", type(204, 209, 72) },\
	{ "turquoise", type(208, 224, 64) },\
	{ "cyan", type(255, 255, 0) },\
	{ "light_cyan", type(255, 255, 224) },\
	{ "cadet_blue", type(160, 158, 95) },\
	{ "medium_aquamarine", type(170, 205, 102) },\
	{ "aquamarine", type(212, 255, 127) },\
	{ "dark_green", type(0, 100, 0) },\
	{ "dark_olive_green", type(47, 107, 85) },\
	{ "dark_sea_green", type(143, 188, 143) },\
	{ "sea_green", type(87, 139, 46) },\
	{ "medium_sea_green", type(113, 179, 60) },\
	{ "light_sea_green", type(170, 178, 32) },\
	{ "pale_green", type(152, 251, 152) },\
	{ "spring_green", type(127, 255, 0) },\
	{ "lawn_green", type(0, 252, 124) },\
	{ "chartreuse", type(0, 255, 127) },\
	{ "medium_spring_green", type(154, 250, 0) },\
	{ "green_yellow", type(47, 255, 173) },\
	{ "lime_green", type(50, 205, 50) },\
	{ "yellow_green", type(50, 205, 154) },\
	{ "forest_green", type(34, 139, 34) },\
	{ "olive_drab", type(35, 142, 107) },\
	{ "dark_khaki", type(107, 183, 189) },\
	{ "khaki", type(140, 230, 240) },\
	{ "pale_goldenrod", type(170, 232, 238) },\
	{ "light_goldenrod_yellow", type(210, 250, 250) },\
	{ "light_yellow", type(224, 255, 255) },\
	{ "yellow", type(0, 255, 255) },\
	{ "gold", type(0, 215, 255) },\
	{ "light_goldenrod", type(130, 221, 238) },\
	{ "goldenrod", type(32, 165, 218) },\
	{ "dark_goldenrod", type(11, 134, 184) },\
	{ "rosy_brown", type(143, 143, 188) },\
	{ "indian_red", type(92, 92, 205) },\
	{ "saddle_brown", type(19, 69, 139) },\
	{ "sienna", type(45, 82, 160) },\
	{ "peru", type(63, 133, 205) },\
	{ "burlywood", type(135, 184, 222) },\
	{ "beige", type(220, 245, 245) },\
	{ "wheat", type(179, 222, 245) },\
	{ "sandy_brown", type(96, 164, 244) },\
	{ "tan", type(140, 180, 210) },\
	{ "chocolate", type(30, 105, 210) },\
	{ "firebrick", type(34, 34, 178) },\
	{ "brown", type(42, 42, 165) },\
	{ "dark_salmon", type(122, 150, 233) },\
	{ "salmon", type(114, 128, 250) },\
	{ "light_salmon", type(122, 160, 255) },\
	{ "orange", type(0, 165, 255) },\
	{ "dark_orange", type(0, 140, 255) },\
	{ "coral", type(80, 127, 255) },\
	{ "light_coral", type(128, 128, 240) },\
	{ "tomato", type(71, 99, 255) },\
	{ "orange_red", type(0, 69, 255) },\
	{ "red", type(0, 0, 255) },\
	{ "hot_pink", type(180, 105, 255) },\
	{ "deep_pink", type(147, 20, 255) },\
	{ "pink", type(203, 192, 255) },\
	{ "light_pink", type(193, 182, 255) },\
	{ "pale_violet_red", type(147, 112, 219) },\
	{ "maroon", type(96, 48, 176) },\
	{ "medium_violet_red", type(133, 21, 199) },\
	{ "violet_red", type(144, 32, 208) },\
	{ "violet", type(238, 130, 238) },\
	{ "plum", type(221, 160, 221) },\
	{ "orchid", type(214, 112, 218) },\
	{ "medium_orchid", type(211, 85, 186) },\
	{ "dark_orchid", type(204, 50, 153) },\
	{ "dark_violet", type(211, 0, 148) },\
	{ "blue_violet", type(226, 43, 138) },\
	{ "purple", type(240, 32, 160) },\
	{ "medium_purple", type(219, 112, 147) },\
	{ "thistle", type(216, 191, 216) },\
	{ "green", type(0, 255, 0) },\
	{ "magenta", type(255, 0, 255) }\
	}

_MTF_BEGIN_NAMESPACE
namespace utils{
	ObjStruct::ObjStruct(){
		corners.create(2, 4, CV_64FC1);
		size_x = size_y = 0;
		pos_x = pos_y = 0;
	}
	void ObjStruct::updateCornerMat(){
		corners.at<double>(0, 0) = min_point.x;
		corners.at<double>(0, 1) = max_point.x;
		corners.at<double>(0, 2) = max_point.x;
		corners.at<double>(0, 3) = min_point.x;
		corners.at<double>(1, 0) = min_point.y;
		corners.at<double>(1, 1) = min_point.y;
		corners.at<double>(1, 2) = max_point.y;
		corners.at<double>(1, 3) = max_point.y;
	}
	void ObjStruct::updateCornerPoints(){
		mtf::Rectd  best_fit_rect = mtf::utils::getBestFitRectangle<double>(corners);
		min_point.x = best_fit_rect.x;
		min_point.y = best_fit_rect.y;
		max_point.x = best_fit_rect.x + best_fit_rect.width;
		max_point.y = best_fit_rect.y + best_fit_rect.height;
		size_x = best_fit_rect.width;
		size_y = best_fit_rect.height;
		pos_x = cv::mean(corners.row(0))[0];
		pos_y = cv::mean(corners.row(1))[0];
		//pos_x=(ulx+urx+llx+lrx)/4.0;
		//pos_y=(uly+ury+lly+lry)/4.0;
		//size_x = ((urx-ulx)+(lrx-llx))/2.0;
		//size_y = ((lly-uly)+(lry-ury))/2.0;
	}
	void ObjStruct::operator*=(double resize_factor){
		min_point.x *= resize_factor;
		min_point.y *= resize_factor;
		max_point.x *= resize_factor;
		max_point.y *= resize_factor;
		size_x *= resize_factor;
		size_y *= resize_factor;
		pos_x *= resize_factor;
		pos_y *= resize_factor;
		corners *= resize_factor;
	}
	int clicked_point_count;
	cv::Point mouse_click_point;
	bool point_selected;
	bool left_button_clicked;
	bool right_button_clicked;
	bool middle_button_clicked;
	cv::Point mouse_hover_point;
	bool mouse_hover_event;
	double hover_font_size = 0.50;
	cv::Scalar hover_color(0, 255, 0);

	inline void getClickedPoint(int mouse_event, int x, int y, int flags,
		void* param) {
		if(mouse_event == CV_EVENT_LBUTTONDOWN) {
			//cout<<"\nReceived a left click at "<<x<<"\t"<<y<<"\n";
			mouse_click_point.x = x;
			mouse_click_point.y = y;
			left_button_clicked = true;
			point_selected = true;
		} else if(mouse_event == CV_EVENT_RBUTTONDOWN) {
			//cout<<"\nReceived a right click at "<<x<<"\t"<<y<<"\n";
			point_selected = true;
			right_button_clicked = true;
		} else if(mouse_event == CV_EVENT_MBUTTONDOWN) {
			//cout<<"\nReceived a right click at "<<x<<"\t"<<y<<"\n";
			point_selected = true;
			middle_button_clicked = true;
		} else if(mouse_event == CV_EVENT_MOUSEMOVE) {
			mouse_hover_point.x = x;
			mouse_hover_point.y = y;
			mouse_hover_event = true;
		}
	}
//	const std::map<std::string, cv::Scalar> ObjUtils::col_rgb = colors(cv::Scalar);
//#ifndef DISABLE_VISP
//	const std::map<std::string, vpColor> ObjUtils::col_rgb_vp = colors(vpColor);
//#endif
	ObjUtils::ObjUtils(const vector_s &_obj_cols, double _resize_factor) :
		col_rgb(colors(cv::Scalar)),
#ifndef DISABLE_VISP
		col_rgb_vp(colors(vpColor)),
#endif
		init_frame_id(0), use_reinit_gt(false), 
		resize_factor(_resize_factor), 
		resized_images(_resize_factor != 1),
		invert_seq(false)
		{
		for(vector_s::const_iterator it = _obj_cols.begin(); it != _obj_cols.end(); ++it) {
			try{
				obj_cols.push_back(col_rgb.at(*it));
			} catch(std::out_of_range){
				printf("Invalid color provided: %s", it->c_str());
			}
		}
		if(obj_cols.empty()) {
			obj_cols.push_back(col_rgb.at("red"));
			obj_cols.push_back(col_rgb.at("green"));
			obj_cols.push_back(col_rgb.at("blue"));
			obj_cols.push_back(col_rgb.at("yellow"));
			obj_cols.push_back(col_rgb.at("magenta"));
			obj_cols.push_back(col_rgb.at("cyan"));
			obj_cols.push_back(col_rgb.at("white"));
			obj_cols.push_back(col_rgb.at("black"));
		}
		no_of_cols = obj_cols.size();
#ifndef DISABLE_VISP
		for(vector_s::const_iterator it = _obj_cols.begin(); it != _obj_cols.end(); ++it) {
			try{
				obj_cols_vp.push_back(col_rgb_vp.at(*it));
			} catch(std::out_of_range){
				printf("Invalid color provided: %s", it->c_str());
			}
			
		}
		if(obj_cols.empty()) {
			obj_cols_vp.push_back(vpColor::red);
			obj_cols_vp.push_back(vpColor::green);
			obj_cols_vp.push_back(vpColor::blue);
			obj_cols_vp.push_back(vpColor::cyan);
			obj_cols_vp.push_back(vpColor::lightRed);
			obj_cols_vp.push_back(vpColor::yellow);
			obj_cols_vp.push_back(vpColor::purple);
			obj_cols_vp.push_back(vpColor::orange);
			obj_cols_vp.push_back(vpColor::white);
			obj_cols_vp.push_back(vpColor::black);
		}
		no_of_cols_vp = obj_cols_vp.size();
#endif		
	}
	ObjUtils::~ObjUtils(){
		init_objects.clear();
		obj_cols.clear();
#ifndef DISABLE_VISP
		obj_cols_vp.clear();
#endif		
		ground_truth.clear();
		reinit_ground_truth.clear();
	}
	bool ObjUtils::addRectObject(InputBase *input, string selection_window,
		int line_thickness, int patch_size) {
		//cout<<"Start getObject\n";
		ObjStruct new_obj;
		cv::Mat hover_image(static_cast<int>(input->getHeight()*resize_factor),
			static_cast<int>(input->getWidth()*resize_factor), input->getFrame().type());
		cv::setMouseCallback(selection_window, getClickedPoint, nullptr);

		int curr_col_id = init_objects.size() % no_of_cols;
		point_selected = false;
		clicked_point_count = 0;
		while(clicked_point_count < 2) {
			if(resized_images){
				cv::resize(input->getFrame(), hover_image, hover_image.size());
			} else{
				hover_image = input->getFrame().clone();
			}
			// draw existing objects
			for(unsigned int obj_id = 0; obj_id < init_objects.size(); obj_id++){
				int col_id = obj_id % no_of_cols;
				cv::rectangle(hover_image, init_objects[obj_id].min_point, init_objects[obj_id].max_point,
					obj_cols[col_id], line_thickness);
			}
			if(clicked_point_count > 0){
				int hover_width = static_cast<int>(abs(mouse_hover_point.x - new_obj.min_point.x));
				int hover_height = static_cast<int>(abs(mouse_hover_point.y - new_obj.min_point.y));
				cv::rectangle(hover_image, new_obj.min_point, mouse_hover_point, obj_cols[curr_col_id], line_thickness);
				cv::putText(hover_image, cv::format("(%d, %d)", hover_width, hover_height),
					mouse_hover_point, cv::FONT_HERSHEY_SIMPLEX, hover_font_size, hover_color);
			}
			//while((!point_selected) && (!mouse_hover_event)) {
			//	//cout<<"Inside the loop\n";
			//	//cvWaitKey(1);
			//	int pressed_key = cv::waitKey(1);
			//	if(pressed_key == 27) {
			//		exit(0);
			//	}
			//}
			if(left_button_clicked) {
				left_button_clicked = false;
				++clicked_point_count;
				if(clicked_point_count == 1) {
					//printf("Adding min point: %d %d\n", mouse_click_point.x, mouse_click_point.y);
					if(patch_size > 0){
						new_obj.min_point.x = mouse_click_point.x - patch_size / 2.0;
						new_obj.min_point.y = mouse_click_point.y - patch_size / 2.0;
						new_obj.max_point.x = mouse_click_point.x + patch_size / 2.0;
						new_obj.max_point.y = mouse_click_point.y + patch_size / 2.0;
						break;
					} else{
						new_obj.min_point.x = mouse_click_point.x;
						new_obj.min_point.y = mouse_click_point.y;
					}
					mouse_hover_point = mouse_click_point;
				} else if(clicked_point_count == 2) {
					//printf("Adding max point: %d %d\n", mouse_click_point.x, mouse_click_point.y);
					new_obj.max_point.x = mouse_click_point.x;
					new_obj.max_point.y = mouse_click_point.y;

					if(new_obj.min_point.x > new_obj.max_point.x) {
						double temp = new_obj.min_point.x;
						new_obj.min_point.x = new_obj.max_point.x;
						new_obj.max_point.x = temp;
					}
					if(new_obj.min_point.y > new_obj.max_point.y) {
						double temp = new_obj.min_point.y;
						new_obj.min_point.y = new_obj.max_point.y;
						new_obj.max_point.y = temp;
					}
					break;
				}
			} else if(middle_button_clicked){
				middle_button_clicked = false;
				if(clicked_point_count > 0){
					--clicked_point_count;
				}
			} else if(mouse_hover_event) {
				mouse_hover_event = false;
			}
			if(point_selected) {
				point_selected = false;
				//cout<<"\nclicked_point_count="<<clicked_point_count<<"\n";
			}
			imshow(selection_window, hover_image);
			int pressed_key = cv::waitKey(1);
			if(pressed_key % 256 == 27) {
				return false;
			} else if(pressed_key % 256 == 8){
				if(clicked_point_count > 0){
					--clicked_point_count;
				}
			}
			if(!input->update()){ return false; }
		}
		new_obj.size_x = abs(new_obj.max_point.x - new_obj.min_point.x);
		new_obj.size_y = abs(new_obj.max_point.y - new_obj.min_point.y);

		new_obj.pos_x = (new_obj.min_point.x + new_obj.max_point.x) / 2;
		new_obj.pos_y = (new_obj.min_point.y + new_obj.max_point.y) / 2;
		new_obj.updateCornerMat();

		init_objects.push_back(new_obj);
		return true;
	}
	/**
	allows the user to select a quadrilateral by clicking on its 4 corners
	*/
	bool ObjUtils::addQuadObject(InputBase *input, string selection_window,
		int line_thickness) {
		//cout<<"Start getObject\n";

		ObjStruct new_obj;

		cv::Mat hover_image(static_cast<int>(input->getHeight()*resize_factor),
			static_cast<int>(input->getWidth()*resize_factor), input->getFrame().type());
		//printf("resize_factor: %f\n", resize_factor);
		//printf("input->size(): %d x %d\n", input->getHeight(), input->getWidth());
		//printf("hover_image.size(): %d x %d\n", hover_image.size().height, hover_image.size().width);

		cv::Point clicked_pts[4];
		int curr_col_id = init_objects.size() % no_of_cols;

		cv::setMouseCallback(selection_window, getClickedPoint, nullptr);
		clicked_point_count = 0;
		while(clicked_point_count < 4) {
			if(resized_images){
				cv::resize(input->getFrame(), hover_image, hover_image.size());
			} else{
				hover_image = input->getFrame().clone();
			}

			// draw existing objects
			for(unsigned int obj_id = 0; obj_id < init_objects.size(); ++obj_id){
				int col_id = obj_id % no_of_cols;
				for(int pt_id = 0; pt_id < 3; ++pt_id){
					cv::Point2d pt1(init_objects[obj_id].corners.at<double>(0, pt_id), init_objects[obj_id].corners.at<double>(1, pt_id));
					cv::Point2d pt2(init_objects[obj_id].corners.at<double>(0, pt_id + 1), init_objects[obj_id].corners.at<double>(1, pt_id + 1));
					cv::line(hover_image, pt1, pt2, obj_cols[col_id], line_thickness);
				}
				cv::Point2d pt1(init_objects[obj_id].corners.at<double>(0, 0), init_objects[obj_id].corners.at<double>(1, 0));
				cv::Point2d pt2(init_objects[obj_id].corners.at<double>(0, 3), init_objects[obj_id].corners.at<double>(1, 3));
				cv::line(hover_image, pt1, pt2, obj_cols[col_id], line_thickness);
			}
			// draw new (incomplete) object
			for(int pt_id = 0; pt_id < clicked_point_count - 1; ++pt_id){
				cv::line(hover_image, clicked_pts[pt_id], clicked_pts[pt_id + 1], obj_cols[curr_col_id], line_thickness);
			}
			if(clicked_point_count > 0){
				cv::line(hover_image, clicked_pts[clicked_point_count - 1], mouse_hover_point, obj_cols[curr_col_id], line_thickness);
				if(clicked_point_count == 3){
					cv::line(hover_image, clicked_pts[0], mouse_hover_point, obj_cols[curr_col_id], line_thickness);
				}
			}

			point_selected = false;
			//while((!point_selected) && (!mouse_hover_event)) {
			//	//cout<<"Inside the loop\n";
			//	//cvWaitKey(1);
			//	int pressed_key = cv::waitKey(1);
			//	if(pressed_key == 27) {
			//		exit(0);
			//	}
			//}
			if(left_button_clicked) {
				left_button_clicked = false;
				clicked_pts[clicked_point_count] = mouse_click_point;

				new_obj.corners.at<double>(0, clicked_point_count) = mouse_click_point.x;
				new_obj.corners.at<double>(1, clicked_point_count) = mouse_click_point.y;
				++clicked_point_count;
				if(clicked_point_count == 4){
					break;
				} else if(clicked_point_count == 1){
					mouse_hover_point = mouse_click_point;
				}

			} else if(middle_button_clicked){
				middle_button_clicked = false;
				if(clicked_point_count > 0){
					--clicked_point_count;
				}
			} else if(mouse_hover_event) {
				mouse_hover_event = false;
				//if(new_obj && (clicked_point_count > 0)) {
				//	cv::line(hover_image, clicked_pts[clicked_point_count - 1], mouse_hover_point, obj_cols[col_id], line_thickness);
				//	if(clicked_point_count == 3){
				//		cv::line(hover_image, clicked_pts[0], mouse_hover_point, obj_cols[col_id], line_thickness);
				//	}
				//}
			}
			imshow(selection_window, hover_image);
			int pressed_key = cv::waitKey(1);
			if(pressed_key % 256 == 27) {
				return false;
			} else if(pressed_key % 256 == 8){
				if(clicked_point_count > 0){
					--clicked_point_count;
				}
			}
			if(!input->update()){ return false; }
		}
		new_obj.min_point.x = new_obj.corners.at<double>(0, 0);
		new_obj.min_point.y = new_obj.corners.at<double>(1, 0);
		new_obj.max_point.x = new_obj.corners.at<double>(0, 2);
		new_obj.max_point.y = new_obj.corners.at<double>(1, 2);

		new_obj.size_x = ((new_obj.corners.at<double>(0, 1) - new_obj.corners.at<double>(0, 0)) +
			(new_obj.corners.at<double>(0, 2) - new_obj.corners.at<double>(0, 3))) / 2;
		new_obj.size_y = ((new_obj.corners.at<double>(1, 3) - new_obj.corners.at<double>(1, 0)) +
			(new_obj.corners.at<double>(1, 2) - new_obj.corners.at<double>(1, 1))) / 2;

		new_obj.pos_x = (new_obj.min_point.x + new_obj.max_point.x) / 2;
		new_obj.pos_y = (new_obj.min_point.y + new_obj.max_point.y) / 2;

		init_objects.push_back(new_obj);
		return true;
	}
	//! overloaded variant for non-live input feed
	bool ObjUtils::selectObjects(const cv::Mat &img, int no_of_objs,
		int patch_size, int line_thickness, int write_objs, bool sel_quad_obj,
		const char* filename){
		InputDummy input(img);
		return selectObjects(&input, no_of_objs, patch_size, line_thickness, write_objs,
			sel_quad_obj, filename);
	}
	bool ObjUtils::selectObjects(InputBase *input, int no_of_objs,
		int patch_size, int line_thickness, int write_objs, bool sel_quad_obj,
		const char* filename) {
		stringstream temp_stream;
		if(no_of_objs > 1) {
			temp_stream << "Please select " << no_of_objs << " objects to track";
			if(patch_size > 0){
				temp_stream << " by clicking at the center of each to add a " << patch_size << "x" << patch_size << " patch";
			} else if(sel_quad_obj){
				temp_stream << " by clicking at the four corners of each";
			} else{
				temp_stream << " by clicking at two opposite corners of each";
			}
		} else {
			temp_stream << "Please select the object to track";
			if(patch_size > 0){
				temp_stream << " by clicking at its center to add a " << patch_size << "x" << patch_size << " patch";
			} else if(sel_quad_obj){
				temp_stream << " by clicking at its four corners";
			} else{
				temp_stream << " by clicking at its two opposite corners";
			}
		}
		string window_title = temp_stream.str();
		cv::namedWindow(window_title, 1);
		for(int i = 0; i < no_of_objs; ++i) {
			if(sel_quad_obj){
				printf("selecting quadrilateral object...\n");
				if(!addQuadObject(input, window_title, line_thickness)){
					cv::destroyWindow(window_title);
					return false;
				};
			} else{
				if(!addRectObject(input, window_title, line_thickness, patch_size)){
					cv::destroyWindow(window_title);
					return false;
				};
			}
		}
		cv::destroyWindow(window_title);
		if(write_objs){
			writeObjectsToFile(no_of_objs, filename);
		}
		return !init_objects.empty();
	}
#ifndef DISABLE_VISP
	bool ObjUtils::addRectObjectVP(InputBase *input, string selection_window,
		int line_thickness, int patch_size) {
		std::unique_ptr<vpDisplay> display;
		// Select one of the available video-devices
#if defined VISP_HAVE_X11
		display.reset(new vpDisplayX);
#elif defined VISP_HAVE_GTK
		display.reset(new vpDisplayGTK);
#elif defined VISP_HAVE_GDI
		display.reset(new vpDisplayGDI);
#elif defined VISP_HAVE_D3D9
		display.reset(new vpDisplayD3D);
#elif defined VISP_HAVE_OPENCV
		display.reset(new vpDisplayOpenCV);
#else
		throw InvalidArgument("None of the window backends supported by ViSP are available");
#endif 
		int n_existing_objs = init_objects.size();
		vpColor curr_col = getColVp(n_existing_objs);

		ObjStruct new_obj;
		vpImage<vpRGBa> hover_image(static_cast<int>(input->getHeight()),
			static_cast<int>(input->getWidth()));
		input->getFrame(hover_image);
		display->init(hover_image, -1, -1, selection_window);
		vpDisplay::display(hover_image);
		vpImagePoint clicked_point, hover_point;
		int _clicked_point_count = 0;
		vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
		while(_clicked_point_count < 2) {	
			while(!vpDisplay::getClick(hover_image, clicked_point, button, false)) {

				if(!input->update()){ return false; }
				input->getFrame(hover_image);

				vpDisplay::getPointerMotionEvent(hover_image, hover_point);

				if(_clicked_point_count > 0){
					vpImagePoint min_point(new_obj.min_point.y, new_obj.min_point.x);
					int hover_width = static_cast<int>(abs(hover_point.get_j() - new_obj.min_point.x));
					int hover_height = static_cast<int>(abs(hover_point.get_i() - new_obj.min_point.y));
					vpDisplay::displayRectangle(hover_image, min_point, hover_width, hover_height, 
						curr_col, false, line_thickness);
				} else {
					vpDisplay::displayPoint(hover_image, hover_point, curr_col, line_thickness);
				}
				for(unsigned int obj_id = 0; obj_id < n_existing_objs; ++obj_id) {
					mtf::utils::drawRegion(hover_image, init_objects[obj_id].corners, getColVp(obj_id), line_thickness);
				}
				vpDisplay::flush(hover_image);
				vpDisplay::display(hover_image);
				char pressed_key;
				if(vpDisplay::getKeyboardEvent(hover_image, &pressed_key, false)) {
					if(pressed_key % 256 == 27){
						vpDisplay::close(hover_image);
						return false;
					}
				}
			}
			++_clicked_point_count;
			if(_clicked_point_count == 1) {
				//printf("Adding min point: %d %d\n", mouse_click_point.x, mouse_click_point.y);
				if(patch_size > 0){
					new_obj.min_point.x = clicked_point.get_j() - patch_size / 2.0;
					new_obj.min_point.y = clicked_point.get_i() - patch_size / 2.0;
					new_obj.max_point.x = clicked_point.get_j() + patch_size / 2.0;
					new_obj.max_point.y = clicked_point.get_i() + patch_size / 2.0;
					break;
				} else{
					new_obj.min_point.x = clicked_point.get_j();
					new_obj.min_point.y = clicked_point.get_i();
				}
				hover_point = clicked_point;
			} else if(_clicked_point_count == 2) {
				//printf("Adding max point: %d %d\n", mouse_click_point.x, mouse_click_point.y);
				new_obj.max_point.x = clicked_point.get_j();
				new_obj.max_point.y = clicked_point.get_i();

				if(new_obj.min_point.x > new_obj.max_point.x) {
					double temp = new_obj.min_point.x;
					new_obj.min_point.x = new_obj.max_point.x;
					new_obj.max_point.x = temp;
				}
				if(new_obj.min_point.y > new_obj.max_point.y) {
					double temp = new_obj.min_point.y;
					new_obj.min_point.y = new_obj.max_point.y;
					new_obj.max_point.y = temp;
				}
				break;
			}
		}
		new_obj.size_x = abs(new_obj.max_point.x - new_obj.min_point.x);
		new_obj.size_y = abs(new_obj.max_point.y - new_obj.min_point.y);

		new_obj.pos_x = (new_obj.min_point.x + new_obj.max_point.x) / 2;
		new_obj.pos_y = (new_obj.min_point.y + new_obj.max_point.y) / 2;
		new_obj.updateCornerMat();

		init_objects.push_back(new_obj);
		vpDisplay::close(hover_image);
		return true;
	}
	bool ObjUtils::addQuadObjectVP(InputBase *input, string selection_window,
		int line_thickness) {
		std::unique_ptr<vpDisplay> display;
		// Select one of the available video-devices
#if defined VISP_HAVE_X11
	display.reset(new vpDisplayX);
#elif defined VISP_HAVE_GTK
		display.reset(new vpDisplayGTK);
#elif defined VISP_HAVE_GDI
		display.reset(new vpDisplayGDI);
#elif defined VISP_HAVE_D3D9
		display.reset(new vpDisplayD3D);
#elif defined VISP_HAVE_OPENCV
		display.reset(new vpDisplayOpenCV);
#else
		throw InvalidArgument("None of the window backends supported by ViSP are available");
#endif 
		int n_existing_objs = init_objects.size();
		vpColor curr_col = getColVp(n_existing_objs);

		ObjStruct new_obj;
		vpImage<vpRGBa> hover_image(static_cast<int>(input->getHeight()),
			static_cast<int>(input->getWidth()));
		input->getFrame(hover_image);
		display->init(hover_image, -1, -1, selection_window);
		vpDisplay::display(hover_image);

		vpImagePoint clicked_pts[4];
		vpImagePoint clicked_point, hover_point;
		int _clicked_point_count = 0;
		vpMouseButton::vpMouseButtonType button = vpMouseButton::button1;
		while(_clicked_point_count < 4) {
			while(!vpDisplay::getClick(hover_image, clicked_point, button, false)) {

				if(!input->update()){ return false; }
				input->getFrame(hover_image);

				vpDisplay::getPointerMotionEvent(hover_image, hover_point);

				// draw new (incomplete) object
				for(int pt_id = 0; pt_id < _clicked_point_count - 1; ++pt_id){
					vpDisplay::displayLine(hover_image, clicked_pts[pt_id], clicked_pts[pt_id + 1], curr_col, line_thickness);
				}
				if(_clicked_point_count > 0){
					vpDisplay::displayLine(hover_image, clicked_pts[_clicked_point_count - 1], hover_point, curr_col, line_thickness);
					if(_clicked_point_count == 3){
						vpDisplay::displayLine(hover_image, clicked_pts[0], hover_point, curr_col, line_thickness);
					}
				}
				for(unsigned int obj_id = 0; obj_id < n_existing_objs; ++obj_id) {
					mtf::utils::drawRegion(hover_image, init_objects[obj_id].corners, getColVp(obj_id), line_thickness);
				}

				vpDisplay::flush(hover_image);
				vpDisplay::display(hover_image);
				char pressed_key;
				if(vpDisplay::getKeyboardEvent(hover_image, &pressed_key, false)) {
					if(pressed_key % 256 == 27){
						vpDisplay::close(hover_image);
						return false;
					}
				}
			}
			clicked_pts[_clicked_point_count] = hover_point;

			new_obj.corners.at<double>(0, _clicked_point_count) = clicked_point.get_j();
			new_obj.corners.at<double>(1, _clicked_point_count) = clicked_point.get_i();

			++_clicked_point_count;
			if(_clicked_point_count == 4){
				break;
			} 
			if(_clicked_point_count == 1){
				hover_point = clicked_point;
			}
		}

		new_obj.min_point.x = new_obj.corners.at<double>(0, 0);
		new_obj.min_point.y = new_obj.corners.at<double>(1, 0);
		new_obj.max_point.x = new_obj.corners.at<double>(0, 2);
		new_obj.max_point.y = new_obj.corners.at<double>(1, 2);

		new_obj.size_x = ((new_obj.corners.at<double>(0, 1) - new_obj.corners.at<double>(0, 0)) +
			(new_obj.corners.at<double>(0, 2) - new_obj.corners.at<double>(0, 3))) / 2;
		new_obj.size_y = ((new_obj.corners.at<double>(1, 3) - new_obj.corners.at<double>(1, 0)) +
			(new_obj.corners.at<double>(1, 2) - new_obj.corners.at<double>(1, 1))) / 2;

		new_obj.pos_x = (new_obj.min_point.x + new_obj.max_point.x) / 2;
		new_obj.pos_y = (new_obj.min_point.y + new_obj.max_point.y) / 2;

		init_objects.push_back(new_obj);
		vpDisplay::close(hover_image);

		return true;
	}
	//! overloaded variant for non-live input feed
	bool ObjUtils::selectObjectsVP(const cv::Mat &img, int no_of_objs,
		int patch_size, int line_thickness, int write_objs, bool sel_quad_obj,
		const char* filename){
		InputDummy input(img);
		return selectObjectsVP(&input, no_of_objs, patch_size, line_thickness, write_objs,
			sel_quad_obj, filename);
	}
	bool ObjUtils::selectObjectsVP(InputBase *input, int no_of_objs,
		int patch_size, int line_thickness, int write_objs, bool sel_quad_obj,
		const char* filename) {
		printf("ObjUtils::selectObjectsVP...\n");
		stringstream temp_stream;
		if(no_of_objs > 1) {
			temp_stream << "Please select " << no_of_objs << " objects to track";
			if(patch_size > 0){
				temp_stream << " by clicking at the center of each to add a " << patch_size << "x" << patch_size << " patch";
			} else if(sel_quad_obj){
				temp_stream << " by clicking at the four corners of each";
			} else{
				temp_stream << " by clicking at two opposite corners of each";
			}
		} else {
			temp_stream << "Please select the object to track";
			if(patch_size > 0){
				temp_stream << " by clicking at its center to add a " << patch_size << "x" << patch_size << " patch";
			} else if(sel_quad_obj){
				temp_stream << " by clicking at its four corners";
			} else{
				temp_stream << " by clicking at its two opposite corners";
			}
		}
		string window_title = temp_stream.str();
		for(int i = 0; i < no_of_objs; ++i) {
			if(sel_quad_obj){
				printf("selecting quadrilateral object...\n");
				if(!addQuadObjectVP(input, window_title, line_thickness)){
					return false;
				}
			} else{
				if(!addRectObjectVP(input, window_title, line_thickness, patch_size)){
					return false;
				}
			}
		}
		if(write_objs){
			writeObjectsToFile(no_of_objs, filename);
		}
		return !init_objects.empty();
	}
#endif
	void ObjUtils::writeObjectsToFile(int no_of_objs, const char* filename){
		ofstream fout;
		cout << "Writing object locations to file: " << filename << "\n";
		fout.open(filename, ios::out);
		if(!fout) {
			cout << "Could not open file for writing object locations.\n";
		}
		for(int obj_id = 0; obj_id < no_of_objs; obj_id++) {
			fout << init_objects[obj_id].max_point.x << "\t";
			fout << init_objects[obj_id].max_point.y << "\t";
			fout << init_objects[obj_id].min_point.x << "\t";
			fout << init_objects[obj_id].min_point.y << "\t";
			fout << init_objects[obj_id].size_x << "\t";
			fout << init_objects[obj_id].size_y << "\n";
		}
		fout.close();
	}
	bool ObjUtils::readObjectFromGT(string source_name, string source_path, int n_frames,
		int _init_frame_id, bool use_opt_gt, string opt_gt_ssm,
		bool _use_reinit_gt, bool _invert_seq, int debug_mode){

		init_frame_id = _init_frame_id;
		use_reinit_gt = _use_reinit_gt;
		invert_seq = _invert_seq;

		if(n_frames <= 0){
			printf("Error while reading objects from ground truth: n_frames: %d is too small\n", n_frames);
			return false;
		}
		if(_init_frame_id >= n_frames){
			printf("Error while reading objects from ground truth: init_frame_id: %d is larger than n_frames: %d\n",
				_init_frame_id, n_frames);
			return false;
		}
		if(_use_reinit_gt){
			if(!readReinitGT(source_name, source_path, _init_frame_id, n_frames, use_opt_gt, opt_gt_ssm)){
				printf("Reinitialization ground truth could not be read for frame %d\n", init_frame_id + 1);
				return false;
			}
			printf("Using reinit ground truth for frame %d as the main ground truth\n", _init_frame_id + 1);
			ground_truth = reinit_ground_truth;
		} else if(!readGT(source_name, source_path, n_frames, _init_frame_id, debug_mode, use_opt_gt, opt_gt_ssm)){
			printf("Ground truth could not be read for frame %d\n", init_frame_id + 1);
			return false;
		}
		if(!_use_reinit_gt && static_cast<unsigned int>(_init_frame_id) >= ground_truth.size()){
			printf("The provided init_frame_id: %d is larger than the maximum frame id in the ground truth: %lu\n",
				_init_frame_id, ground_truth.size() - 1);
			return false;
		}
		ObjStruct obj;
		obj.corners = _use_reinit_gt ? ground_truth[0] : ground_truth[_init_frame_id];
		obj.updateCornerPoints();
		//if(resized_images){
		//	obj *= resize_factor;
		//}
		init_objects.push_back(obj);
		//printf("Done reading objects from ground truth\n");
		return true;
	}
	const cv::Mat& ObjUtils::getGT(int frame_id, int _reinit_frame_id){
		if(_reinit_frame_id < 0){ _reinit_frame_id = frame_id; }
		if(use_reinit_gt){
			if(frame_id < _reinit_frame_id){
				throw mtf::utils::InvalidArgument(
					cv::format("getGT :: frame_id: %d is less than reinit_frame_id: %d", frame_id, _reinit_frame_id));
			}
			if(_reinit_frame_id != reinit_frame_id){
				readReinitGT(_reinit_frame_id);
			}
			if(frame_id - reinit_frame_id >= static_cast<int>(reinit_ground_truth.size())){
				throw mtf::utils::InvalidArgument(
					cv::format("Invalid frame ID: %d provided for reinit ground truth for frame %d with only %d entries",
					frame_id - reinit_frame_id, reinit_frame_id, reinit_ground_truth.size()));
			}
			return reinit_ground_truth[frame_id - reinit_frame_id];
		}
		if(frame_id >= static_cast<int>(ground_truth.size())){
			throw mtf::utils::InvalidArgument(
				cv::format("Invalid frame ID: %d provided for ground truth with only %d entries",
				frame_id, ground_truth.size()));
		}
		return ground_truth[frame_id];

	}
	const cv::Mat& ObjUtils::getReinitGT(int frame_id, int _reinit_frame_id){
		if(_reinit_frame_id < 0){ _reinit_frame_id = reinit_frame_id; }
		if(frame_id < _reinit_frame_id){
			throw mtf::utils::InvalidArgument(
				cv::format("getReinitGT :: frame_id: %d is less than reinit_frame_id: %d", frame_id, _reinit_frame_id));
		}
		if(_reinit_frame_id != reinit_frame_id){
			readReinitGT(_reinit_frame_id);
		}
		if(frame_id - _reinit_frame_id >= static_cast<int>(reinit_ground_truth.size())){
			throw mtf::utils::InvalidArgument(
				cv::format("Invalid frame ID: %d provided for reinit ground truth for frame %d with only %d entries",
				frame_id - _reinit_frame_id, _reinit_frame_id, reinit_ground_truth.size()));
		}
		return reinit_ground_truth[frame_id - _reinit_frame_id];
	}
	bool ObjUtils::readGT(string source_name, string source_path,
		int n_frames, int init_frame_id, int debug_mode,
		bool use_opt_gt, string opt_gt_ssm){

		string gt_filename = use_opt_gt ?
			source_path + "/OptGT/" + source_name + "_" + opt_gt_ssm + ".txt" :
			source_path + "/" + source_name + ".txt";

		printf("Reading ground truth from %s\n", gt_filename.c_str());

		ifstream fin;
		fin.open(gt_filename, ios::in);
		if(!fin) {
			printf("Could not open ground truth file for reading object location.\n");
			return false;
		}
		char header[500];
		fin.getline(header, 500);

		/**
		estimate the no. of frames from the ground truth file itself by
		counting the no. of lines in it
		*/
		if(n_frames <= 0){
			int curr_pos = static_cast<int>(fin.tellg());
			std::string line;
			n_frames = 0;
			while(std::getline(fin, line)){ ++n_frames; }
			fin.seekg(curr_pos);
		}
		//cout << "header: "<<header<<"\n";
		//cout << "n_frames: " << n_frames << "\n";
		//cout << "init_frame_id: " << init_frame_id << "\n";

		ground_truth.resize(n_frames);

		for(int frame_id = 0; frame_id < n_frames; ++frame_id) {
			fin >> header;
			if(!fin.good()){
				printf("Ground truth file has ended unexpectedly - only %d out of %d entries were read\n",
					frame_id, n_frames);
				break;
			}
			string header_1 = std::string(header).substr(0, 5);
			string header_2 = std::string(header).substr(5, 5);
			string header_3 = std::string(header).substr(10, 4);
			int header_len = strlen(header);
			if(header_len != 14 || header_1.compare("frame") ||
				atoi(header_2.c_str()) != frame_id + 1 || header_3.compare(".jpg")){
				printf("Invalid header in line %d of the ground truth file: %s\n", frame_id + 1, header);
				printf("header_len: %d\n", header_len);
				printf("header_1: %s\n", header_1.c_str());
				printf("header_2: %s\n", header_2.c_str());
				printf("header_3: %s\n", header_3.c_str());
				return false;
			}
			float ulx, uly, urx, ury, lrx, lry, llx, lly;
			fin >> ulx >> uly >> urx >> ury >> lrx >> lry >> llx >> lly;

			if(!fin){
				printf("Invalid formatting in line %d of the ground truth file. Aborting...\n", frame_id + 1);
				cout << ulx << "\t" << uly << "\t" << urx << "\t" << ury << "\t" << lrx << "\t" << lry << "\t" << llx << "\t" << lly << "\n";
				return false;
			}
			int gt_id = invert_seq ? n_frames - frame_id - 1 : frame_id;
			ground_truth[gt_id].create(2, 4, CV_64FC1);
			ground_truth[gt_id].at<double>(0, 0) = ulx;
			ground_truth[gt_id].at<double>(0, 1) = urx;
			ground_truth[gt_id].at<double>(0, 2) = lrx;
			ground_truth[gt_id].at<double>(0, 3) = llx;
			ground_truth[gt_id].at<double>(1, 0) = uly;
			ground_truth[gt_id].at<double>(1, 1) = ury;
			ground_truth[gt_id].at<double>(1, 2) = lry;
			ground_truth[gt_id].at<double>(1, 3) = lly;
			if(resized_images) {
				ground_truth[gt_id] *= resize_factor;
			}
		}
		fin.close();
		return true;
	}
	bool ObjUtils::readReinitGT(string source_name, string source_path,
		int _reinit_frame_id, int _n_frames, bool use_opt_gt,
		string opt_gt_ssm){
		if(_n_frames <= 0){
			printf("Error while reading reinit ground truth: n_frames: %d is too small\n", _n_frames);
			return false;
		}
		if(invert_seq){
			reinit_gt_filename = use_opt_gt ?
				cv::format("%s/ReinitGT/%s_%s_inv.bin", source_path.c_str(), source_name.c_str(), opt_gt_ssm.c_str()) :
				cv::format("%s/ReinitGT/%s_inv.bin", source_path.c_str(), source_name.c_str());
		} else{
			reinit_gt_filename = use_opt_gt ?
				cv::format("%s/ReinitGT/%s_%s.bin", source_path.c_str(), source_name.c_str(), opt_gt_ssm.c_str()) :
				cv::format("%s/ReinitGT/%s.bin", source_path.c_str(), source_name.c_str());
		}
		printf("Reading reinit ground truth from file: %s\n", reinit_gt_filename.c_str());
		ifstream fin(reinit_gt_filename, ios::in | ios::binary);
		if(!fin.good()) {
			printf("File could not be opened.\n");
			return false;
		}
		int expected_file_size = (_n_frames*(_n_frames + 1) * 4) * sizeof(double) + sizeof(int);
		fin.seekg(0, ios_base::end);
		int actual_file_size = static_cast<int>(fin.tellg());
		if(actual_file_size != expected_file_size){
			printf("Size of the file: %d does not match the expected size: %d\n",
				actual_file_size, expected_file_size);
			return false;
		}
		fin.seekg(0, ios_base::beg);
		fin.read((char*)(&reinit_n_frames), sizeof(int));
		if(reinit_n_frames != _n_frames){
			printf("File contains data for %d rather than %d frames\n", reinit_n_frames, _n_frames);
			return false;
		}
		fin.close();
		printf("Reinit ground truth successfully read\n");
		readReinitGT(_reinit_frame_id);

		return true;
	}
	bool ObjUtils::readObjectsFromFile(int no_of_objs,
		const char* filename, int debug_mode){
		ifstream fin;
		cout << "Reading object locations from file: " << filename << "\n";
		fin.open(filename, ios::in);
		if(!fin) {
			printf("Could not open file for reading object locations.\n");
			return false;
		}
		for(int i = 0; i < no_of_objs; i++) {
			ObjStruct obj;
			fin >> obj.max_point.x;
			fin >> obj.max_point.y;
			fin >> obj.min_point.x;
			fin >> obj.min_point.y;
			fin >> obj.size_x;
			fin >> obj.size_y;

			obj.pos_x = (obj.min_point.x + obj.max_point.x) / 2;
			obj.pos_y = (obj.min_point.y + obj.max_point.y) / 2;

			obj.updateCornerMat();

			if(resized_images){
				obj *= resize_factor;
			}

			init_objects.push_back(obj);

			//if(debug_mode){
			//	cout<<"Object "<<i<<":\n\t";
			//	cout<<"pos_x="<<obj->pos_x<<"\n\t";
			//	cout<<"pos_y="<<obj->pos_y<<"\n\t";
			//	cout<<"size_x="<<obj->size_x<<"\n\t";
			//	cout<<"size_y="<<obj->size_y<<"\n";
			//}
		}
		return true;

	}
	cv::Point ObjUtils::getMeanPoint(cv::Point *pt_array, int no_of_points){
		long sum_x = 0;
		long sum_y = 0;
		for(int i = 0; i < no_of_points; i++){
			sum_x += pt_array[i].x;
			sum_y += pt_array[i].y;
		}
		return cv::Point(sum_x / no_of_points, sum_y / no_of_points);
	}
	void ObjUtils::cornersToPoint2D(cv::Point2d(&cv_corners)[4], const cv::Mat &cv_corners_mat) {
		for(int corner_id = 0; corner_id < 4; corner_id++) {
			cv_corners[corner_id].x = cv_corners_mat.at<double>(0, corner_id);
			cv_corners[corner_id].y = cv_corners_mat.at<double>(1, corner_id);
		}
	}
	void ObjUtils::readReinitGT(int _reinit_frame_id){
		reinit_frame_id = _reinit_frame_id;
		printf("Reading reinit gt for frame %d\n", reinit_frame_id + 1);
		ifstream fin(reinit_gt_filename, ios::in | ios::binary);
		//! file position where gt for _reinit_frame_id begins;
		int start_pos = _reinit_frame_id*(2 * reinit_n_frames - _reinit_frame_id + 1) * 4 * sizeof(double) + sizeof(int);
		fin.seekg(start_pos);
		reinit_ground_truth.clear();
		for(int frame_id = _reinit_frame_id; frame_id < reinit_n_frames; ++frame_id) {
			cv::Mat curr_gt(2, 4, CV_64FC1);
			fin.read((char*)(curr_gt.data), 8 * sizeof(double));
			if(resized_images) {
				curr_gt *= resize_factor;
			}
			reinit_ground_truth.push_back(curr_gt);
		}
		fin.close();
	}
	//bool readReinitGT(string source_name, string source_path,
	//	int n_frames = 1, int debug_mode = 0, bool use_opt_gt = false,
	//	bool read_from_bin = true, string opt_gt_ssm = "2"){
	//	if(n_frames <= 0){
	//		printf("Error while reading reinit ground truth: n_frames: %d is too small\n", n_frames);
	//		return false;
	//	}

	//	string gt_file_path = source_path + "/ReinitGT";

	//	//cout << "n_frames: " << n_frames << "\n";		
	//	string gt_filename;
	//	ifstream fin;
	//	if(read_from_bin){
	//		if(use_opt_gt){
	//			gt_filename = cv::format("%s/%s_%s.bin", gt_file_path.c_str(), source_name.c_str(), opt_gt_ssm.c_str());
	//		} else{
	//			gt_filename = cv::format("%s/%s.bin", gt_file_path.c_str(), source_name.c_str());
	//		}
	//		cout << "Reading reinit ground truth from binary file: " << gt_filename << "\n";
	//		fin.open(gt_filename, ios::in | ios::binary);
	//		if(!fin) {
	//			printf("Could not open ground truth file.\n");
	//			return false;
	//		}
	//		int expected_file_size = (n_frames*(n_frames + 1) * 4) * sizeof(double) + sizeof(int);
	//		fin.seekg(0, ios_base::end);
	//		int actual_file_size = fin.tellg();
	//		if(actual_file_size != expected_file_size){
	//			printf("The size of the file: %d does not match the  expected size: %d\n",
	//				actual_file_size, expected_file_size);
	//			return false;
	//		}
	//		fin.seekg(0, ios_base::beg);
	//		int reinit_n_frames;
	//		fin.read((char*)(&reinit_n_frames), sizeof(int));

	//		if(reinit_n_frames < n_frames){
	//			printf("The file contains data only for %d out of %d frames\n", reinit_n_frames, n_frames);
	//			return false;
	//		}
	//	}
	//	char header[500];
	//	for(int start_id = 0; start_id < n_frames; start_id++) {
	//		vector<cv::Mat> curr_reinit_gt;
	//		if(!read_from_bin){
	//			if(use_opt_gt){
	//				gt_filename = cv::format("%s/%s/frame%05d_%s.txt", gt_file_path.c_str(), source_name.c_str(), start_id + 1, opt_gt_ssm.c_str());
	//			} else{
	//				gt_filename = cv::format("%s/%s/frame%05d.txt", gt_file_path.c_str(), source_name.c_str(), start_id + 1);
	//			}
	//			cout << "Reading ground truth for frame " << start_id + 1 << " from file: " << gt_filename << "\n";
	//			fin.open(gt_filename, ios::in);
	//			if(!fin) {
	//				printf("Could not open ground truth file.\n");
	//				return false;
	//			}
	//			fin.getline(header, 500);
	//		}

	//		for(int frame_id = start_id; frame_id < n_frames; frame_id++) {
	//			cv::Mat curr_gt(2, 4, CV_64FC1);
	//			if(read_from_bin){
	//				fin.read((char*)(curr_gt.data), 8 * sizeof(double));
	//			} else{
	//				fin >> header;
	//				if(!fin.good()){
	//					printf("Ground truth file has ended unexpectedly - only %d out of %d entries were read\n",
	//						frame_id, n_frames);
	//					break;
	//				}
	//				//cout << "header2: "<<header<<"\n";
	//				float ulx, uly, urx, ury, lrx, lry, llx, lly;
	//				fin >> ulx >> uly >> urx >> ury >> lrx >> lry >> llx >> lly;

	//				if(!fin){
	//					printf("Invalid formatting in line %d of the ground truth file. Aborting...\n", frame_id + 1);
	//					cout << ulx << "\t" << uly << "\t" << urx << "\t" << ury << "\t" << lrx << "\t" << lry << "\t" << llx << "\t" << lly << "\n";
	//					return false;
	//				}
	//				curr_gt.at<double>(0, 0) = ulx;
	//				curr_gt.at<double>(0, 1) = urx;
	//				curr_gt.at<double>(0, 2) = lrx;
	//				curr_gt.at<double>(0, 3) = llx;
	//				curr_gt.at<double>(1, 0) = uly;
	//				curr_gt.at<double>(1, 1) = ury;
	//				curr_gt.at<double>(1, 2) = lry;
	//				curr_gt.at<double>(1, 3) = lly;
	//			}

	//			if(debug_mode){
	//				printf("start_id: %d frame_id: %d : \n", start_id, frame_id);
	//				cout << curr_gt << "\n";
	//			}
	//			curr_reinit_gt.push_back(curr_gt);
	//		}
	//		reinit_ground_truth.push_back(curr_reinit_gt);
	//		if(!read_from_bin){
	//			fin.close();
	//		}
	//	}
	//	if(read_from_bin){
	//		fin.close();
	//	}
	//	return true;
	//}

}
_MTF_END_NAMESPACE

