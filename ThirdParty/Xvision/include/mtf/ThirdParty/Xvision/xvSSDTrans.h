#ifndef MTF_XVSSD_TRANS_
#define MTF_XVSSD_TRANS_

#include "xvSSDMain.h"

class XVSSDTrans: public XVSSDMain {

    typedef XVTransStepper< IMAGE_TYPE  > STEPPER_TYPE;
    typedef XVSSD< IMAGE_TYPE, STEPPER_TYPE > TRACKER_TYPE;
    typedef TRACKER_TYPE::SP STATE_PAIR_TYPE;
    typedef STEPPER_TYPE::STATE_TYPE STATE_TYPE;

public:

    TRACKER_TYPE *ssd;
	STATE_PAIR_TYPE curr_state;
	STATE_PAIR_TYPE diff_state;	

	vector<STATE_PAIR_TYPE> states;

	XVSSDTrans(const ParamType *xv_params = NULL) :
        XVSSDMain(xv_params){
				name="trans";
		}

    void initTracker() {
		ssd=new TRACKER_TYPE(STATE_TYPE(*init_pos), init_template);
		//printf("Calling setStepper\n");
        //ssd->setStepper(init_template);
		//printf("Calling initState\n");
		//ssd->initState( (STATE_TYPE)(*xv_roi) );
		template_img = ssd->getTarget();
		curr_state=ssd->getCurrentState();
		diff_state=ssd->diffState();
		diff_tracker_state->setState(diff_state.state.PosX(), diff_state.state.PosY());
		current_tracker_state->setState(curr_state.state.PosX(), curr_state.state.PosY());
		/*printf("Obtaining warped image: warped_img=%d\n", &warped_img);
		IMAGE_TYPE_GS warped_img2=ssd->warpedImage();
		printf("Obtained warped image: warped_img=%d\n", &warped_img2);
		int img_height = warped_img2.SizeY();
		int img_width = warped_img2.SizeX();
		printf("img_height: %d, img_width=%d\n", img_height, img_width);*/
    }

    double updateTrackerState() {
		curr_state=ssd->step(*xv_frame);
		warped_img=ssd->warpedImage();
		//warp_height = warped_img.SizeY();
		//warp_width = warped_img.SizeX();

		/*printf("img_height: %d, img_width=%d\n", img_height, img_width);*/
		current_tracker_state->setState(curr_state.state.PosX(), curr_state.state.PosY());
		diff_state=ssd->diffState();
		diff_tracker_state->setState(diff_state.state.PosX(), diff_state.state.PosY());
		return curr_state.error;		
    }

	void updateCorners(){
		curr_state=ssd->getCurrentState();
		states.push_back(curr_state);
		for(int i=0; i<NCORNERS; ++i){
			corners[i] = points[i] + curr_state.state;
		}
	}
	void getTransformedPoints(vector<XVPositionD> &in_pts, vector<XVPositionD> &out_pts){
		for(int i=0;i<in_pts.size();i++){
			out_pts[i]=in_pts[i] + curr_state.state;			
		}
	}
	void resetTrackerPosition(double pos_x, double pos_y){
		ssd->initState(STATE_TYPE(pos_x, pos_y));
	}
	void resetTrackerTemplate(IMAGE_TYPE *img, double pos_x, double pos_y,
		double size_x, double size_y){
			updateTemplate(img, pos_x, pos_y, size_x, size_y);
			ssd->setStepper(init_template);
			resetTrackerPosition(pos_x, pos_y);
	}
};
#endif