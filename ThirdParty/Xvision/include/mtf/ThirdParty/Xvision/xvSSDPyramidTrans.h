#ifndef MTF_XVSSD_PYRAMID_TRANS_
#define MTF_XVSSD_PYRAMID_TRANS_

#include "xvSSDMain.h"

class XVSSDPyramidTrans: public XVSSDMain {

    typedef XVPyramidStepper<XVTransStepper< IMAGE_TYPE > >  STEPPER_TYPE;
    typedef XVSSD< IMAGE_TYPE, STEPPER_TYPE > TRACKER_TYPE;
    typedef TRACKER_TYPE::SP STATE_PAIR_TYPE;
    typedef STEPPER_TYPE::STATE_TYPE STATE_TYPE;

public:

    TRACKER_TYPE *ssd;
	STEPPER_TYPE *stepper;
	STATE_PAIR_TYPE current_state;	

	int no_of_levels;
	double scale;


	XVSSDPyramidTrans(const ParamType *xv_params = NULL,
               int no_of_levels=2, double scale=0.5):
        XVSSDMain(xv_params) {
					  this->no_of_levels=no_of_levels;
					  this->scale=scale;
					  name="pyramid_trans";
		}

    void initTracker() {
        //printf("Using Pyramidal Trans SSD Tracker with:\n\t");
		printf("\tno_of_levels=%d\n\t",no_of_levels);
		printf("scale=%f\n\t",scale);

		stepper=new STEPPER_TYPE(init_template, scale, no_of_levels);
		ssd=new TRACKER_TYPE;
		ssd->setStepper(*(stepper));
		ssd->initState((STATE_TYPE)(*init_pos));
		template_img = ssd->getTarget();
    }

	double updateTrackerState() {
		current_state=ssd->step(*xv_frame);
		warped_img=ssd->warpedImage();
		return current_state.error;		
	}
	void updateCorners(){
		current_state=ssd->getCurrentState();
		for(int i=0; i<NCORNERS; ++i)
			corners[i] = points[i] + current_state.state;
	}

	void getTransformedPoints(vector<XVPositionD> &in_pts, vector<XVPositionD> &out_pts){
		for(int i=0;i<in_pts.size();i++){
			out_pts[i]=in_pts[i] + current_state.state;
		}
	}
	void resetTrackerPosition(double pos_x, double pos_y){
		ssd->initState(STATE_TYPE(pos_x, pos_y));
	}
	void resetTrackerTemplate(IMAGE_TYPE *img, double pos_x, double pos_y,
		double size_x, double size_y){
			updateTemplate(img, pos_x, pos_y, size_x, size_y);
			ssd->setStepper(STEPPER_TYPE(init_template, scale, no_of_levels));
			resetTrackerPosition(pos_x, pos_y);
	}
};
#endif