#ifndef MTF_XVSSD_AFFINE_
#define MTF_XVSSD_AFFINE_

#include "xvSSDMain.h"

class XVSSDAffine: public XVSSDMain {

	typedef XVAffineStepper< IMAGE_TYPE  > STEPPER_TYPE;
    typedef XVSSD< IMAGE_TYPE, STEPPER_TYPE > TRACKER_TYPE;
    typedef TRACKER_TYPE::SP STATE_PAIR_TYPE;
    typedef STEPPER_TYPE::STATE_TYPE STATE_TYPE;

public:

    TRACKER_TYPE *ssd;
	STATE_PAIR_TYPE current_state;	

	XVSSDAffine(const ParamType *xv_params = NULL) :
        XVSSDMain(xv_params){
				name="affine";
		}

    void initTracker() {
        //printf("Using Affine SSD Tracker with:\n\t");
		
		STATE_TYPE state;
		state.trans=*init_pos;
		state.a=1.0;
		state.b=0.0;
		state.c=0.0;
		state.d=1.0;
		ssd=new TRACKER_TYPE(state, init_template);
		template_img = ssd->getTarget();
		current_state=ssd->getCurrentState();
		printf("current_state: tx: %12.6f,  ty: %12.6f,  a: %12.6f,  b: %12.6f  c: %12.6f,  d: %12.6f\n", 
			current_state.state.trans.PosX(), current_state.state.trans.PosY(),
			current_state.state.a, current_state.state.b, 
			current_state.state.c, current_state.state.d);
    }

	double updateTrackerState() {
		current_state=ssd->step(*xv_frame);
		printf("current_state: tx: %12.6f,  ty: %12.6f,  a: %12.6f,  b: %12.6f  c: %12.6f,  d: %12.6f\n", 
			current_state.state.trans.PosX(), current_state.state.trans.PosY(),
			current_state.state.a, current_state.state.b, 
			current_state.state.c, current_state.state.d);

		warped_img=ssd->warpedImage();
		current_tracker_state->setState(current_state.state.a, 
			current_state.state.b, current_state.state.c, current_state.state.d);
		return current_state.error;		
	}
	void updateCorners(){
		current_state=ssd->getCurrentState();
		XVAffineMatrix tformMat(current_state.state.a, current_state.state.b, 
			current_state.state.c, current_state.state.d);
		for(int i=0; i<NCORNERS; ++i)
			corners[i] = (tformMat * points[i]) + current_state.state.trans;
	}
	void getTransformedPoints(vector<XVPositionD> &in_pts, vector<XVPositionD> &out_pts){
		XVAffineMatrix tformMat(current_state.state.a, current_state.state.b, 
			current_state.state.c, current_state.state.d);
		for(int i=0; i<NCORNERS; ++i)
			out_pts[i] = (tformMat * in_pts[i]) + current_state.state.trans;
	}

	void resetTrackerPosition(double pos_x, double pos_y){
		STATE_TYPE temp_state(current_state.state);
		temp_state.trans.setX(pos_x);
		temp_state.trans.setY(pos_y);
		ssd->initState(temp_state);
	}
	void resetTrackerTemplate(IMAGE_TYPE *img, double pos_x, double pos_y,
		double size_x, double size_y){
			updateTemplate(img, pos_x, pos_y, size_x, size_y);
			ssd->setStepper(init_template);
			resetTrackerPosition(pos_x, pos_y);
	}
};
#endif