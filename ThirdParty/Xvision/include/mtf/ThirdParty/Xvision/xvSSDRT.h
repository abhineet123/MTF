#ifndef MTF_XVSSD_RT
#define MTF_XVSSD_RT

#include "xvSSDMain.h"

class XVSSDRT: public XVSSDMain {

	typedef XVRTStepper< IMAGE_TYPE  > STEPPER_TYPE;
    typedef XVSSD< IMAGE_TYPE, STEPPER_TYPE > TRACKER_TYPE;
    typedef TRACKER_TYPE::SP STATE_PAIR_TYPE;
    typedef STEPPER_TYPE::STATE_TYPE STATE_TYPE;

public:

    TRACKER_TYPE *ssd;
	STATE_PAIR_TYPE current_state;	

	XVSSDRT(const ParamType *xv_params = NULL) :
        XVSSDMain(xv_params){
				name="rt";
		}

    void initTracker() {
		STATE_TYPE state;
		state.trans = *init_pos;
		state.angle = 0.0;
		ssd=new TRACKER_TYPE(state, init_template);
		template_img = ssd->getTarget();
    }

	double updateTrackerState() {
		current_state=ssd->step(*xv_frame);
		warped_img=ssd->warpedImage();
		current_tracker_state->setState(current_state.state.trans.PosX(), 
			current_state.state.trans.PosY(), current_state.state.angle);
		return current_state.error;
	}

	void updateCorners(){
		current_state=ssd->getCurrentState();
		XVAffineMatrix angleMat(-current_state.state.angle);
		for(int i=0; i<NCORNERS; ++i)
			corners[i] = (angleMat * points[i]) + current_state.state.trans;
	}
	void getTransformedPoints(vector<XVPositionD> &in_pts, vector<XVPositionD> &out_pts){
		XVAffineMatrix angleMat(-current_state.state.angle);
		for(int i=0; i<in_pts.size(); ++i)
			out_pts[i] = (angleMat * in_pts[i]) + current_state.state.trans;
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