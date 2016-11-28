#ifndef MTF_XVSSD_SCALING
#define MTF_XVSSD_SCALING

#include "xvSSDMain.h"

class XVSSDScaling: public XVSSDMain {

	typedef XVScalingStepper< IMAGE_TYPE  > STEPPER_TYPE;
    typedef XVSSD< IMAGE_TYPE, STEPPER_TYPE > TRACKER_TYPE;
    typedef TRACKER_TYPE::SP STATE_PAIR_TYPE;
    typedef STEPPER_TYPE::STATE_TYPE STATE_TYPE;

public:

    TRACKER_TYPE *ssd;
	STATE_PAIR_TYPE current_state;	

	XVSSDScaling(const ParamType *xv_params = NULL) :
        XVSSDMain(xv_params){
				name="scaling";
		}

    void initTracker() {
		STATE_TYPE state;
		state.trans=*init_pos;
		state.angle=0.0;
		state.scale=1.0;
		ssd=new TRACKER_TYPE(state, init_template);
		template_img = ssd->getTarget();
		current_state=ssd->getCurrentState();
		printf("current_state: %12.6f,  %12.6f,  %12.6f,  %12.6f\n", 
			current_state.state.trans.PosX(), current_state.state.trans.PosY(),
			current_state.state.scale, current_state.state.angle);
    }

	double updateTrackerState() {
		current_state=ssd->step(*xv_frame);
		printf("current_state: %12.6f,  %12.6f,  %12.6f,  %12.6f\n", 
			current_state.state.trans.PosX(), current_state.state.trans.PosY(),
			current_state.state.scale, current_state.state.angle);
		warped_img=ssd->warpedImage();
		return current_state.error;		
	}
	void updateCorners(){
		current_state=ssd->getCurrentState();
		XVAffineMatrix angleMat(-current_state.state.angle);
		XVAffineMatrix scaleMat( 1 / current_state.state.scale,
			1 / current_state.state.scale);
		XVAffineMatrix tformMat((XVMatrix)scaleMat * (XVMatrix)angleMat);
		for(int i=0; i<NCORNERS; ++i)
			corners[i] = (tformMat * points[i]) + current_state.state.trans;
	}
	void getTransformedPoints(vector<XVPositionD> &in_pts, vector<XVPositionD> &out_pts){
		XVAffineMatrix angleMat(-current_state.state.angle);
		XVAffineMatrix scaleMat( 1 / current_state.state.scale,
			1 / current_state.state.scale);
		XVAffineMatrix tformMat((XVMatrix)scaleMat * (XVMatrix)angleMat);
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