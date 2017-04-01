#ifndef _INC_XVSSDTR
#define _INC_XVSSDTR

#include "xvSSDMain.h"

class XVSSDTR: public XVSSDMain {

	typedef XVTransStepper< IMAGE_TYPE  > TRANS_STEPPER_TYPE;
	typedef XVRotateStepper< IMAGE_TYPE  > ROT_STEPPER_TYPE;

    typedef XVSSD< IMAGE_TYPE, TRANS_STEPPER_TYPE > TRANS_TRACKER_TYPE;
	typedef XVSSD< IMAGE_TYPE, ROT_STEPPER_TYPE > ROT_TRACKER_TYPE;

    typedef TRANS_TRACKER_TYPE::SP TRANS_STATE_PAIR_TYPE;
    typedef TRANS_STEPPER_TYPE::STATE_TYPE TRANS_STATE_TYPE;

	typedef ROT_TRACKER_TYPE::SP ROT_STATE_PAIR_TYPE;
	typedef ROT_STEPPER_TYPE::STATE_TYPE ROT_STATE_TYPE;


public:
    TRANS_TRACKER_TYPE *trans_ssd;
	ROT_TRACKER_TYPE *rot_ssd;

	TRANS_STATE_PAIR_TYPE trans_current_state;
	ROT_STATE_PAIR_TYPE rot_current_state;

    XVSSDTrans(bool show_xv_window,
		int steps_per_frame, bool copy_frame=true, bool refresh_win=true):
        XVSSDMain(show_xv_window, steps_per_frame,
			copy_frame, refresh_win){
				name="trans";
		}

    virtual void initTracker() {
        printf("Using Trans SSD Tracker with:\n\t");
		printf("size_x=%d\n\t",init_size->Width());
		printf("size_y=%d\n\t",init_size->Height());
        printf("steps_per_frame=%d\n",steps_per_frame);		

        IMAGE_TYPE init_tmpl = subimage(*xv_frame, *xv_roi);

        trans_ssd=new TRANS_TRACKER_TYPE;
        trans_ssd->setStepper(init_tmpl);
        trans_ssd->initState( (TRANS_STATE_TYPE)(*xv_roi) );

		ROT_STATE_TYPE state;
		state.trans=*xv_roi;
		state.angle=0.0;
		rot_ssd=new ROT_TRACKER_TYPE(state, init_tmpl);
    }

    virtual double updateTrackerState() {
		trans_current_state=trans_ssd->step(*xv_frame);
		rot_ssd->setState();
		rot_current_state=rot_ssd->step(*xv_frame);
		return trans_current_state.error;		
    }

	virtual void updateCorners(){
		for(int i=0; i<NCORNERS; ++i){
			corners[i] = points[i] + current_state.state;
		}
	}
	virtual void getTransformedPoints(vector<XVPositionD> &in_pts, vector<XVPositionD> &out_pts){
		for(int i=0;i<in_pts.size();i++){
			out_pts[i]=in_pts[i] + current_state.state;			
		}
	}
};
#endif