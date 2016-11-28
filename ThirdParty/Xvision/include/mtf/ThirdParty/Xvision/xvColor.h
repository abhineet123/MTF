#ifndef MTF_XVCOLOR
#define MTF_XVCOLOR

#include "xvSSDMain.h"
#include <XVBlobFeature.h>
#include <XVColorSeg.h>

class XVColor: public XVSSDMain {

    typedef XVImageScalar<u_short> IMAGE_OUT_TYPE;
    typedef XVBlobFeature< IMAGE_TYPE, IMAGE_OUT_TYPE > TRACKER_TYPE;
    typedef XVHueRangeSeg<PIX_TYPE, u_short> SEG_TYPE;
    typedef XVBlobState STATE_PAIR_TYPE;

public:

    TRACKER_TYPE *blob;
    SEG_TYPE *seg;
	STATE_PAIR_TYPE current_state;	

	bool color_resample;

	XVColor(const ParamType *xv_params = NULL,
			bool color_resample=false):
        XVSSDMain(xv_params) {
        name="color";
		this->color_resample=color_resample;
    }

    void initTracker() {
        seg=new SEG_TYPE;
        blob=new TRACKER_TYPE(*seg, color_resample);
		XVROI roi( *init_size, *init_pos_min );
        blob->init(roi, *xv_frame);
    }

	double updateTrackerState() {
		current_state=blob->step(*xv_frame);
		return current_state.error;		
	}

	void updateCorners(){
		int min_x=current_state.state.PosX();
		int max_x=current_state.state.PosX()+current_state.state.Width();
		int min_y=current_state.state.PosY();
		int max_y=current_state.state.PosY()+current_state.state.Height();

		corners[0].reposition(min_x, min_y);
		corners[1].reposition(max_x, min_y);
		corners[2].reposition(max_x, max_y);
		corners[3].reposition(min_x, max_y);  
	}
	void getTransformedPoints(vector<XVPositionD> &in_pts, vector<XVPositionD> &out_pts){
	}
	void resetTrackerPosition(double pos_x, double pos_y){}
	void resetTrackerTemplate(IMAGE_TYPE *img, double pos_x, double pos_y,
		double size_x, double size_y){}
};
#endif