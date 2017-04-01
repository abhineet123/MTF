#ifndef MTF_XVEDGE
#define MTF_XVEDGE

#include "xvSSDMain.h"
#include <XVEdgeFeature.h>

class XVEdgeTracker: public XVSSDMain {

    typedef XVEdge<int> EDGE_TYPE;
	typedef XVImageScalar<u_short> IMAGE_OUT_TYPE;
	typedef XVEdgeFeature< int, EDGE_TYPE > TRACKER_TYPE;
	typedef XVLineState STATE_PAIR_TYPE;

public:
	EDGE_TYPE *edge;
    TRACKER_TYPE *edge_tracker;
	STATE_PAIR_TYPE current_state;	
	STATE_PAIR_TYPE state_change;	

	int search_width;
	int search_angle;
	int line_width;

	XVEdgeTracker(const ParamType *xv_params = NULL,
			int search_width=50, int search_angle=8, int line_width=10):
        XVSSDMain(xv_params) {
        name="edge";
		this->search_width=search_width;
		this->search_angle=search_angle;
		this->line_width=line_width;
    }

    virtual void initTracker() {
		printf("\tsearch_width=%d\n\t",search_width);
		printf("search_angle=%d\n\t",search_angle);
		printf("line_width=%d\n\t",line_width);

		edge=new EDGE_TYPE();
        edge_tracker=new TRACKER_TYPE(*edge, search_width, search_angle, line_width);
		XVPosition ends[2];
		ends[0].setX(init_pos->PosX()-init_size->Width()/2.0);
		ends[0].setY(init_pos->PosY()-init_size->Height()/2.0);
		ends[1].setX(init_pos->PosX()+init_size->Width()/2.0);
		ends[1].setY(init_pos->PosY()-init_size->Height()/2.0);
        edge_tracker->init(ends);
		printf("start_pt = (%d, %d)\n\t",ends[0].PosX(), ends[0].PosY());
		printf("end_pt = (%d, %d)\n",ends[1].PosX(), ends[1].PosY());
    }

	virtual double updateTrackerState() {
		current_state=edge_tracker->step(*xv_frame);
		state_change = edge_tracker->diffState();
		return current_state.error;		
	}

	virtual void updateCorners(){
		XV2Vec<double> ends[2];
		current_state.state.endpoints(ends);
		XV2Vec<double> diffTop = ends[1] - ends[0];
		double val = (state_change.state.center * diffTop) / diffTop.length();
		//printf("val: %f\n", val);
		corners[0].reposition(ends[0].PosX(), ends[0].PosY());
		corners[1].reposition(ends[1].PosX(), ends[1].PosY());
		corners[2].reposition(ends[0].PosX(), ends[0].PosY());
		corners[3].reposition(ends[1].PosX(), ends[1].PosY());
	}
	virtual void getTransformedPoints(vector<XVPositionD> &in_pts, vector<XVPositionD> &out_pts){
	}
	virtual void resetTrackerPosition(double pos_x, double pos_y){}
	virtual void resetTrackerTemplate(IMAGE_TYPE *img, double pos_x, double pos_y,
		double size_x, double size_y){}
};
#endif