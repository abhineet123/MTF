#ifndef _INC_XVSSD_HELPER
#define _INC_XVSSD_HELPER

#include "xvSSDMain.h"

/************************** INIT FUNCTIONS **********************************************************************/

void XVSSDMain::getInitPoints() {
    init_pts.resize( no_of_pts );
    trans_pts.resize( no_of_pts );
    XVPositionD tmpPoint = XVPositionD( init_size->Width() / 2, init_size->Height() / 2 );
    int pt_idx = 0;
    for( int w = 0; w < init_size->Width(); w++ ) {
        for( int h = 0; h < init_size->Height(); h++ ) {
            double x = w - tmpPoint.PosX();
            double y = h - tmpPoint.PosY();
            init_pts[pt_idx] = XVPositionD( x, y );
            pt_idx++;
        }
    }
}

void XVSSDMain::getInitPixelValues() {
    //init_pixel_vals.resize(no_of_pts);
    init_pixel_vals_gs.resize( no_of_pts );
    int count = 0;
    for( int w = 0; w < init_size->Width(); w++ ) {
        for( int h = 0; h < init_size->Height(); h++ ) {
            //init_pixel_vals[count]=getPixelValue(w, h, &init_template);
            init_pixel_vals_gs[count] = getPixelValueGS( w, h, &init_template_gs );
            count++;
        }
    }
}
void XVSSDMain::getInitPixelValues2() {
    RGBtoScalar( *xv_frame, xv_frame_gs );
    //init_pixel_vals.resize(no_of_pts);
    init_pixel_vals_gs.resize( no_of_pts );
    for( int i = 0; i < no_of_pts; i++ ) {
        int x = init_pts[i].PosX() + init_pos->PosX();
        int y = init_pts[i].PosY() + init_pos->PosY();
        init_pixel_vals_gs[i] = getPixelValueGS( x, y, &xv_frame_gs );
    }
}


/************************** WRITING FUNCTIONS **********************************************************************/

inline void XVSSDMain::writeParamsBin() {
    float size_x = init_size->Width();
    float size_y = init_size->Height();
    float pos_x = init_pos->PosX();
    float pos_y = init_pos->PosY();

    char fname[FNAME_SIZE];
    snprintf( fname, FNAME_SIZE, "%s/params.bin", pts_dir );
    FILE *fp = fopen( fname, "wb" );

    printf( "Writing parameters to %s\n", fname );

    fwrite( &img_height, sizeof( int ), 1, fp );
    fwrite( &img_width, sizeof( int ), 1, fp );
    fwrite( &no_of_pixels, sizeof( int ), 1, fp );
    fwrite( &size_x, sizeof( float ), 1, fp );
    fwrite( &size_y, sizeof( float ), 1, fp );
    fwrite( &pos_x, sizeof( float ), 1, fp );
    fwrite( &pos_y, sizeof( float ), 1, fp );
    fwrite( &params.steps_per_frame, sizeof( int ), 1, fp );
    fwrite( &no_of_pts, sizeof( int ), 1, fp );
    fwrite( &no_of_vals, sizeof( int ), 1, fp );

    printf( "img_height=%d\n", img_height );
    printf( "img_width=%d\n", img_width );
    printf( "no_of_pixels=%d\n", no_of_pixels );
    printf( "size_x=%f\n", size_x );
    printf( "size_y=%f\n", size_y );
    printf( "pos_x=%f\n", pos_x );
    printf( "pos_y=%f\n", pos_y );
    printf( "no_of_pts=%d\n", no_of_pts );
    printf( "steps_per_frame=%d\n", params.steps_per_frame );
    printf( "no_of_vals=%d\n", no_of_vals );

    fclose( fp );
}

inline void XVSSDMain::writeInitData() {
    ofstream fout;
    char fname[FNAME_SIZE];
    snprintf( fname, FNAME_SIZE, "%s/init_pts.txt", pts_dir );
    fout.open( fname, ios::out );
    cout << "Writing init points to " << fname << "\n";
    for( int i = 0; i < init_pts.size(); i++ ) {
        fout << init_pts[i].PosX() << "\t" << init_pts[i].PosY() << "\n";
    }
    fout.close();

    snprintf( fname, FNAME_SIZE, "%s/init_pixel_vals_gs.txt", pts_dir );
    fout.open( fname, ios::out );
    cout << "Writing init pixel values to " << fname << "\n";
    for( int i = 0; i < init_pixel_vals_gs.size(); i++ ) {
        fout << init_pixel_vals_gs[i] << "\n";
    }
    fout.close();
}

inline void XVSSDMain::writeTransformedPoints() {
    char fname[FNAME_SIZE];
    snprintf( fname, FNAME_SIZE, "%s/current_pts_%d.txt", pts_dir, no_of_frames );
    ofstream fout;
    cout << "Writing transformed points to " << fname << "\n";
    fout.open( fname, ios::app );
    for( int i = 0; i < no_of_pts; i++ ) {
        fout << trans_pts[i].PosX() << ", " << trans_pts[i].PosY() << "\t";
    }
    fout << "\n";
    fout.close();
}

inline void XVSSDMain::writeTransformedPointsBin() {
    //printf("in writeTransformedPointsBin\n");
    char fname[FNAME_SIZE];
    snprintf( fname, FNAME_SIZE, "%s/frame_%d.bin", pts_dir, no_of_frames );
    //printf("Writing transformed points to %s\n",fname);
    FILE *fp = fopen( fname, "wb" );
    fwrite( current_pts, sizeof( double ), no_of_vals, fp );
    fclose( fp );
}

inline void XVSSDMain::writeCurrentFrameGS() {
    char fname[FNAME_SIZE];
    snprintf( fname, FNAME_SIZE, "%s/frame_%d_gs.txt", frame_dir, no_of_frames );
    //cout<<"Writing current frame to "<<fname<<"\n";
    ofstream fout( fname );
    RGBtoScalar( *xv_frame, xv_frame_gs );
    fout << xv_frame_gs;
    fout.close();
}

inline void XVSSDMain::writeCurrentFrameBin() {
    char fname[FNAME_SIZE];
    snprintf( fname, FNAME_SIZE, "%s/%d.bin", frame_dir, no_of_frames );
    //printf("Writing current frame to %s\n", fname);
    FILE *fp = fopen( fname, "wb" );
    fwrite( xv_frame->data(), sizeof( uchar ), no_of_pixels * xv_nch, fp );
    fclose( fp );
}

inline void XVSSDMain::writeCurrentFrameGSBin() {

    char fname[FNAME_SIZE];
    snprintf( fname, FNAME_SIZE, "%s/%d_gs.bin", frame_dir, no_of_frames );
    //printf("Writing current frame GS to %s\n", fname);
    RGBtoScalar( *xv_frame, xv_frame_gs );
    FILE *fp = fopen( fname, "wb" );
    fwrite( xv_frame_gs.data(), sizeof( PIX_TYPE_GS ), no_of_pixels, fp );
    fclose( fp );
}

/************************** PRINTING FUNCTIONS **********************************************************************/

void XVSSDMain::printSSDOfRegion( int tracker_id ) {
    ofstream fout;
    int no_of_frames = region_ssd_log.size();
    for( int frame = 0; frame <= no_of_frames; frame++ ) {
        char fname[FNAME_SIZE];
        snprintf( fname, FNAME_SIZE, "Region SSD/tracker_%d_%s_frame_%d.txt", tracker_id, name.c_str(), frame );
        printf( "Opening file: %s\n", fname );
        fout.open( fname, ios::out );
        int count = 0;
        for( int x = -region_thresh; x <= region_thresh; x++ ) {
            for( int y = -region_thresh; y <= region_thresh; y++ ) {
                fout << region_ssd_log[frame][count] << "\t";
                count++;
            }
            fout << "\n";
        }
        fout.close();
    }
}
void XVSSDMain::printSSDOfRegion( double *region_ssd ) {
    ofstream fout;
    char fname[FNAME_SIZE];
    snprintf( fname, FNAME_SIZE, "Region SSD/%s_%d__frame_%d.txt", name.c_str(), region_thresh, no_of_frames );
    printf( "Opening file: %s\n", fname );
    fout.open( fname, ios::out );
    int count = 0;
    for( int x = -region_thresh; x <= region_thresh; x++ ) {
        for( int y = -region_thresh; y <= region_thresh; y++ ) {
            fout << region_ssd[count] << "\t";
            count++;
        }
        fout << "\n";
    }
    fout.close();
}
void XVSSDMain::printSSSDLog( int tracker_id ) {
    ofstream ssd_out;
    char ssd_fname[FNAME_SIZE];
    snprintf( ssd_fname, FNAME_SIZE, "ssd_log_tracker_%d_%s.txt", tracker_id, name.c_str() );

    printf( "Opening file: %s\n", ssd_fname );
    ssd_out.open( ssd_fname, ios::out );
    /*ssd_out.setf(ios::center);
    ssd_out.width(12);*/
    for( int frame = 0; frame <= no_of_frames; frame++ ) {
        ssd_out << "frame_" << frame << "\t";
    }
    ssd_out << "\n";
    for( int step = 0; step < params.steps_per_frame; step++ ) {
        for( int frame = 0; frame < no_of_frames; frame++ ) {
            ssd_out << ssd_log[frame][step] << "\t";
        }
        ssd_out << "\n";
    }
    ssd_out.close();
}
void XVSSDMain::printErrorLog( int tracker_id ) {
    ofstream err_out;
    char error_fname[FNAME_SIZE];
    snprintf( error_fname, FNAME_SIZE, "error_log_tracker_%d_%s.txt", tracker_id, name.c_str() );

    printf( "Opening file: %s\n", error_fname );
    err_out.open( error_fname, ios::out );
    /*ssd_out.setf(ios::center);
    ssd_out.width(12);*/
    for( int frame = 0; frame <= no_of_frames; frame++ ) {
        err_out << "frame_" << frame << "\t";
    }
    err_out << "\n";
    for( int step = 0; step < params.steps_per_frame; step++ ) {
        for( int frame = 0; frame < no_of_frames; frame++ ) {
            err_out << error_log[frame][step] << "\t";
        }
        err_out << "\n";
    }

    err_out.close();
}



/************************** PIXEL FUNCTIONS **********************************************************************/

inline void XVSSDMain::getCurrentPixelValues() {
    for( int i = 0; i < no_of_pts; i++ ) {
        curr_pixel_vals_gs[i] = getPixelValueGS( trans_pts[i].PosX(), trans_pts[i].PosY(), &xv_frame_gs );
    }
}

inline int* XVSSDMain::getPixelValue( int x, int y, IMAGE_TYPE* img ) {

    uchar* xv_data = ( uchar* )img->data();
    int xv_location = x * xv_nch + y * xv_row_size;
    int* pixel_val = new int[NCHANNELS];
    for( int i = 0; i < NCHANNELS; i++ ) {
        pixel_val[i] = xv_data[xv_location + i];
    }
    return pixel_val;
}

inline double* XVSSDMain::getPixelValue( double x, double y, IMAGE_TYPE* img ) {
    int x1 = floor( x );
    int x2 = ceil( x );
    int y1 = floor( y );
    int y2 = ceil( y );

    int *p11 = getPixelValue( x1, y1, img );
    int *p12 = getPixelValue( x1, y2, img );
    int *p21 = getPixelValue( x2, y1, img );
    int *p22 = getPixelValue( x2, y2, img );

    double* pixel_val = new double[NCHANNELS];
    for( int i = 0; i < NCHANNELS; i++ ) {
        double p1 = ( x2 - x ) * p11[i] + ( x - x1 ) * p21[i];
        double p2 = ( x2 - x ) * p12[i] + ( x - x1 ) * p22[i];
        pixel_val[i] = ( y2 - y ) * p1 + ( y - y1 ) * p2;
    }
    /*printf("\n=================================================\n");
    printf("x=%f y=%f\n", x, y);
    printf("x1=%d y1=%d\n", x1, y1);
    printf("x2=%d y2=%d\n", x2, y2);
    printf("p11: [%d, %d, %d]\t", p11[0], p11[1], p11[2]);
    printf("p12: [%d, %d, %d]\n", p12[0], p12[1], p12[2]);
    printf("p21: [%d, %d, %d]\t", p21[0], p21[1], p21[2]);
    printf("p22: [%d, %d, %d]\n", p22[0], p22[1], p22[2]);
    printf("pixel_val: [%f, %f, %f]\n", pixel_val[0], pixel_val[1], pixel_val[2]);
    printf("=================================================\n");*/
    return pixel_val;
}

inline PIX_TYPE_GS XVSSDMain::getPixelValueGS( int x, int y, IMAGE_TYPE_GS* img ) {
    PIX_TYPE_GS* xv_data = ( PIX_TYPE_GS* )img->data();
    PIX_TYPE_GS pixel_val = xv_data[x + y * img->SizeX()];
    return pixel_val;
}
inline double XVSSDMain::getPixelValueGS( double x, double y, IMAGE_TYPE_GS* img ) {
    int x1 = floor( x );
    int x2 = ceil( x );
    int y1 = floor( y );
    int y2 = ceil( y );

    PIX_TYPE_GS p11 = getPixelValueGS( x1, y1, img );
    PIX_TYPE_GS p12 = getPixelValueGS( x1, y2, img );
    PIX_TYPE_GS p21 = getPixelValueGS( x2, y1, img );
    PIX_TYPE_GS p22 = getPixelValueGS( x2, y2, img );

    double p1 = ( x2 - x ) * p11 + ( x - x1 ) * p21;
    double p2 = ( x2 - x ) * p12 + ( x - x1 ) * p22;
    double pixel_val = ( y2 - y ) * p1 + ( y - y1 ) * p2;
    /*printf("\n=================================================\n");
    printf("x=%fl y=%f\n", x, y);
    printf("x1=%d y1=%d\n", x1, y1);
    printf("x2=%d y2=%d\n", x2, y2);
    printf("p11: [%d, %d, %d]\t", p11[0], p11[1], p11[2]);
    printf("p12: [%d, %d, %d]\n", p12[0], p12[1], p12[2]);
    printf("p21: [%d, %d, %d]\t", p21[0], p21[1], p21[2]);
    printf("p22: [%d, %d, %d]\n", p22[0], p22[1], p22[2]);
    printf("pixel_val: [%f, %f, %f]\n", pixel_val[0], pixel_val[1], pixel_val[2]);
    printf("=================================================\n");*/
    return pixel_val;
}


/************************** SSD FUNCTIONS **********************************************************************/


inline double XVSSDMain::getCurrentSSD() {
    double ssd = 0;
    for( int i = 0; i < no_of_pts; i++ ) {
        double *curr_pixel_val = getPixelValue( trans_pts[i].PosX(), trans_pts[i].PosY(), xv_frame );
        double pixel_ssd = 0;
        for( int ch = 0; ch < NCHANNELS; ch++ ) {
            double diff = ( double )init_pixel_vals[i][ch] - curr_pixel_val[ch];
            pixel_ssd += diff * diff;
        }
        pixel_ssd /= ( double )NCHANNELS;
        ssd += pixel_ssd;
    }
    return ssd;
}

inline double XVSSDMain::getCurrentSSDGS() {
    getTransformedPoints( init_pts, trans_pts );
    double ssd = 0;
    for( int i = 0; i < no_of_pts; i++ ) {
        double curr_pixel_val = getPixelValueGS( trans_pts[i].PosX(), trans_pts[i].PosY(), &xv_frame_gs );
        double diff = ( double )init_pixel_vals_gs[i] - curr_pixel_val;
        ssd += diff * diff;
    }
    return ssd;
}

inline double* XVSSDMain::getSSDOfRegion() {
    getTransformedPoints( init_pts, trans_pts );
    double *region_ssd = new double[region_size];
    int ssd_count = 0;
    for( int x = -region_thresh; x <= region_thresh; x++ ) {
        for( int y = -region_thresh; y <= region_thresh; y++ ) {
            double ssd = 0;
            for( int i = 0; i < no_of_pts; i++ ) {
                XVPositionD trans_pt = current_pts[i] + XVPositionD( x, y );
                //double curr_pixel_val=getPixelValueGS(trans_pt.PosX(), trans_pt.PosY(), &xv_frame_gs);
                double *curr_pixel_val = getPixelValue( trans_pt.PosX(), trans_pt.PosY(), xv_frame );
                double pixel_ssd = 0;
                for( int ch = 0; ch < NCHANNELS; ch++ ) {
                    double diff = ( double )init_pixel_vals[i][ch] - curr_pixel_val[ch];
                    pixel_ssd += diff * diff;
                }
                pixel_ssd /= ( double )NCHANNELS;
                ssd += pixel_ssd;
            }
            region_ssd[ssd_count] = ssd;
            ssd_count++;
        }
    }
    return region_ssd;
}

/************************************************************************************************/
#endif