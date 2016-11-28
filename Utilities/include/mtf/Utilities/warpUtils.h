#ifndef MTF_WARP_UTILS_H
#define MTF_WARP_UTILS_H

#include "mtf/Macros/common.h"

_MTF_BEGIN_NAMESPACE

namespace utils{
	template<typename T1, typename T2>
	inline void homogenize(const T1 &dehom_mat, T2 &hom_mat){
		assert(hom_mat.cols() == dehom_mat.cols());
		hom_mat.topRows(2) = dehom_mat;
		hom_mat.row(2).fill(1);
	}

	template<typename T1, typename T2>
	inline void dehomogenize(const T1 &hom_mat, T2 &dehom_mat){
		assert(hom_mat.cols() == dehom_mat.cols());
		dehom_mat = hom_mat.topRows(2);
		dehom_mat = dehom_mat.array().rowwise() / hom_mat.array().row(2);
	}

	// returning versions
	inline Matrix3Xd homogenize(const Matrix2Xd &dehom_mat){
		Matrix3Xd hom_mat;
		hom_mat.resize(NoChange, dehom_mat.cols());
		homogenize(dehom_mat, hom_mat);
		return hom_mat;
	}
	inline Matrix2Xd dehomogenize(const Matrix3Xd &hom_mat){
		Matrix2Xd dehom_mat;
		dehom_mat.resize(NoChange, hom_mat.cols());
		dehomogenize(hom_mat, dehom_mat);
		return dehom_mat;
	}
	inline Matrix3d getTranslationMatrix(double tx, double ty){
		Matrix3d trans_mat = Matrix3d::Identity();
		trans_mat(0, 2) = tx;
		trans_mat(1, 2) = ty;
		return trans_mat;
	}
	inline Matrix3d getRotationMatrix(double theta){
		Matrix3d rot_mat = Matrix3d::Identity();
		double cos_theta = cos(theta);
		double sin_theta = sin(theta);
		rot_mat(0, 0) = cos_theta;
		rot_mat(0, 1) = -sin_theta;
		rot_mat(1, 0) = sin_theta;
		rot_mat(1, 1) = cos_theta;
		return rot_mat;
	}
	inline Matrix3d getScalingMatrix(double s){
		Matrix3d scale_mat = Matrix3d::Identity();
		scale_mat(0, 0) += s;
		scale_mat(1, 1) += s;
		return scale_mat;
	}
	inline Matrix3d getShearingMatrix(double a, double b){
		Matrix3d shear_mat = Matrix3d::Identity();
		shear_mat(0, 0) += a;
		shear_mat(0, 1) = b;
		return shear_mat;
	}

	//! Variants of Direct Linear Transformation (DLT) algorithm to estimate best fit parameters 
	//! for different SSMs from pairs of corresponding points and/or corners
	Matrix3d computeHomographyDLT(const Matrix2Xd &in_pts, const Matrix2Xd &out_pts, int n_pix);
	Matrix3d computeHomographyDLT(const Matrix24d &in_corners, const Matrix24d &out_corners);
	Matrix3d computeHomographyNDLT(const Matrix24d &in_corners, const Matrix24d &out_corners);
	Matrix3d computeAffineDLT(const Matrix24d &in_corners, const Matrix24d &out_corners);
	Matrix3d computeAffineDLT(const Matrix2Xd &in_pts, const Matrix2Xd &out_pts);
	Matrix3d computeAffineDLT(const Matrix23d &in_pts, const Matrix23d &out_pts);
	Matrix3d computeAffineNDLT(const Matrix24d &in_corners, const Matrix24d &out_corners);
	Matrix3d computeSimilitudeDLT(const Matrix24d &in_corners, const Matrix24d &out_corners);
	Matrix3d computeSimilitudeNDLT(const Matrix24d &in_corners, const Matrix24d &out_corners);
	Matrix3d computeSimilitudeDLT(const Matrix2Xd &in_pts, const Matrix2Xd &out_pts);
	Matrix3d computeSimilitudeNDLT(const Matrix2Xd &in_pts, const Matrix2Xd &out_pts);
	Vector3d computeIsometryDLT(const Matrix2Xd &in_pts, const Matrix2Xd &out_pts);
	Matrix3d computeTranscalingDLT(const Matrix24d &in_corners, const Matrix24d &out_corners);
	Matrix3d computeTranscalingDLT(const Matrix2Xd &in_corners, const Matrix2Xd &out_corners);

	void decomposeHomographyForward(Matrix3d &affine_mat, Matrix3d &proj_mat, const Matrix3d &hom_mat);
	void decomposeHomographyInverse(Matrix3d &affine_mat, Matrix3d &proj_mat, const Matrix3d &hom_mat);
	void decomposeAffineForward(Vector6d &affine_params, const Matrix3d &affine_mat);
	void decomposeAffineInverse(Vector6d &affine_params, const Matrix3d &affine_mat);
	void decomposeAffineInverse(Matrix3d &trans_mat, Matrix3d &rot_mat,
		Matrix3d &scale_mat, Matrix3d &shear_mat, const Matrix3d &affine_mat);
	void decomposeAffineForward(Matrix3d &trans_mat, Matrix3d &rot_mat,
		Matrix3d &scale_mat, Matrix3d &shear_mat, const Matrix3d &affine_mat);


	/**
	normalizes the given points so that their mean (centroid) moves to origin
	and their mean distance from origin becomes unity; also returns the inverse normalization
	matrix which, when multiplied to the normalized points will give the original points back
	*/
	void normalizePts(Matrix24d &norm_pts, Matrix3d &inv_norm_mat, const Matrix24d &pts);
	void normalizePts(Matrix2Xd &norm_pts, Matrix3d &inv_norm_mat, const Matrix2Xd &pts);
		/**
	computes the 2D coordinates for an equally spaced grid of points that covers a
	square centered at the origin and lying between +c and -c in both x and y directions
	*/
	void getNormUnitSquarePts(Matrix2Xd &std_grid, Matrix24d &basis_corners,
		int resx, int resy, double min_x = -0.5, double min_y = -0.5,
		double max_x = 0.5, double max_y = 0.5);
	void getPtsFromCorners(PtsT &pts, const CornersT &corners,
		int resx, int resy);
	//! overload for OpenCV corners
	PtsT getPtsFromCorners(const cv::Mat &corners_cv,int resx, int resy);
	void getPtsFromCorners(PtsT &pts, const CornersT &corners,
		const PtsT basis_pts, const CornersT &basis_corners);

	/**
	extract points along the boundary of the given region
	represented by a grid of points with the given sampling resolution
	and arranged in row major order
	*/
	void getBoundingPts(cv::Mat &bounding_pts, const PtsT &grid_pts, int res_x, int res_y);
	/**
	tests if the given point is inside the given region - specified by the vertices of the corresponding polygon;
	undefined for points on the edges
	*/
	bool isInsideRegion(const cv::Mat &verices, double testx, double testy);
	//! returns the four nearest grid points around the given point that can be used for bilinear interpolation
	void getBilinearPts(cv::Vec4i &neigh_pts_id, std::vector<cv::Vec2d> &neigh_pts_dist,
		double x, double y, const mtf::PtsT &grid_pts, int n_pts);
	//! returns the id of the grid point nearest to the given point
	double getNearestPt(double x, double y, const mtf::PtsT &grid_pts, int n_pts);

	MatrixX2d computeTPS(const Matrix24d &in_corners, const Matrix24d &out_corners);
	void applyTPS(PtsT &out_pts, const PtsT &in_pts,
		const PtsT &control_pts, const MatrixX2d &tps_params);
	inline double tps(double r){ return r*r*log(r); }

	//void getSupportPoints(ProjWarpT &warp, PtsT &pts, HomPtsT &pts_hm, 
	//	const CornersT &corners,	const CornersT &basis_corners, 
	//	const HomPtsT &basis_pts_hm);	
}
_MTF_END_NAMESPACE
#endif
