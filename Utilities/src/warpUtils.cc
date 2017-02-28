#include "mtf/Utilities//warpUtils.h"
#include "mtf/Utilities/miscUtils.h"

#include <iostream>

_MTF_BEGIN_NAMESPACE

namespace utils{
	// specialization for dehomogenous matrix with 3 rows
	template<>
	inline void dehomogenize<HomPtsT, HomPtsT>(const HomPtsT &hom_mat, HomPtsT &dehom_mat){
		assert(hom_mat.cols() == dehom_mat.cols());
		dehom_mat = hom_mat.array().rowwise() / hom_mat.array().row(2);
	}
	void getNormUnitSquarePts(PtsT &basis_pts, CornersT &basis_corners,
		int resx, int resy, double min_x, double min_y,
		double max_x, double max_y){
		assert(basis_pts.cols() == resx*resy);

		VectorXd x_vals = VectorXd::LinSpaced(resx, min_x, max_x);
		VectorXd y_vals = VectorXd::LinSpaced(resy, min_y, max_y);

		int pt_id = 0;
		for(int row_id = 0; row_id < resy; row_id++){
			for(int col_id = 0; col_id < resx; col_id++){
				basis_pts(0, pt_id) = x_vals(col_id);
				basis_pts(1, pt_id) = y_vals(row_id);
				++pt_id;
			}
		}
		basis_corners.row(0) << min_x, max_x, max_x, min_x;
		basis_corners.row(1) << min_y, min_y, max_y, max_y;
	}
	void getPtsFromCorners(PtsT &pts, const CornersT &corners,
		int resx, int resy){
		assert(pts.cols() == resx*resy);
		CornersT basis_corners;
		PtsT basis_pts;
		basis_pts.resize(Eigen::NoChange, resx*resy);
		getNormUnitSquarePts(basis_pts, basis_corners, resx, resy);
		getPtsFromCorners(pts, corners, basis_pts, basis_corners);
	}
	//! returning variant
	PtsT getPtsFromCorners(const CornersT &corners_eig, int resx, int resy){
		PtsT pts;
		pts.resize(Eigen::NoChange, resx*resy);
		getPtsFromCorners(pts, corners_eig, resx, resy);
		return pts;
	}
	//! overload for OpenCV corners
	PtsT getPtsFromCorners(const cv::Mat &corners_cv, int resx, int resy){
		assert(corners_cv.rows == 2 && corners_cv.cols == 4);
		return getPtsFromCorners(Corners(corners_cv).eig(), resx, resy);
	}
	//! overload for precomputed basis corners and points
	void getPtsFromCorners(PtsT &pts, const CornersT &corners,
		const PtsT basis_pts, const CornersT &basis_corners){
		assert(pts.cols() == basis_pts.cols());
		dehomogenize(computeHomographyDLT(basis_corners, corners)*homogenize(basis_pts), pts);
	}

	void getBoundingPts(cv::Mat &bounding_pts, const PtsT &grid_pts,
		int res_x, int res_y){
		assert(grid_pts.cols() == res_x*res_y);
		assert(bounding_pts.cols == 2 * (res_x + res_y - 2));

		int row_id = 0, col_id = 0, bounding_id = 0;
		while(col_id < res_x){
			int pt_id = row_id*res_x + col_id;
			bounding_pts.at<double>(0, bounding_id) = grid_pts(0, pt_id);
			bounding_pts.at<double>(1, bounding_id) = grid_pts(1, pt_id);
			++col_id;
			++bounding_id;
		}
		--col_id;
		while(row_id < res_y){
			int pt_id = row_id*res_x + col_id;
			bounding_pts.at<double>(0, bounding_id) = grid_pts(0, pt_id);
			bounding_pts.at<double>(1, bounding_id) = grid_pts(1, pt_id);
			++row_id;
			++bounding_id;
		}
		--row_id;
		while(col_id >= 0){
			int pt_id = row_id*res_x + col_id;
			bounding_pts.at<double>(0, bounding_id) = grid_pts(0, pt_id);
			bounding_pts.at<double>(1, bounding_id) = grid_pts(1, pt_id);
			--col_id;
			++bounding_id;
		}
		++col_id;
		while(row_id > 0){
			int pt_id = row_id*res_x + col_id;
			bounding_pts.at<double>(0, bounding_id) = grid_pts(0, pt_id);
			bounding_pts.at<double>(1, bounding_id) = grid_pts(1, pt_id);
			--row_id;
			++bounding_id;
		}
	}


	//! tests if the given point is inside the given polygon - undefined for points on the edges
	bool isInsideRegion(const cv::Mat &verices, double testx, double testy){
		int n_vert = verices.cols;
		int i, j, c = 0;
		for(i = 0, j = n_vert - 1; i < n_vert; j = i++) {
			if(((verices.at<double>(1, i)>testy) != (verices.at<double>(1, j) > testy)) &&
				(testx < (verices.at<double>(0, j) - verices.at<double>(0, i)) *
				(testy - verices.at<double>(1, i)) / (verices.at<double>(1, j) - verices.at<double>(1, i)) + verices.at<double>(0, i)))
				c = !c;
		}
		return c;
	}
	//! returns the four nearest grid points around the given point that can be used for bilinear interpolation
	void getBilinearPts(cv::Vec4i &neigh_pts_id, std::vector<cv::Vec2d> &neigh_pts_dist,
		double x, double y, const mtf::PtsT &grid_pts, int n_pts){

		double inf = std::numeric_limits<double>::infinity();
		double ul_dist = inf, ur_dist = inf, lr_dist = inf, ll_dist = inf;

		neigh_pts_id[0] = neigh_pts_id[1] = neigh_pts_id[2] = neigh_pts_id[3] = -1;

		for(int pt_id = 0; pt_id < n_pts; ++pt_id){
			double curr_x = grid_pts(0, pt_id), curr_y = grid_pts(1, pt_id);
			double dx = fabs(curr_x - x), dy = fabs(curr_y - y);
			double curr_dist = dx*dx + dy*dy;
			if(curr_x <= x && curr_y <= y && curr_dist < ul_dist){
				ul_dist = curr_dist;
				neigh_pts_id[0] = pt_id;
				neigh_pts_dist[0][0] = dx;
				neigh_pts_dist[0][1] = dy;
			}
			if(curr_x >= x && curr_y <= y && curr_dist < ur_dist){
				ur_dist = curr_dist;
				neigh_pts_id[1] = pt_id;
				neigh_pts_dist[1][0] = dx;
				neigh_pts_dist[1][1] = dy;
			}
			if(curr_x >= x && curr_y >= y && curr_dist < lr_dist){
				lr_dist = curr_dist;
				neigh_pts_id[2] = pt_id;
				neigh_pts_dist[2][0] = dx;
				neigh_pts_dist[2][1] = dy;
			}
			if(curr_x <= x && curr_y >= y && curr_dist < ll_dist){
				ll_dist = curr_dist;
				neigh_pts_id[3] = pt_id;
				neigh_pts_dist[3][0] = dx;
				neigh_pts_dist[3][1] = dy;
			}
		}
	}

	//! returns the id of the grid point nearest to the given point
	double getNearestPt(double x, double y, const mtf::PtsT &grid_pts, int n_pts){
		double min_dist = std::numeric_limits<double>::infinity();
		int nearest_id = 0;
		for(int pt_id = 0; pt_id < n_pts; ++pt_id){
			double curr_x = grid_pts(0, pt_id), curr_y = grid_pts(1, pt_id);
			double dx = fabs(curr_x - x), dy = fabs(curr_y - y);
			double curr_dist = dx*dx + dy*dy;
			if(curr_dist < min_dist){
				min_dist = curr_dist;
				nearest_id = pt_id;
			}
		}
		return nearest_id;
	}

	ProjWarpT computeHomographyDLT(const CornersT &in_corners, const CornersT &out_corners){
		// special function for computing homography between two sets of corners
		Matrix89d constraint_matrix;
		for(int i = 0; i < 4; i++){
			int r1 = 2 * i;
			constraint_matrix(r1, 0) = 0;
			constraint_matrix(r1, 1) = 0;
			constraint_matrix(r1, 2) = 0;
			constraint_matrix(r1, 3) = -in_corners(0, i);
			constraint_matrix(r1, 4) = -in_corners(1, i);
			constraint_matrix(r1, 5) = -1;
			constraint_matrix(r1, 6) = out_corners(1, i) * in_corners(0, i);
			constraint_matrix(r1, 7) = out_corners(1, i) * in_corners(1, i);
			constraint_matrix(r1, 8) = out_corners(1, i);

			int r2 = 2 * i + 1;
			constraint_matrix(r2, 0) = in_corners(0, i);
			constraint_matrix(r2, 1) = in_corners(1, i);
			constraint_matrix(r2, 2) = 1;
			constraint_matrix(r2, 3) = 0;
			constraint_matrix(r2, 4) = 0;
			constraint_matrix(r2, 5) = 0;
			constraint_matrix(r2, 6) = -out_corners(0, i) * in_corners(0, i);
			constraint_matrix(r2, 7) = -out_corners(0, i) * in_corners(1, i);
			constraint_matrix(r2, 8) = -out_corners(0, i);
		}
		JacobiSVD<Matrix89d> svd(constraint_matrix, ComputeFullU | ComputeFullV);
		VectorXd h = svd.matrixV().col(8);

		//MatrixXd U = svd.matrixU();
		//MatrixXd V = svd.matrixV();
		//MatrixXd S = svd.singularValues();

		//cout<<"svd.U\n"<<U<<"\n";
		//cout<<"svd.S:\n"<<S<<"\n";
		//cout<<"svd.V:\n"<<V<<"\n";
		//cout<<"h:\n"<<h<<"\n";
		//cout<<"constraint_matrix:\n"<<constraint_matrix<<"\n";

		//Matrix89d rec_mat = U * S.asDiagonal() * V.leftcols(8).transpose();
		//Matrix89d err_mat = rec_mat - constraint_matrix;
		//double svd_error = err_mat.squaredNorm();
		//cout<<"rec_mat:\n"<<rec_mat<<"\n";
		//cout<<"err_mat:\n"<<err_mat<<"\n";
		//cout<<"svd_error:"<<svd_error<<"\n";

		ProjWarpT hom_mat;
		hom_mat << h(0), h(1), h(2),
			h(3), h(4), h(5),
			h(6), h(7), h(8);
		hom_mat /= h(8);
		return hom_mat;
	}

	ProjWarpT computeHomographyDLT(const PtsT &in_pts, const PtsT &out_pts, int n_pts){
		//general function for computing homography between two sets of points
		assert(in_pts.cols() == n_pts);
		assert(out_pts.cols() == n_pts);

		MatrixXd constraint_matrix(2 * n_pts, 9);

		for(int i = 0; i < n_pts; i++){
			int r1 = 2 * i;
			constraint_matrix(r1, 0) = 0;
			constraint_matrix(r1, 1) = 0;
			constraint_matrix(r1, 2) = 0;
			constraint_matrix(r1, 3) = -in_pts(0, i);
			constraint_matrix(r1, 4) = -in_pts(1, i);
			constraint_matrix(r1, 5) = -1;
			constraint_matrix(r1, 6) = out_pts(1, i) * in_pts(0, i);
			constraint_matrix(r1, 7) = out_pts(1, i) * in_pts(1, i);
			constraint_matrix(r1, 8) = out_pts(1, i);

			int r2 = 2 * i + 1;
			constraint_matrix(r2, 0) = in_pts(0, i);
			constraint_matrix(r2, 1) = in_pts(1, i);
			constraint_matrix(r2, 2) = 1;
			constraint_matrix(r2, 3) = 0;
			constraint_matrix(r2, 4) = 0;
			constraint_matrix(r2, 5) = 0;
			constraint_matrix(r2, 6) = -out_pts(0, i) * in_pts(0, i);
			constraint_matrix(r2, 7) = -out_pts(0, i) * in_pts(1, i);
			constraint_matrix(r2, 8) = -out_pts(0, i);
		}
		JacobiSVD<MatrixXd> svd(constraint_matrix, ComputeThinU | ComputeThinV);
		int n_cols = svd.matrixV().cols();
		VectorXd h = svd.matrixV().col(n_cols - 1);
		ProjWarpT hom_mat;
		hom_mat << h(0), h(1), h(2),
			h(3), h(4), h(5),
			h(6), h(7), h(8);
		hom_mat /= h(8);

		return hom_mat;
	}
	// normalizes the target corners before computing the homography; 
	// supposed to be more numerically stable than the standard non-normalized version;
	ProjWarpT computeHomographyNDLT(const CornersT &in_corners, const CornersT &out_corners){
		CornersT norm_corners;
		ProjWarpT inv_norm_mat;
		normalizePts(norm_corners, inv_norm_mat, out_corners);
		ProjWarpT init_warp = computeHomographyDLT(in_corners, norm_corners);
		init_warp = inv_norm_mat * init_warp;
		return init_warp;
	}
	ProjWarpT computeAffineDLT(const CornersT &in_corners, const CornersT &out_corners){
		Matrix86d A;
		A.fill(0);
		Vector8d pt_vec;
		//! flattened version of the 2x4 matrix of target corner points
		Map<const Vector8d> out_corners_vec(out_corners.data());

		//utils::printMatrix(in_corners, "in_corners");
		//utils::printMatrix(out_corners_vec, "out_corners_vec");
		//utils::printMatrix(out_corners, "out_corners");


		for(int i = 0; i < 4; i++){
			int r1 = 2 * i;
			pt_vec(r1) = out_corners(0, i);
			A(r1, 0) = in_corners(0, i);
			A(r1, 1) = in_corners(1, i);
			A(r1, 2) = 1;

			int r2 = 2 * i + 1;
			pt_vec(r2) = out_corners(1, i);
			A(r2, 3) = in_corners(0, i);
			A(r2, 4) = in_corners(1, i);
			A(r2, 5) = 1;
		}
		//utils::printMatrix(pt_vec, "pt_vec");
		//utils::printMatrix(A, "A");

		JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
		Matrix86d U = svd.matrixU();
		Matrix66d V = svd.matrixV();
		Vector6d S = svd.singularValues();

		//utils::printMatrix(U, "U");
		//utils::printMatrix(V, "V");
		//utils::printMatrix(S, "S");
		//utils::printMatrixToFile(in_corners, "in_corners", "./log/mtf_log.txt");
		//utils::printMatrixToFile(out_corners, "out_corners", "./log/mtf_log.txt");
		//utils::printMatrixToFile(A, "A", "./log/mtf_log.txt");
		//MatrixXd rec_A = U * S.asDiagonal() * V.transpose();
		//MatrixXd rec_err_mat = rec_A - A;
		//utils::printMatrixToFile(U, "U", "./log/mtf_log.txt");
		//utils::printMatrixToFile(V, "V", "./log/mtf_log.txt");
		//utils::printMatrixToFile(S, "S", "./log/mtf_log.txt");
		//utils::printMatrixToFile(rec_A, "rec_A", "./log/mtf_log.txt");
		//utils::printMatrixToFile(rec_err_mat, "rec_err_mat", "./log/mtf_log.txt");
		//utils::printMatrixToFile(pt_vec, "b", "./log/mtf_log.txt");

		//VectorXd b2 = U.transpose() * pt_vec;
		//VectorXd y = b2.array() / S.array();
		//VectorXd x2 = V * y;

		Vector6d x = V * ((U.transpose() * out_corners_vec).array() / S.array()).matrix();

		//utils::printMatrix(x2, "x2");
		//utils::printMatrix(x, "x");

		ProjWarpT affine_mat = ProjWarpT::Zero();
		affine_mat(0, 0) = x(0);
		affine_mat(0, 1) = x(1);
		affine_mat(0, 2) = x(2);
		affine_mat(1, 0) = x(3);
		affine_mat(1, 1) = x(4);
		affine_mat(1, 2) = x(5);
		affine_mat(2, 2) = 1;
		return affine_mat;
	}

	ProjWarpT computeAffineDLT(const PtsT &in_pts, const PtsT &out_pts){
		assert(in_pts.cols() == out_pts.cols());
		int n_pts = in_pts.cols();
		MatrixXd A(2 * n_pts, 6);
		A.fill(0);
		//! flattened version of the 2x4 matrix of target corner points
		Map<const VectorXd> out_pts_vec(out_pts.data(), n_pts*2);

		for(int pt_id = 0; pt_id < n_pts; pt_id++){
			int r1 = 2 * pt_id;
			A(r1, 0) = in_pts(0, pt_id);
			A(r1, 1) = in_pts(1, pt_id);
			A(r1, 2) = 1;

			int r2 = 2 * pt_id + 1;
			A(r2, 3) = in_pts(0, pt_id);
			A(r2, 4) = in_pts(1, pt_id);
			A(r2, 5) = 1;
		}

		JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);	
		VectorXd x = svd.matrixV() * ((svd.matrixU().transpose() * out_pts_vec).array() / 
			svd.singularValues().array()).matrix();

		ProjWarpT affine_mat = ProjWarpT::Zero();
		affine_mat(0, 0) = x(0);
		affine_mat(0, 1) = x(1);
		affine_mat(0, 2) = x(2);
		affine_mat(1, 0) = x(3);
		affine_mat(1, 1) = x(4);
		affine_mat(1, 2) = x(5);
		affine_mat(2, 2) = 1;
		return affine_mat;
	}

	ProjWarpT computeAffineNDLT(const CornersT &in_corners, const CornersT &out_corners){
		CornersT norm_corners;
		ProjWarpT inv_norm_mat;
		normalizePts(norm_corners, inv_norm_mat, out_corners);
		ProjWarpT init_warp = computeAffineDLT(in_corners, norm_corners);
		init_warp = inv_norm_mat * init_warp;
		return init_warp;
	}
	//! for 3 pairs of corresponding points
	ProjWarpT computeAffineDLT(const Matrix23d &in_pts, const Matrix23d &out_pts){
		Matrix66d A;
		A.fill(0);
		Vector6d pt_vec;
		//! flattened version of the 2x4 matrix of target canonical points
		Map<const Vector6d> out_pts_vec(out_pts.data());

		for(int pt_id = 0; pt_id < 3; pt_id++){
			int r1 = 2 * pt_id;
			pt_vec(r1) = out_pts(0, pt_id);
			A(r1, 0) = in_pts(0, pt_id);
			A(r1, 1) = in_pts(1, pt_id);
			A(r1, 2) = 1;

			int r2 = 2 * pt_id + 1;
			pt_vec(r2) = out_pts(1, pt_id);
			A(r2, 3) = in_pts(0, pt_id);
			A(r2, 4) = in_pts(1, pt_id);
			A(r2, 5) = 1;
		}
		JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
		Matrix66d U = svd.matrixU();
		Matrix66d V = svd.matrixV();
		Vector6d S = svd.singularValues();
		Vector6d x = V * ((U.transpose() * out_pts_vec).array() / S.array()).matrix();
		ProjWarpT affine_mat = ProjWarpT::Zero();
		affine_mat(0, 0) = x(0);
		affine_mat(0, 1) = x(1);
		affine_mat(0, 2) = x(2);
		affine_mat(1, 0) = x(3);
		affine_mat(1, 1) = x(4);
		affine_mat(1, 2) = x(5);
		affine_mat(2, 2) = 1;
		return affine_mat;
	}

	ProjWarpT computeSimilitudeDLT(const CornersT &in_corners, const CornersT &out_corners){
		Matrix84d A;
		A.fill(0);
		// flattened version of the 2x4 matrix of target corner points
		//Map<Vector8d> in_corners_vec((double*)in_corners.data());
		//Map<Vector8d> out_corners_vec((double*)out_corners.data());
		//Vector8d corner_diff_vec = out_corners_vec - in_corners_vec;

		Vector8d corner_diff_vec;
		for(int i = 0; i < 4; i++){
			int r1 = 2 * i;
			A(r1, 0) = 1;
			A(r1, 2) = in_corners(0, i);
			A(r1, 3) = -in_corners(1, i);
			corner_diff_vec(r1) = out_corners(0, i) - in_corners(0, i);

			int r2 = 2 * i + 1;
			A(r2, 1) = 1;
			A(r2, 2) = in_corners(1, i);
			A(r2, 3) = in_corners(0, i);
			corner_diff_vec(r2) = out_corners(1, i) - in_corners(1, i);
		}
		JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
		Matrix84d U = svd.matrixU();
		Matrix44d V = svd.matrixV();
		Vector4d S = svd.singularValues();

		Vector4d x = V * ((U.transpose() * corner_diff_vec).array() / S.array()).matrix();

		ProjWarpT sim_mat = ProjWarpT::Zero();
		sim_mat(0, 0) = 1 + x(2);
		sim_mat(0, 1) = -x(3);
		sim_mat(0, 2) = x(0);
		sim_mat(1, 0) = x(3);
		sim_mat(1, 1) = 1 + x(2);
		sim_mat(1, 2) = x(1);
		sim_mat(2, 2) = 1;

		return sim_mat;
	}
	ProjWarpT computeSimilitudeNDLT(const CornersT &in_corners, const CornersT &out_corners){
		CornersT norm_corners;
		ProjWarpT inv_norm_mat;
		normalizePts(norm_corners, inv_norm_mat, out_corners);
		ProjWarpT init_warp = computeSimilitudeDLT(in_corners, norm_corners);
		init_warp = inv_norm_mat * init_warp;
		return init_warp;
	}
	ProjWarpT computeSimilitudeDLT(const PtsT &in_pts, const PtsT &out_pts){
		assert(in_pts.cols() == out_pts.cols());
		int n_pts = in_pts.cols();
		MatrixXd A(2 * n_pts, 4);
		A.fill(0);
		VectorXd corner_diff_vec(n_pts*2);
		for(int pt_id = 0; pt_id < n_pts; pt_id++){
			int r1 = 2 * pt_id;
			A(r1, 0) = 1;
			A(r1, 2) = in_pts(0, pt_id);
			A(r1, 3) = -in_pts(1, pt_id);
			corner_diff_vec(r1) = out_pts(0, pt_id) - in_pts(0, pt_id);

			int r2 = 2 * pt_id + 1;
			A(r2, 1) = 1;
			A(r2, 2) = in_pts(1, pt_id);
			A(r2, 3) = in_pts(0, pt_id);
			corner_diff_vec(r2) = out_pts(1, pt_id) - in_pts(1, pt_id);
		}
		JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
		VectorXd x = svd.matrixV() * ((svd.matrixU().transpose() * corner_diff_vec).array() / 
			svd.singularValues().array()).matrix();

		ProjWarpT sim_mat = ProjWarpT::Zero();
		sim_mat(0, 0) = 1 + x(2);
		sim_mat(0, 1) = -x(3);
		sim_mat(0, 2) = x(0);
		sim_mat(1, 0) = x(3);
		sim_mat(1, 1) = 1 + x(2);
		sim_mat(1, 2) = x(1);
		sim_mat(2, 2) = 1;

		return sim_mat;
	}
	ProjWarpT computeSimilitudeNDLT(const PtsT &in_pts, const PtsT &out_pts){
		assert(in_pts.cols() == out_pts.cols());
		PtsT norm_pts;
		norm_pts.resize(Eigen::NoChange, in_pts.cols());
		ProjWarpT inv_norm_mat;
		normalizePts(norm_pts, inv_norm_mat, out_pts);
		ProjWarpT init_warp = computeSimilitudeDLT(in_pts, norm_pts);
		init_warp = inv_norm_mat * init_warp;
		return init_warp;
	}

	Vector3d computeIsometryDLT(const PtsT &in_pts, const PtsT &out_pts){
		assert(in_pts.cols() == out_pts.cols());
		int n_pts = in_pts.cols();
		MatrixXd A(2 * n_pts, 4);
		A.fill(0);
		VectorXd corner_diff_vec(n_pts * 2);
		for(int pt_id = 0; pt_id < n_pts; ++pt_id){
			int r1 = 2 * pt_id;
			A(r1, 0) = 1;
			A(r1, 2) = in_pts(0, pt_id);
			A(r1, 3) = -in_pts(1, pt_id);
			corner_diff_vec(r1) = out_pts(0, pt_id) - in_pts(0, pt_id);

			int r2 = 2 * pt_id + 1;
			A(r2, 1) = 1;
			A(r2, 2) = in_pts(1, pt_id);
			A(r2, 3) = in_pts(0, pt_id);
			corner_diff_vec(r2) = out_pts(1, pt_id) - in_pts(1, pt_id);
		}
		JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
		VectorXd x = svd.matrixV() * ((svd.matrixU().transpose() * corner_diff_vec).array() /
			svd.singularValues().array()).matrix();

		double a = x[2], b = x[3];
		double theta = atan2(b, a + 1);
		Vector3d iso_params;
		iso_params[0] = x[0];
		iso_params[1] = x[1];
		iso_params[2] = theta;
		return iso_params;
	}
	Vector3d computeIsometryDLT(const CornersT &in_pts, const CornersT &out_pts){
		assert(in_pts.cols() == out_pts.cols());
		Matrix84d A;
		A.fill(0);
		Vector8d corner_diff_vec;
		for(int corner_id = 0; corner_id < 4; ++corner_id){
			int r1 = 2 * corner_id;
			A(r1, 0) = 1;
			A(r1, 2) = in_pts(0, corner_id);
			A(r1, 3) = -in_pts(1, corner_id);
			corner_diff_vec(r1) = out_pts(0, corner_id) - in_pts(0, corner_id);

			int r2 = 2 * corner_id + 1;
			A(r2, 1) = 1;
			A(r2, 2) = in_pts(1, corner_id);
			A(r2, 3) = in_pts(0, corner_id);
			corner_diff_vec(r2) = out_pts(1, corner_id) - in_pts(1, corner_id);
		}
		JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
		VectorXd x = svd.matrixV() * ((svd.matrixU().transpose() * corner_diff_vec).array() /
			svd.singularValues().array()).matrix();

		double a = x[2], b = x[3];
		double theta = atan2(b, a + 1);
		Vector3d iso_params;
		iso_params[0] = x[0];
		iso_params[1] = x[1];
		iso_params[2] = theta;
		return iso_params;
	}

	ProjWarpT computeTranscalingDLT(const CornersT &in_corners, const CornersT &out_corners){
		Matrix83d A;
		A.fill(0);
		// flattened version of the 2x4 matrix of target corner points
		//Map<Vector8d> in_corners_vec((double*)in_corners.data());
		//Map<Vector8d> out_corners_vec((double*)out_corners.data());
		//Vector8d corner_diff_vec = out_corners_vec - in_corners_vec;

		Vector8d corner_diff_vec;
		//utils::printMatrix(in_corners, "in_corners");
		//utils::printMatrix(out_corners_vec, "out_corners_vec");
		//utils::printMatrix(out_corners, "out_corners");
		for(int i = 0; i < 4; i++){
			int r1 = 2 * i;
			A(r1, 0) = 1;
			A(r1, 2) = in_corners(0, i);
			corner_diff_vec(r1) = out_corners(0, i) - in_corners(0, i);

			int r2 = 2 * i + 1;
			A(r2, 1) = 1;
			A(r2, 2) = in_corners(1, i);
			corner_diff_vec(r2) = out_corners(1, i) - in_corners(1, i);
		}
		JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
		Matrix83d U = svd.matrixU();
		ProjWarpT V = svd.matrixV();
		Vector3d S = svd.singularValues();

		Vector3d x = V * ((U.transpose() * corner_diff_vec).array() / S.array()).matrix();

		ProjWarpT transcale_mat = ProjWarpT::Zero();
		transcale_mat(0, 0) = 1 + x(2);
		transcale_mat(0, 2) = x(0);
		transcale_mat(1, 1) = 1 + x(2);
		transcale_mat(1, 2) = x(1);
		transcale_mat(2, 2) = 1;

		return transcale_mat;
	}

	ProjWarpT computeTranscalingDLT(const PtsT &in_pts, const PtsT &out_pts){
		assert(in_pts.cols() == out_pts.cols());
		int n_pts = in_pts.cols();
		MatrixXd A(2 * n_pts, 3);
		A.fill(0);
		VectorXd corner_diff_vec(n_pts * 2);
		//utils::printMatrix(in_corners, "in_corners");
		//utils::printMatrix(out_corners_vec, "out_corners_vec");
		//utils::printMatrix(out_corners, "out_corners");
		for(int pt_id = 0; pt_id < n_pts; pt_id++){
			int r1 = 2 * pt_id;
			A(r1, 0) = 1;
			A(r1, 2) = in_pts(0, pt_id);
			corner_diff_vec(r1) = out_pts(0, pt_id) - in_pts(0, pt_id);

			int r2 = 2 * pt_id + 1;
			A(r2, 1) = 1;
			A(r2, 2) = in_pts(1, pt_id);
			corner_diff_vec(r2) = out_pts(1, pt_id) - in_pts(1, pt_id);
		}
		JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

		Vector3d x = svd.matrixV() * ((svd.matrixU().transpose() * corner_diff_vec).array() /
			svd.singularValues().array()).matrix();

		ProjWarpT transcale_mat = ProjWarpT::Zero();
		transcale_mat(0, 0) = 1 + x(2);
		transcale_mat(0, 2) = x(0);
		transcale_mat(1, 1) = 1 + x(2);
		transcale_mat(1, 2) = x(1);
		transcale_mat(2, 2) = 1;

		return transcale_mat;
	}

	// normalizes the given points so that their mean (centroid) moves to origin
	// and their mean distance from origin becomes unity; also returns the inverse normalization
	// matrix which, when multiplied to the normalized points will give the original points back
	void normalizePts(CornersT &norm_pts, ProjWarpT &inv_norm_mat, const CornersT &pts){
		Vector2d centroid = pts.rowwise().mean();
		CornersT trans_pts = pts.colwise() - centroid;

		//double mean_dist = (trans_pts.norm())/4;

		double mean_dist = (trans_pts.array() * trans_pts.array()).rowwise().sum().sqrt().mean();

		if(mean_dist == 0){
			cout << "Error in getNormalizedPoints:: mean distance between the given points is zero:\n";
			cout << pts << "\n";
			return;
		}

		double norm_scale = sqrt(2) / mean_dist;
		norm_pts = trans_pts * norm_scale;

		//utils::printMatrix(pts, "pts");
		//utils::printMatrix(centroid, "centroid");

		//printf("mean_dist: %f\n", mean_dist);
		//printf("norm_scale: %f\n", norm_scale);

		//utils::printMatrix(trans_pts, "trans_pts");
		//utils::printMatrix(norm_pts, "norm_pts");

		inv_norm_mat.setIdentity();
		inv_norm_mat(0, 0) = inv_norm_mat(1, 1) = (1.0 / norm_scale);
		inv_norm_mat(0, 2) = centroid(0);
		inv_norm_mat(1, 2) = centroid(1);
	}

	void normalizePts(PtsT &norm_pts, ProjWarpT &inv_norm_mat, const PtsT &pts){
		Vector2d centroid = pts.rowwise().mean();
		PtsT trans_pts = pts.colwise() - centroid;

		//double mean_dist = (trans_pts.norm())/4;

		double mean_dist = (trans_pts.array() * trans_pts.array()).rowwise().sum().sqrt().mean();

		if(mean_dist == 0){
			cout << "Error in getNormalizedPoints:: mean distance between the given points is zero:\n";
			cout << pts << "\n";
			return;
		}

		double norm_scale = sqrt(2) / mean_dist;
		norm_pts = trans_pts * norm_scale;

		inv_norm_mat.setIdentity();
		inv_norm_mat(0, 0) = inv_norm_mat(1, 1) = (1.0 / norm_scale);
		inv_norm_mat(0, 2) = centroid(0);
		inv_norm_mat(1, 2) = centroid(1);
	}
	void decomposeHomographyForward(ProjWarpT &affine_mat, ProjWarpT &proj_mat,
		const ProjWarpT &hom_mat){
		assert(hom_mat(2, 2) == 1.0);

		double h1 = hom_mat(0, 0);
		double h2 = hom_mat(0, 1);
		double h3 = hom_mat(0, 2);
		double h4 = hom_mat(1, 0);
		double h5 = hom_mat(1, 1);
		double h6 = hom_mat(1, 2);
		double h7 = hom_mat(2, 0);
		double h8 = hom_mat(2, 1);

		double a1 = h1 - h3 * h7;
		double a2 = h2 - h3 * h8;
		double a3 = h3;
		double a4 = h4 - h6 * h7;
		double a5 = h5 - h6 * h8;
		double a6 = h6;
		affine_mat << a1, a2, a3, a4, a5, a6, 0, 0, 1;
		proj_mat << 1, 0, 0, 0, 1, 0, h7, h8, 1;
	}
	void decomposeHomographyInverse(ProjWarpT &affine_mat, ProjWarpT &proj_mat,
		const ProjWarpT &hom_mat){
		assert(hom_mat(2, 2) == 1.0);

		double h1 = hom_mat(0, 0);
		double h2 = hom_mat(0, 1);
		double h3 = hom_mat(0, 2);
		double h4 = hom_mat(1, 0);
		double h5 = hom_mat(1, 1);
		double h6 = hom_mat(1, 2);
		double h7 = hom_mat(2, 0);
		double h8 = hom_mat(2, 1);

		double v1 = (h5 * h7 - h4 * h8) / (h1 * h5 - h2 * h4);
		double v2 = (h1 * h8 - h2 * h7) / (h1 * h5 - h2 * h4);
		double u = 1 - v1 * h3 - v2 * h6;

		affine_mat << h1, h2, h3, h4, h5, h6, 0, 0, 1;
		proj_mat << 1, 0, 0, 0, 1, 0, v1, v2, u;
	}
	void decomposeAffineForward(Vector6d &affine_params,
		const ProjWarpT &affine_mat){
		assert(affine_mat(2, 0) == 0.0);
		assert(affine_mat(2, 1) == 0.0);
		assert(affine_mat(2, 2) == 1.0);

		double a1 = affine_mat(0, 0);
		double a2 = affine_mat(0, 1);
		double a3 = affine_mat(0, 2);
		double a4 = affine_mat(1, 0);
		double a5 = affine_mat(1, 1);
		double a6 = affine_mat(1, 2);

		double b = (a1*a2 + a4*a5) / (a1*a5 - a2*a4);
		double a = (b*a1 - a4) / a2 - 1;
		double s = sqrt(a1*a1 + a4*a4) / (1 + a) - 1;
		double tx = a3;
		double ty = a6;
		double cos_theta = a1 / ((1 + s) * (1 + a));
		double sin_theta = a4 / ((1 + s) * (1 + a));
		//if (cos_theta > 1.0){
		//	cos_theta = 0.0;
		//}
		double theta = acos(cos_theta);
		double new_sin_theta = sin(theta);
		double new_cos_theta = cos(theta);

		printScalar(theta, "theta");
		printScalar(sin_theta, "sin_theta");
		printScalar(cos_theta, "cos_theta");
		printScalar(new_sin_theta, "new_sin_theta");
		printScalar(new_cos_theta, "new_cos_theta");
		affine_params << tx, ty, theta, s, a, b;
	}
	void decomposeAffineForward(ProjWarpT &trans_mat, ProjWarpT &rot_mat,
		ProjWarpT &scale_mat, ProjWarpT &shear_mat, const ProjWarpT &affine_mat){
		Vector6d affine_params;
		decomposeAffineForward(affine_params, affine_mat);
		trans_mat = getTranslationMatrix(affine_params(0), affine_params(1));
		rot_mat = getRotationMatrix(affine_params(2));
		scale_mat = getScalingMatrix(affine_params(3));
		shear_mat = getShearingMatrix(affine_params(4), affine_params(5));
	}
	void decomposeAffineInverse(Vector6d &affine_params,
		const ProjWarpT &affine_mat){
		assert(affine_mat(2, 0) == 0.0);
		assert(affine_mat(2, 1) == 0.0);
		assert(affine_mat(2, 2) == 1.0);

		double a1 = affine_mat(0, 0);
		double a2 = affine_mat(0, 1);
		double a3 = affine_mat(0, 2);
		double a4 = affine_mat(1, 0);
		double a5 = affine_mat(1, 1);
		double a6 = affine_mat(1, 2);

		double a = (a1*a5 - a2*a4) / (a5*a5 + a4*a4) - 1;
		double b = (a2*a5 + a1*a4) / (a5*a5 + a4*a4);
		double a7 = (a3 - a6*b) / (1 + a);
		double s = sqrt(a5*a5 + a4*a4) - 1;
		double tx = (a4 * a6 + a5 * a7) / (a5 * a5 + a4 * a4);
		double ty = (a5 * a6 - a4 * a7) / (a5 * a5 + a4 * a4);
		double cos_theta = a5 / (1 + s);
		double sin_theta = a4 / (1 + s);

		if(cos_theta < 0 && sin_theta < 0){
			cos_theta = -cos_theta;
			sin_theta = -sin_theta;
			s = -(s + 2);
		}
		double theta = acos(cos_theta);
		affine_params << tx, ty, theta, s, a, b;
	}
	void decomposeAffineInverse(ProjWarpT &trans_mat, ProjWarpT &rot_mat,
		ProjWarpT &scale_mat, ProjWarpT &shear_mat, const ProjWarpT &affine_mat){
		Vector6d affine_params;
		decomposeAffineInverse(affine_params, affine_mat);
		trans_mat = getTranslationMatrix(affine_params(0), affine_params(1));
		rot_mat = getRotationMatrix(affine_params(2));
		scale_mat = getScalingMatrix(affine_params(3));
		shear_mat = getShearingMatrix(affine_params(4), affine_params(5));

		printMatrix(affine_params, "affine_params");
		printMatrix(trans_mat, "trans_mat");
		printMatrix(rot_mat, "rot_mat");
		printMatrix(scale_mat, "scale_mat");
		printMatrix(shear_mat, "shear_mat");
	}
	// compute the thin plate spline transformation relating two sets of corners
	MatrixX2d computeTPS(const CornersT &in_corners, const CornersT &out_corners){
		Matrix4d K;
		K(0, 0) = K(1, 1) = K(2, 2) = K(3, 3) = 0;
		K(0, 1) = K(1, 0) = tps((in_corners.col(0) - in_corners.col(1)).norm());
		K(0, 2) = K(2, 0) = tps((in_corners.col(0) - in_corners.col(2)).norm());
		K(0, 3) = K(3, 0) = tps((in_corners.col(0) - in_corners.col(3)).norm());
		K(1, 2) = K(2, 1) = tps((in_corners.col(1) - in_corners.col(2)).norm());
		K(1, 3) = K(3, 1) = tps((in_corners.col(1) - in_corners.col(3)).norm());
		K(2, 3) = K(3, 2) = tps((in_corners.col(2) - in_corners.col(3)).norm());
		Matrix43d P;
		P.col(0).setOnes();
		P.col(1) = in_corners.row(0).transpose();
		P.col(2) = in_corners.row(1).transpose();

		Matrix7d L;
		L << K, P, P.transpose(), ProjWarpT::Zero();
		Matrix72d Y;
		Y.topRows <4>() = out_corners.transpose();
		Y.bottomRows<3>().fill(0);
		MatrixX2d tps_params = L.colPivHouseholderQr().solve(Y);
		//VectorXd tps_params_vec=
		return tps_params;
	}
	// applies the thin plate spline transformation defined by the given parameters
	// and control points to the given points
	void applyTPS(PtsT &out_pts, const PtsT &in_pts,
		const PtsT &control_pts, const MatrixX2d &tps_params){
		assert(out_pts.cols() == in_pts.size());
		assert(tps_params.rows() == control_pts.cols() + 3);
		int n_pts = in_pts.cols();
		int n_ctrl_pts = control_pts.cols();
		double ax = tps_params(n_ctrl_pts, 0),
			bx = tps_params(n_ctrl_pts + 1, 0),
			cx = tps_params(n_ctrl_pts + 2, 0);
		double ay = tps_params(n_ctrl_pts, 1),
			by = tps_params(n_ctrl_pts + 1, 1),
			cy = tps_params(n_ctrl_pts + 2, 1);
		for(int pt_id = 0; pt_id < n_pts; ++n_pts){

			double out_pt_x = ax*in_pts(0, pt_id) + bx*in_pts(1, pt_id) + cx;
			double out_pt_y = ay*in_pts(0, pt_id) + by*in_pts(1, pt_id) + cy;
			for(int ctrl_pt_id = 0; ctrl_pt_id < n_ctrl_pts; ++n_pts){
				double pt_tps = tps((control_pts.col(ctrl_pt_id) - in_pts.col(pt_id)).norm());
				out_pt_x += tps_params(ctrl_pt_id, 0)*pt_tps;
				out_pt_y += tps_params(ctrl_pt_id, 1)*pt_tps;
			}
			out_pts(0, pt_id) = out_pt_x;
			out_pts(1, pt_id) = out_pt_y;
		}
	}	
}

_MTF_END_NAMESPACE
