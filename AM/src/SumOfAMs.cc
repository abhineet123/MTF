#include "mtf/AM/SumOfAMs.h"

_MTF_BEGIN_NAMESPACE

SumOfAMs::SumOfAMs(AppearanceModel *_am1, AppearanceModel *_am2) :
AppearanceModel(),
am1(_am1), am2(_am2){
	name = "sum";
	printf("\n");
	printf("Using Sum of AMs with...\n");
	printf("am1: %s\n", am1->name.c_str());
	printf("am2: %s\n", am2->name.c_str());
	resx = am1->getResX();
	resy = am1->getResY();
	n_pix = am1->getNPix();
	patch_size = am1->getPatchSize();
	grad_eps = am1->getGradOffset();
	hess_eps = am1->getHessOffset();
}

void SumOfAMs::setCurrImg(const cv::Mat &cv_img){

	am1->setCurrImg(cv_img);
	am2->setCurrImg(cv_img);
}

void SumOfAMs::initializePixVals(const Matrix2Xd& init_pts){
	if(!isInitialized()->pix_vals){
		I0.resize(patch_size);
		It.resize(patch_size);
	}

	am1->initializePixVals(init_pts);
	am2->initializePixVals(init_pts);

	I0 = am1->getInitPixVals();

	if(!isInitialized()->pix_vals){
		It = I0;
		isInitialized()->pix_vals = true;
	}
}

void SumOfAMs::initializePixGrad(const Matrix2Xd &init_pts){
	if(!isInitialized()->pix_grad){
		dI0_dx.resize(patch_size, Eigen::NoChange);
		dIt_dx.resize(patch_size, Eigen::NoChange);
	}

	am1->initializePixGrad(init_pts);
	am2->initializePixGrad(init_pts);

	dI0_dx = am1->getInitPixGrad();
	if(!isInitialized()->pix_grad){
		dIt_dx = dI0_dx;
		isInitialized()->pix_grad = true;
	}
}

void SumOfAMs::initializePixGrad(const Matrix8Xd &warped_offset_pts){
	assert(warped_offset_pts.cols() == n_pix);
	if(!isInitialized()->pix_grad){
		dI0_dx.resize(patch_size, Eigen::NoChange);
		dIt_dx.resize(patch_size, Eigen::NoChange);
	}

	am1->initializePixGrad(warped_offset_pts);
	am2->initializePixGrad(warped_offset_pts);

	dI0_dx = am1->getInitPixGrad();

	if(!isInitialized()->pix_grad){
		dIt_dx = dI0_dx;
		isInitialized()->pix_grad = true;
	}
}

void SumOfAMs::initializePixHess(const Matrix2Xd& init_pts,
	const Matrix16Xd &warped_offset_pts){
	if(!isInitialized()->pix_hess){
		d2I0_dx2.resize(Eigen::NoChange, patch_size);
		d2It_dx2.resize(Eigen::NoChange, patch_size);
	}

	am1->initializePixHess(init_pts, warped_offset_pts);
	am2->initializePixHess(init_pts, warped_offset_pts);

	dI0_dx = am1->getInitPixHess();

	if(!isInitialized()->pix_hess){
		d2It_dx2 = d2I0_dx2;
		isInitialized()->pix_hess = true;
	}
}
void SumOfAMs::initializePixHess(const Matrix2Xd &init_pts){
	if(!isInitialized()->pix_hess){
		d2I0_dx2.resize(Eigen::NoChange, patch_size);
		d2It_dx2.resize(Eigen::NoChange, patch_size);
	}

	am1->initializePixHess(init_pts);
	am2->initializePixHess(init_pts);

	dI0_dx = am1->getInitPixHess();

	if(!isInitialized()->pix_hess){
		d2It_dx2 = d2I0_dx2;
		isInitialized()->pix_hess = true;
	}
}

void SumOfAMs::updatePixVals(const Matrix2Xd& curr_pts){
	am1->updatePixVals(curr_pts);
	am2->updatePixVals(curr_pts);
	It = am1->getCurrPixVals();
}

void SumOfAMs::updatePixGrad(const Matrix2Xd &curr_pts){
	am1->updatePixGrad(curr_pts);
	am2->updatePixGrad(curr_pts);
	dIt_dx = am1->getCurrPixGrad();
}

void SumOfAMs::updatePixHess(const Matrix2Xd &curr_pts){
	am1->updatePixHess(curr_pts);
	am2->updatePixHess(curr_pts);
	d2It_dx2 = am1->getCurrPixHess();
}

void SumOfAMs::updatePixGrad(const Matrix8Xd &warped_offset_pts){
	am1->updatePixGrad(warped_offset_pts);
	am2->updatePixGrad(warped_offset_pts);
	dIt_dx = am1->getCurrPixGrad();
}

void SumOfAMs::updatePixHess(const Matrix2Xd& curr_pts,
	const Matrix16Xd &warped_offset_pts){
	am1->updatePixHess(curr_pts, warped_offset_pts);
	am2->updatePixHess(curr_pts, warped_offset_pts);
	d2It_dx2 = am1->getCurrPixHess();
}

void SumOfAMs::initializeSimilarity(){
	if(!is_initialized.similarity){
		am1->initializeSimilarity();
		am2->initializeSimilarity();
		am1_norm_factor = 1.0 / (1 + am1->getSimilarity()*am1->getSimilarity());
		am2_norm_factor = 1.0 / (1 + am2->getSimilarity()*am2->getSimilarity());
		is_initialized.similarity = true;
	}
}

void SumOfAMs::initializeGrad(){
	if(!is_initialized.grad){
		am1->initializeGrad();
		am2->initializeGrad();
		is_initialized.grad = true;
	}
}
void SumOfAMs::initializeHess(){
	if(!is_initialized.hess){
		am1->initializeHess();
		am2->initializeHess();
		is_initialized.hess = true;
	}
}

void SumOfAMs::updateSimilarity(bool prereq_only){
	am1->updateSimilarity(prereq_only);
	am2->updateSimilarity(prereq_only);
	if(prereq_only){ return; }
	f = am1->getSimilarity()*am1_norm_factor + am2->getSimilarity()*am2_norm_factor;
}

void SumOfAMs::updateInitGrad(){
	am1->updateInitGrad();
	am2->updateInitGrad();
	df_dI0 = am1->getInitGrad()*am1_norm_factor + am2->getInitGrad()*am2_norm_factor;

}
void SumOfAMs::updateCurrGrad(){
	am1->updateCurrGrad();
	am2->updateCurrGrad();
	df_dIt = am1->getCurrGrad()*am1_norm_factor + am2->getCurrGrad()*am2_norm_factor;
}

void SumOfAMs::cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian){
	MatrixXd am1_init_hessian(init_hessian.rows(), init_hessian.cols());
	MatrixXd am2_init_hessian(init_hessian.rows(), init_hessian.cols());

	am1->cmptInitHessian(am1_init_hessian, init_pix_jacobian);
	am2->cmptInitHessian(am2_init_hessian, init_pix_jacobian);
	init_hessian = am1_init_hessian*am1_norm_factor + am2_init_hessian*am2_norm_factor;
}

void SumOfAMs::cmptInitHessian(MatrixXd &init_hessian, const MatrixXd &init_pix_jacobian,
	const MatrixXd &init_pix_hessian){
	MatrixXd am1_init_hessian(init_hessian.rows(), init_hessian.cols());
	MatrixXd am2_init_hessian(init_hessian.rows(), init_hessian.cols());

	am1->cmptInitHessian(am1_init_hessian, init_pix_jacobian, init_pix_hessian);
	am2->cmptInitHessian(am2_init_hessian, init_pix_jacobian, init_pix_hessian);
	init_hessian = am1_init_hessian*am1_norm_factor + am2_init_hessian*am2_norm_factor;
}

void SumOfAMs::cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian){
	MatrixXd am1_curr_hessian(curr_hessian.rows(), curr_hessian.cols());
	MatrixXd am2_curr_hessian(curr_hessian.rows(), curr_hessian.cols());

	am1->cmptCurrHessian(am1_curr_hessian, curr_pix_jacobian);
	am2->cmptCurrHessian(am2_curr_hessian, curr_pix_jacobian);
	curr_hessian = am1_curr_hessian*am1_norm_factor + am2_curr_hessian*am2_norm_factor;
}

void SumOfAMs::cmptCurrHessian(MatrixXd &curr_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	MatrixXd am1_curr_hessian(curr_hessian.rows(), curr_hessian.cols());
	MatrixXd am2_curr_hessian(curr_hessian.rows(), curr_hessian.cols());

	am1->cmptCurrHessian(am1_curr_hessian, curr_pix_jacobian, curr_pix_hessian);
	am2->cmptCurrHessian(am2_curr_hessian, curr_pix_jacobian, curr_pix_hessian);
	curr_hessian = am1_curr_hessian*am1_norm_factor + am2_curr_hessian*am2_norm_factor;
}

void SumOfAMs::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian){
	MatrixXd am1_self_hessian(self_hessian.rows(), self_hessian.cols());
	MatrixXd am2_self_hessian(self_hessian.rows(), self_hessian.cols());

	am1->cmptSelfHessian(am1_self_hessian, curr_pix_jacobian);
	am2->cmptSelfHessian(am2_self_hessian, curr_pix_jacobian);
	self_hessian = am1_self_hessian*am1_norm_factor + am2_self_hessian*am2_norm_factor;
}

void SumOfAMs::cmptSelfHessian(MatrixXd &self_hessian, const MatrixXd &curr_pix_jacobian,
	const MatrixXd &curr_pix_hessian){
	MatrixXd am1_self_hessian(self_hessian.rows(), self_hessian.cols());
	MatrixXd am2_self_hessian(self_hessian.rows(), self_hessian.cols());

	am1->cmptSelfHessian(am1_self_hessian, curr_pix_jacobian, curr_pix_hessian);
	am2->cmptSelfHessian(am2_self_hessian, curr_pix_jacobian, curr_pix_hessian);
	self_hessian = am1_self_hessian*am1_norm_factor + am2_self_hessian*am2_norm_factor;
}


/*Support for FLANN library*/

int SumOfAMs::getDistFeatSize(){ return am1->getDistFeatSize() + am2->getDistFeatSize(); }

void SumOfAMs::initializeDistFeat(){
	am1->initializeDistFeat();
	am2->initializeDistFeat();
	am1_dist_feat_size = am1->getDistFeatSize();
	am2_dist_feat_size = am2->getDistFeatSize();
	curr_feat_vec.resize(am1_dist_feat_size + am2_dist_feat_size);
}

void SumOfAMs::updateDistFeat(double* feat_addr){
	am1->updateDistFeat(feat_addr);
	am2->updateDistFeat(feat_addr + am1_dist_feat_size);
}

void SumOfAMs::updateDistFeat(){
	updateDistFeat(curr_feat_vec.data());
}

double SumOfAMs::operator()(const double* a, const double* b, size_t size, double worst_dist) const{
	double am1_dist = am1->operator()(a, b, am1_dist_feat_size, worst_dist);
	double am2_dist = am2->operator()(a + am1_dist_feat_size, 
		b + am1_dist_feat_size, am2_dist_feat_size, worst_dist);
	return am1_dist*am1_norm_factor + am2_dist*am2_norm_factor;
}

_MTF_END_NAMESPACE

