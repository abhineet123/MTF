#include "mtf/AM/ImageBase.h"
#include "mtf/Utilities/imgUtils.h"

_MTF_BEGIN_NAMESPACE

ImgParams::ImgParams(int _resx, int _resy,
double _grad_eps, double _hess_eps,
bool _use_uchar_input) :
resx(_resx), resy(_resy),
grad_eps(_grad_eps), hess_eps(_hess_eps),
uchar_input(_use_uchar_input){}

ImgParams::ImgParams(const ImgParams *img_params) :
resx(MTF_RES), resy(MTF_RES),
grad_eps(GRAD_EPS), hess_eps(HESS_EPS),
uchar_input(UCHAR_INPUT){
	if(img_params){
		resx = img_params->resx;
		resy = img_params->resy;
		grad_eps = img_params->grad_eps;
		hess_eps = img_params->hess_eps;
		uchar_input = img_params->uchar_input;
	}
}

ImageBase::ImageBase(const ImgParams *img_params, const int _n_channels) :
curr_img(nullptr, 0, 0), img_height(0), img_width(0),
resx(MTF_RES), resy(MTF_RES), n_pix(MTF_RES*MTF_RES), n_channels(_n_channels),
pix_norm_add(0.0), pix_norm_mult(1.0), frame_count(0),
grad_eps(GRAD_EPS), hess_eps(HESS_EPS),
uchar_input(UCHAR_INPUT){
	if(img_params) {
		if(img_params->resx <= 0 || img_params->resy <= 0) {
			throw std::invalid_argument("ImageBase::Invalid sampling resolution provided");
		}
		resx = img_params->resx;
		resy = img_params->resy;
		n_pix = resx*resy;
		grad_eps = img_params->grad_eps;
		hess_eps = img_params->hess_eps;
		uchar_input = img_params->uchar_input;
	}
	patch_size = n_pix*n_channels;
}

void ImageBase::setCurrImg(const cv::Mat &cv_img){
	assert(cv_img.type() == inputType());
	img_height = cv_img.rows;
	img_width = cv_img.cols;
	curr_img_cv = cv_img;
	uchar_input = cv_img.type() == CV_8UC1 || cv_img.type() == CV_8UC3;
	if(cv_img.type() != inputType()){
		throw std::invalid_argument(
			cv_format("ImageBase::Input image type: %s does not match the required type: %s",
			utils::getType(cv_img), utils::typeToString(inputType()))
			);
	}
	if(!uchar_input && n_channels == 1){
		// single channel image can share data with an Eigen matrix
		// not really necessary but remains as a relic from the past
		new (&curr_img) EigImgT((EigPixT*)(cv_img.data), img_height, img_width);
	}
}

void ImageBase::initializePixVals(const Matrix2Xd& init_pts){
	assert(init_pts.cols() == n_pix);
	if(!isInitialized()->pix_vals){
		I0.resize(patch_size);
		It.resize(patch_size);
#if !defined DEFAULT_PIX_INTERP_TYPE
		printf("Using pixel interpolation type: %s\n", utils::toString(PIX_INTERP_TYPE));
#endif
#if !defined DEFAULT_PIX_BORDER_TYPE
		printf("Using pixel border type: %s\n", utils::toString(PIX_BORDER_TYPE));
#endif
	}
	++frame_count;
	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getPixVals<uchar>(I0, curr_img_cv, init_pts, n_pix,
				img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		case 3:
			utils::mc::getPixVals<uchar>(I0, curr_img_cv, init_pts, n_pix,
				img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getPixVals(I0, curr_img, init_pts, n_pix,
				img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		case 3:
			utils::mc::getPixVals<float>(I0, curr_img_cv, init_pts, n_pix,
				img_height, img_width, pix_norm_mult, pix_norm_add);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
	if(!isInitialized()->pix_vals){
		It = I0;
		isInitialized()->pix_vals = true;
	}
}

void ImageBase::initializePixGrad(const Matrix2Xd &init_pts){
	assert(init_pts.cols() == n_pix);

	if(!isInitialized()->pix_grad){
		dI0_dx.resize(patch_size, Eigen::NoChange);
		dIt_dx.resize(patch_size, Eigen::NoChange);
	}
	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getImgGrad<uchar>(dI0_dx, curr_img_cv, init_pts, grad_eps, n_pix,
				img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getImgGrad<uchar>(dI0_dx, curr_img_cv, init_pts, grad_eps, n_pix,
				img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getImgGrad(dI0_dx, curr_img, init_pts, grad_eps, n_pix,
				img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getImgGrad<float>(dI0_dx, curr_img_cv, init_pts, grad_eps, n_pix,
				img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
	if(!isInitialized()->pix_grad){
		setCurrPixGrad(getInitPixGrad());
		isInitialized()->pix_grad = true;
	}
}

void ImageBase::initializePixGrad(const Matrix8Xd &warped_offset_pts){
	assert(warped_offset_pts.cols() == n_pix);
	if(!isInitialized()->pix_grad){
#if !defined DEFAULT_GRAD_INTERP_TYPE
		printf("Using gradient interpolation type: %s\n",
			utils::toString(GRAD_INTERP_TYPE));
#endif
		dI0_dx.resize(patch_size, Eigen::NoChange);
		dIt_dx.resize(patch_size, Eigen::NoChange);
	}

	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getWarpedImgGrad<uchar>(dI0_dx,
				curr_img_cv, warped_offset_pts, grad_eps, n_pix,
				img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getWarpedImgGrad<uchar>(dI0_dx,
				curr_img_cv, warped_offset_pts, grad_eps, n_pix,
				img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getWarpedImgGrad(dI0_dx,
				curr_img, warped_offset_pts, grad_eps, n_pix,
				img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getWarpedImgGrad<float>(dI0_dx,
				curr_img_cv, warped_offset_pts, grad_eps, n_pix,
				img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
	if(!isInitialized()->pix_grad){
		setCurrPixGrad(getInitPixGrad());
		isInitialized()->pix_grad = true;
	}
}

void ImageBase::initializePixHess(const Matrix2Xd& init_pts,
	const Matrix16Xd &warped_offset_pts){
	assert(init_pts.cols() == n_pix && warped_offset_pts.cols() == n_pix);
	if(!isInitialized()->pix_hess){
		d2I0_dx2.resize(Eigen::NoChange, patch_size);
		d2It_dx2.resize(Eigen::NoChange, patch_size);
	}

	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getWarpedImgHess<uchar>(d2I0_dx2, curr_img_cv, init_pts, warped_offset_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getWarpedImgHess<uchar>(d2I0_dx2, curr_img_cv, init_pts, warped_offset_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getWarpedImgHess(d2I0_dx2, curr_img, init_pts, warped_offset_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getWarpedImgHess<float>(d2I0_dx2, curr_img_cv, init_pts, warped_offset_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
	if(!isInitialized()->pix_hess){
		setCurrPixHess(getInitPixHess());
		isInitialized()->pix_hess = true;
	}
}
void ImageBase::initializePixHess(const Matrix2Xd &init_pts){
	assert(init_pts.cols() == n_pix);
	if(!isInitialized()->pix_hess){
#if !defined DEFAULT_HESS_INTERP_TYPE
		printf("Using Hessian interpolation type: %s\n",
			utils::toString(HESS_INTERP_TYPE));
#endif
		d2I0_dx2.resize(Eigen::NoChange, patch_size);
		d2It_dx2.resize(Eigen::NoChange, patch_size);
	}

	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getImgHess<uchar>(d2I0_dx2, curr_img_cv, init_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getImgHess<uchar>(d2I0_dx2, curr_img_cv, init_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getImgHess(d2I0_dx2, curr_img, init_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getImgHess<float>(d2I0_dx2, curr_img_cv, init_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
	if(!isInitialized()->pix_hess){
		setCurrPixHess(getInitPixHess());
		isInitialized()->pix_hess = true;
	}
}

void ImageBase::extractPatch(VectorXd &pix_vals, const Matrix2Xd& pts){
	assert(pix_vals.size() == patch_size && pts.cols() == n_pix);

	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getPixVals<uchar>(pix_vals, curr_img_cv, pts, n_pix, img_height, img_width);
			break;
		case 3:
			utils::mc::getPixVals<uchar>(pix_vals, curr_img_cv, pts, n_pix, img_height, img_width);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getPixVals(pix_vals, curr_img, pts, n_pix, img_height, img_width);
			break;
		case 3:
			utils::mc::getPixVals<float>(pix_vals, curr_img_cv, pts, n_pix, img_height, img_width);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
}

VectorXd ImageBase::getPatch(const PtsT& curr_pts){
	VectorXd curr_patch(patch_size);
	extractPatch(curr_patch, curr_pts);
	return curr_patch;
}

void ImageBase::updatePixVals(const Matrix2Xd& curr_pts){
	assert(curr_pts.cols() == n_pix);

	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getPixVals<uchar>(It, curr_img_cv, curr_pts, n_pix, img_height, img_width,
				pix_norm_mult, pix_norm_add);
			break;
		case 3:
			utils::mc::getPixVals<uchar>(It, curr_img_cv, curr_pts, n_pix, img_height, img_width,
				pix_norm_mult, pix_norm_add);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getPixVals(It, curr_img, curr_pts, n_pix, img_height, img_width,
				pix_norm_mult, pix_norm_add);
			break;
		case 3:
			utils::mc::getPixVals<float>(It, curr_img_cv, curr_pts, n_pix, img_height, img_width,
				pix_norm_mult, pix_norm_add);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
}

void ImageBase::updatePixGrad(const Matrix2Xd &curr_pts){
	assert(curr_pts.cols() == n_pix);

	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getImgGrad<uchar>(dIt_dx, curr_img_cv, curr_pts,
				grad_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getImgGrad<uchar>(dIt_dx, curr_img_cv, curr_pts,
				grad_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getImgGrad(dIt_dx, curr_img, curr_pts,
				grad_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getImgGrad<float>(dIt_dx, curr_img_cv, curr_pts,
				grad_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
}

void ImageBase::updatePixHess(const Matrix2Xd &curr_pts){
	assert(curr_pts.cols() == n_pix);
	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getImgHess<uchar>(d2It_dx2, curr_img_cv, curr_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getImgHess<uchar>(d2It_dx2, curr_img_cv, curr_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getImgHess(d2It_dx2, curr_img, curr_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getImgHess<float>(d2It_dx2, curr_img_cv, curr_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
}

void ImageBase::updatePixGrad(const Matrix8Xd &warped_offset_pts){
	assert(warped_offset_pts.cols() == n_pix);

	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getWarpedImgGrad<uchar>(dIt_dx, curr_img_cv, warped_offset_pts,
				grad_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getWarpedImgGrad<uchar>(dIt_dx, curr_img_cv, warped_offset_pts,
				grad_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	} else{

		switch(n_channels){
		case 1:
			utils::getWarpedImgGrad(dIt_dx, curr_img, warped_offset_pts,
				grad_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getWarpedImgGrad<float>(dIt_dx, curr_img_cv, warped_offset_pts,
				grad_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
}

void ImageBase::updatePixHess(const Matrix2Xd& curr_pts,
	const Matrix16Xd &warped_offset_pts){
	assert(curr_pts.cols() == n_pix && warped_offset_pts.cols() == n_pix);

	if(uchar_input){
		switch(n_channels){
		case 1:
			utils::sc::getWarpedImgHess<uchar>(d2It_dx2, curr_img_cv, curr_pts, warped_offset_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getWarpedImgHess<uchar>(d2It_dx2, curr_img_cv, curr_pts, warped_offset_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	} else{
		switch(n_channels){
		case 1:
			utils::getWarpedImgHess(d2It_dx2, curr_img, curr_pts, warped_offset_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		case 3:
			utils::mc::getWarpedImgHess<float>(d2It_dx2, curr_img_cv, curr_pts, warped_offset_pts,
				hess_eps, n_pix, img_height, img_width, pix_norm_mult);
			break;
		default:
			throw std::domain_error(cv::format("%d channel images are not supported yet", n_channels));
		}
	}
}

_MTF_END_NAMESPACE