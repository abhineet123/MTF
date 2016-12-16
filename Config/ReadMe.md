Input parameters can be specified in 5 plain text files called **mtf.cfg**, **modules.cfg**, **sigma.cfg**, **thirdparty.cfg** and **multi.cfg** (located in this folder by default) where each line specifies the value of one parameter as: `<param_name><tab><param_val>`. If these files are present in some other folder, its path can be specified at runtime as `runMTF config_dir <directory containing the cfg files>`

Note that all parameters described here or present in **mtf.cfg**, **modules.cfg** and **thirdparty.cfg** can be specified in either of these files since all are read sequentially - this split is done only for convenience with mtf.cfg containing parameters relevant to the tracking task in general and the other two having parameters specific to MTF modules and third party trackers respectively;; if a parameter is specified in multiple files, its value in a later file will override that in the earlier ones; **sigma.cfg** specifies a list of standard deviations and means along with respective IDs that can be used as input arguments (in modules.cfg) for stochastic modules like NN, PF and RegNet to specify the Gaussian distributions from where these will draw samples;

**multi.cfg** specifies configurations for individual trackers in multi tracker setups like CascadeTracker/ParallelTracker for single object tracking or when tracking multiple objects simultaneously.
The parameters that can be specified here are same as in the last two files and will override the values specified there for each specific tracker thus enabling different trackers to have independent settings.
The settings for two trackers should be separated by an empty line.  Also note that the changes made by specifying parameters in **multi.cfg** are global, i.e. if the value specified for a specific parameter for one tracker will be used for all subsequent trackers too unless overridden again.

**Note: If the value of any parameter is prefixed by #, it is ignored and its default value in parameters.h, if any, is used instead. Similarly any line that starts with # is treated as a comment and hence ignored. Also argument names are supposed to start with a letter so any numbers or special characters at the beginning of any argument name (except # of course) are ignored and only the part starting at the first letter is considered as the name. This can be used to assign numeric IDs to arguments if needed (e.g. as done for ssm_sigma in modules.cfg)**

The parameters can also be specified from the command line through a list of argument pairs as follows:

	runMTF <arg_name_1> <arg_val_1> <arg_name_2> <arg_val2> .... <arg_name_n> <arg_val_n>
	
where the valid values of `arg_name` and `arg_val` are same as in the cfg files - these arguments will override the values specified in those files in case both are provided;
any invalid values for `arg_name` will be ignored along with its `arg_val`;

Following are some of important parameters, their brief descriptions and possible values:

Input/Output related parameters:
================================
	 Parameter:	'pipeline'
		Description:
			input video pipeline
		Possible Values:
			c: OpenCV
		If ViSP is enabled during compilation:
			v: ViSP
		If Xvision is enabled during compilation:
			x: Xvision
			
	 Parameter:	'img_source'
		Description:
			input video source/stream
		Possible Values:
			m: MPEG video file(OpenCV can read AVI files too)
			j: JPEG image files (some other common formats line PNG and BMP are supported too)
			u: USB camera
			f: Firewire camera (only ViSP and Xvision  pipelines; 
				USB camera option(u) can be used to access Firewire cameras with OpenCV as long as no USB cameras are attached)
			
	 Parameter:	'actor_id'
		Description:
			integral index of the dataset (or "actor") to use out of the possible datasets hard coded in datasets.h;
			used in conjunction with the parameter 'source_id'  to get the source name;
			only matters if both are non negative
		Possible Values:
				0: TMT
				1: UCSB
				2: LinTrack
				3: PAMI
				4: LinTrackShort
				5: METAIO
				6: CMT
				7: VOT
				8: VOT16
				9: VTB
				10: VIVID
				11: TrakMark
				12: TMT_FINE
				13: Mosaic
				14: Misc
				15: Synthetic
				16: Live
			Note: the first 10 datasets (except LinTrackShort) in a ready to use form can be downloaded from the MTF website: http://webdocs.cs.ualberta.ca/~vis/mtf/

	 Parameter:	'db_root_path'
		Description:
			location of the root directory that contains the files for all datasets (or 'actors');
			for JPEG file the full path is constructed as: db_root_path/actor/source_name/*.source_fmt;
			for MPEG/Video file the full path is constructed as: db_root_path/actor/source_name.source_fmt;
				
	 Parameter:	'source_id'
		Description:
			integral index of the source name to use out of the sources hard coded in parameters.h;
			used in conjunction with the parameter 'actor_id' to get the source name;
			only matters if both are non negative
		Possible Values:
			refer parameters.h for details of what each index means for each actor type; 
			following are the valid (inclusive) ranges for different actors:
				TMT:	0-108
				UCSB:	0-95
				LinTrack:	0-2
				PAMI:	0-27
				METAIO:	0-39
				CMT:	0-19				
				VOT:	0-99	
				VTB:	0-24	
				VIVID:	0-8
				Synthetic:	0-2
				Live:	0-1
				
	 Parameter:	'source_name'
		Description:
			name of the input video file (for MPEG source) or folder (for JPEG source); 
				overridden if both the parameters 'source_id' and 'actor_id' are non-negative;
				does not matter if a camera stream is being used;
			
	 Parameter:	'source_path'
		Description:
			only matters for Xvision pipeline and camera streams; 
			specifies the path of the camera device to use
		Possible Values:
			depend on the number and types of camera attached to the system;
			following are common values for the two camera types:
				/dev/video0:	USB
				/dev/fw1:	Firewire
				for OpenCV pipeline with USB camera source, it should be an integer ID specifying the camera (e.g. 0 for the first camera, 1 for the second camera and so on.)
			
	 Parameter:	'source_fmt'
		Description:
			file extension for image and video file streams;
			any special formatting strings to be passed to the Xvision initializer for camera streams;
		Possible Values:
			jpg:	JPEG image files
			mpg:	MPEG video file
			avi:	AVI	video file (only OpenCV pipeline)
			if XVision pipeline is used with a camera source, this can be used to pass any formatting strings specifying the resolution, FPS and other relevant options.
			
	 Parameter:	'init_frame_id'
		Description:
			id of the frame at which the tracker is to be initialized in case tracking is desired to be started in the middle of the sequence rather than the beginning;
		Possible Values:
			should be between 0 and no_of_frames-1	
			
	 Parameter:	'start_frame_id'
		Description:
			id of the frame after which the tracking actually starts; can be used to start tracking in the middle of the sequence but still initialize in the first frame;
			only matters if it is greater than init_frame_id;
			only works with trackers that have setRegion function implemented (none of the third party trackers currently)
		Possible Values:
			should be between 0 and no_of_frames-1	
			
	 Parameter:	'read_obj_from_file'
		Description:
			read initial location of the object to be tracked from the text file specified by 'read_obj_fname' where they were previously written to by enabling 'write_objs';
			this is meant to avoid having to specify a custom initialization locations for one or more trackers repeatedly
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'read_obj_fname'
		Description:
			name of the text file where the initial location of the object to be tracked will be read from;
			only matters if read_objs is 1			
			
	 Parameter:	'write_objs'
		Description:
			write the manually selected initial object location to the text file specified by 'write_obj_fname';
			only matters if manual selection is enabled by disabling both 'read_objs' and 'read_obj_from_gt';
			this is meant to avoid having to specify a custom initialization locations for one or more trackers repeatedly
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'write_obj_fname'
		Description:
			name of the text file where the initial location of the object to be tracked will be written to;
			only matters if manual selection is enabled by disabling both 'read_objs' and 'read_obj_from_gt' and enabling 'write_objs'
			
	 Parameter:	'read_obj_from_gt'
		Description:
			read initial object location from a ground truth file present in the same directory as the input source file;
			matters only if a file stream is being used; 
			the format of this file should be identical to the ground truth files for the TMT dataset available here:
				http://webdocs.cs.ualberta.ca/~vis/trackDB/firstpage.html
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'show_cv_window'
		Description:
			show the result of tracking from frame to frame in an OpenCV window;
			disabling it can speed up the overall tracking speed by eliminating the delay caused by drawing the object locations on the current frame;
			useful for benchmarking
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'show_tracking_error'
		Description:
			show the the tracking error in terms of the mean corner distance between the tracking result and the ground truth in the OpenCV window; 
			only matters if read_objs_from_gt is enabled and a file input source (video or image) is used; 
			a valid text file containing the show_tracking_error for all the frames in the source should also be present;
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'tracking_err_type'
		Description:		
			method used for computing the tracking error; 
		Possible Values:
			0: Mean Corner Distance or MCD error - mean euclidean distance between the corners of the two bounding boxes
			1: Center Location Error or CLE - euclidean distance between the centroids of the two bounding boxes
			2: Jaccard Error - ration of intersection to union between the two bounding boxes
			
	 Parameter:	'record_frames'
		Description:
			record the tracked frames into a video file called Tracked_video.avi; 
			enabling this may significantly decrease the overall tracking speed
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'pause_after_frame'
		Description:
			pause tracking after each frame; 
			pressing space bar will resume tracking; 
			pressing any other key (except Esc) will move to next frame (Esc will exit the program);
		Possible Values:
			0: Disable
			1: Enable		
			
	 Parameter:	'reinit_at_each_frame'
		Description:
			reinitialize tracker from ground truth at each frame so that tracking is only done from frame to frame;
			only works when a dataset sequence is used and its ground truth is available;
			normally used for testing tracker on synthetic sequences;
			
	 Parameter:	'reset_at_each_frame'
		Description:
			reset tracker to the ground truth at each frame; 
			unlike the previous option, here the template remains unchanged;
			only works when a dataset sequence is used and its ground truth is available;
			
	 Parameter:	'reinit_on_failure'
		Description:
			reinitialize tracker when it fails, i.e. when its MCD/Jaccard/CL error goes above err_thresh; 
			only works when a dataset sequence is used and its ground truth is available;	
			
	 Parameter:	'reinit_err_thresh'
		Description:
			tracking error threshold at which the tracker is reinitialized;
			only matters if 'reinit_on_failure' is enabled;
			
	 Parameter:	'reinit_frame_skip'
		Description:
			no. of frames to skip before reinitializing tracker when it fails;
			only matters if 'reinit_on_failure' is enabled;

	 Parameter:	'use_reinit_gt'
		Description:
			use reinitialization ground truth instead of the normal one;
			this only differs from normal ground truth is optimized low DOF ground truth is being used, i.e. if use_opt_gt is enabled too;
			this can be generated from the normal ground truth using generateReinitGTByOptimization.py script in PTF;
			
	 Parameter:	'reinit_gt_from_bin'
		Description:
			read reinitialization ground truth from binary file instead of ASCII text files;
			this can be generated from the normal ground truth using generateReinitGTByOptimization.py script in PTF;
			
MTF Tracker specific parameters:
================================
	 Parameter:	'mtf_sm'
		Description:
			Search method to use for the MTF tracker or the name of the detection based tracker
		Possible Values:
			ic/iclk:	Inverse Compositional Lucas Kanade
				icl/iclm:	use Levenberg Marquardt (LM) formulation
			fc/fclk:	Forward Compositional Lucas Kanade
				fcl/fclm:	use LM formulation
			fa/falk:	Forward Additive Lucas Kanade
				fal/falm:	use LM formulation
			ia/ialk:	Inverse Additive Lucas Kanade
				ial/ialm:	use LM formulation
			esm:	Efficient Second-order Minimization
				esl/eslm:	use LM formulation
			
			Note:	LM formulation can also be enabled for all of the above SMs by setting leven_marq to 1
			
			pf:	Particle filter 
				pfic:	cascade tracker with PF+ICLK
				pffc:	cascade tracker with PF+FCLK
				pfes:	cascade tracker with PF+NESM
				pfk:	k layer PF - k can be set using pfk_n_layers
				pfkic:	cascade tracker with k layer PF + ICLK - k can be set using pfk_n_layers
				pfkfc:	cascade tracker with k layer PF + FCLK - k can be set using pfk_n_layers
				pfkes:	cascade tracker with k layer PF + ESM - k can be set using pfk_n_layers
				pfrk:	cascade tracker with PF+RKLT
			if NN is not disabled during compilation:
				nn:	Nearest Neighbour (based on FLANN)
				nnic:	cascade tracker with NN+ICLK
				nnfc:	cascade tracker with NN+FCLK
				nnes:	cascade tracker with NN+NESM
				nnk:	k layer NN - k can be set using nnk_n_layers
				nnkic:	cascade tracker with k layer NN + ICLK - k can be set using nnk_n_layers
				nnkfc:	cascade tracker with k layer NN + FCLK - k can be set using nnk_n_layers
				nnkes:	cascade tracker with k layer NN + ESM - k can be set using nnk_n_layers
				nnrk:	cascade tracker with NN+RKLT
			gnn:	Graph based Nearest Neighbour
				this is implemented as a special case of the general NN tracker so can also be run by setting 'mtf_sm' to 'nn' and 'nn_index_type' to 0
			Following SMs have only NT implementations:
				aesm:	Additive formulation of ESM 
				fcgd:	Forward Compositional Gradient Descent
			casc:	general cascade tracker whose configuration is read from multi.cfg
			casm:	cascade of SMs whose configuration is read from multi.cfg
			prl/prlt:	Parallel tracker whose configuration is read from multi.cfg
			prsm/prls:	Parallel SM whose configuration is read from multi.cfg
			pyr/pyrt:	Pyramidal tracker - construct a Gaussian image pyramid and track each level with a different tracker of the same type
				pyr_sm in modules.cfg specifies the search method in the underlying tracker
			pysm/pyrs:	Pyramidal search method - identical to Pyramidal tracker except all constituents SMs must have same AM and SSM;
			grid:	Grid Tracker
				setting grid_sm in modules.cfg to cv will run the OpenCV version of this tracker
			rkl/rklt:	RKLT (Grid tracker + template tracker with SPI and failure detection)
			hrch:	Hierarchical SSM tracker - uses same SM ('hrch_sm') and AM with four different SSMs - 2, 4, 6 and 8 dof that are run in a cascade
			if learning based trackers are not disabled during compilation:
				dsst:	Discriminative Scale Space Tracker 
				kcf:	Kernelized Correlation Filter Tracker 
				cmt:	Consensus-based Tracker 
				tld:	Tracking-Learning-Detection Tracker
				rct:	Realtime Compressive Tracker
				mil:	Multiple Instance Learning based tracker
				strk:	Struck: structured output tracking with kernels
				dft:	Descriptor Fields Tracker
				frg:	Fragments based tracker (or FragTrack)
			if ViSP template tracker module is enabled during compilation:
				visp:	ViSP template tracker
			if PFSL3 template tracker module is enabled during compilation:
			If Xvision is enabled during compilation:			
				xv1 / xv1p:	XVSSD Rotate / Pyramidal version
				xv2 / xv1p:	XVSSD Translation / Pyramidal version
				xv3 / xv1p:	XVSSD RT / Pyramidal version
				xv4 / xv1p:	XVSSD SE2 / Pyramidal version
				xv6 / xv1p:	XVSSD Affine / Pyramidal version
				xvc:	XVColor tracker
				xve:	XVEdge tracker
				xvg:	XV Grid tracker			
				xvgl:	XV Grid Line tracker			
			Note: tracker specific parameters including those for the learning based trackers are specified in modules.cfg		
			
	 Parameter:	'mtf_am'
		Description:
			Appearance model to use for the MTF tracker
		Possible Values:
			ssd:	Sum of Squared Differences
			    mcssd/ssd3: RGB variant
			sad:	Sum of Absolute Differences
			    mcsad/sad3: RGB variant
			zncc:	Zero mean Normalized Cross-Correlation
			    mczncc/zncc3: RGB variant
			nssd:	Normalized SSD
			    mcnssd/nssd3: RGB variant
			ncc:	Normalized Cross-Correlation
			    mcncc/ncc3: RGB variant
			scv:	Sum of Conditional Variance
			    mvscv/scv3: RGB variant
			rscv:	Reversed Sum of Conditional Variance
			    mcrscv/rscv3: RGB variant
			lscv:	Localized SCV
			lrscv:	Localized RSCV
			mi:	Mutual Information
			    mcmi/mi3: RGB variant
			ccre:	Cross Cumulative Residual Entropy
			    mcccre/ccre3: RGB variant
			ssim:	Structural Similarity
			    mcssim/ssim3: RGB variant
			spss:	Sum of Pixel wise Structural Similarity
			    mcspss/spss: RGB variant
			riu:	Ratio Image Uniformity
			    mcriu/riu3: RGB variant
			ngf: Normalized Gradient Fields
			kld:	Kullback–Leibler Divergence (does not work well)
			lkld:	Localized KLD (incomplete/does not work well yet)
			if enabled during compilation:
				pca: Principal Components Analysis
				dfm: Deep Feature Maps
				
			
	 Parameter:	'mtf_ssm'
		Description:
			State space model to use for the MTF tracker
		Possible Values:
			lhom or l8:	Lie Homography (8 dof)
			sl3: an alternative Lie parameterization of homography with different basis
			hom or 8:	Homography (8 dof)
			chom or c8:	Corner based Homography (8 dof)
			aff or 6:	Affine (6 dof)	
			sim or 4:	Similarity (translation + rotation + isotropic scaling)(4 dof)
			iso or 3:	Isometry (translation + rotation)(3 dof)
			trs or 3s:	Transcaling (Translation + isotropic scaling)
			trans or 2:	Translation (2 dof)	
			spl:	Spline based SSM with piecewise translation
			
	 Parameter:	'mtf_ilm'
		Description:
			Illumination model to use with the AM
			    these currently only work with SSD like AMs - SSD, SCV, RSCV, LSCV, LRSCV and ZNCC
		Possible Values:
			0:	None
			gb:	Gain and Bias
			pgb:	Piecewise Gain and Bias
			rbf:	Radial Basis Function
			
	 Parameter:	'mtf_res':
		Description:
			single value for both vertical and horizontal sampling resolutions, if this is >=0, then resx=resy=mtf_res
			
	 Parameter:	'resx' / 'resy'
		Description:
			horizontal and vertical sampling resolutions for extracting pixel values from the object patch;
			object pixel values are sampled from a rectangular grid of size resx x resy;
			higher values usually lead to better tracking performance but also slow it down; 
			these options only matter if mtf_res<=0	
			
	 Parameter:	'res_from_size'
		Description:
			set the horizontal and vertical sampling resolutions equal to the actual size of the object selected for tracking;
			overrides the last two parameters;
			
Efficient Second order Minimization:
====================================
	 Parameter:	'esm_jac_type'
		Description:
			type of Jacobian to be used with ESM
		Possible Values:
			0:	Original formulation where it is computed using the mean of gradients
			1: Extended formulation where it is the difference between the forward and inverse Jacobians
	 Parameter:	'esm_hess_type'
		Description:
			type of Hessian to be used with ESM
	 Parameter:	'esm_chained_warp'
		Description:
			use chain rule to compute pixel Jacobian and Hessian
		Possible Values:
			0:	Inverse/initial Self (or extended Gauss Newton) Hessian
			1:	Forward/current Self (or extended Gauss Newton) Hessian
			2:	Sum of forward and inverse Self (or extended Gauss Newton) Hessians
			3:	Original formulation where it is computed using the mean of gradients
			4:	Sum of forward and inverse Newton Hessians
			5:	Forward Newton Hessian
	 Parameter:	'esm_spi_enable'
		Description:
			Enable selective pixel integration by rejecting pixels whose residual is more than the given fraction of the maximum residual
	 Parameter:	'esm_spi_thresh'
		Description:
			Fraction of the maximum residual used as threshold for rejecting pixels;
			only matters if esm_spi_enable	is enabled;	

For Nearest Neighbour (NN) search method:
=========================================
	 Parameter:	'nn_max_iters'
		Description:
			maximum no. of iterations per frame 
	 Parameter:	'nn_n_samples'
		Description:
			no. of samples in the index/dataset that is searched for the nearest neighbour 
	 Parameter:	'nn_ssm_sigma_ids'
		Description:
			one or more numeric IDs (separated by commas with no spaces) corresponding to the list of "ssm_sigma" specified at the end of modules.cfg
			each specifies the standard deviation of the joint Gaussian distribution used for generating the random SSM parameters for the samples in the index; 
			normally a vector of the same size as the SSM state vector but may have special meaning/format depending on the specific SSM being used;
			only matters if nn_pix_sigma is <=0;
	 Parameter:	'nn_pix_sigma'
		Description:
			one or more standard deviations of displacement of corners in pixels produced by random SSM samples in the index; 
			a gradient based method is current used for converting this to the sigma for each SSM parameter and may not work well for some SSMs (especially more complex ones with higher DOFs);
			overrides the values specified in 'nn_ssm_sigma' unless it is <= 0;
	 Parameter:	'nn_ssm_mean_ids'
		Description:
			one or more numeric IDs (separated by commas with no spaces) that correspond to the list of "ssm_mean" specified at the end of modules.cfg (if any);
			these specify the means of the joint Gaussian distributions used for generating the random SSM parameters for each sample in the index;			
			normally a vector of the same size as the SSM state vector but may have special meaning/format depending on the specific SSM being used;
			any ID that is < 0 or > no. of specified ssm_mean vectors will be ignored; 
			if the number of IDs specified < that for "pf_ssm_sigma_ids", the last one is used for all remaining distributions; 
			therefore if no valid ID is specified here (either all IDs < 0 or no "ssm_mean" specified), all distributions will be zero mean;
	 Parameter:	'nn_index_type'
		Description:
			ID of the type of FLANN index to be built;			
		Possible Values:
			0:	GNN - this is not a FLANN provided index but is instead implemented within MTF;
			1:	KD Tree - only compatible with AMs whose similarity function is a kd-tree compatible distance, i.e.  full distance between a pair of features can be accumulated from the partial distances between the individual dimensions; 
			2:	Hierarchical Clustering - compatible with all AMs
			3:	KMeans (hierarchical k-means tree)
			4:	Composite 
			5:	Linear
			6:	KDTreeSingle
			7:	KDTreeCuda3d
			8:	Autotuned
			please refer FLANN manual(www.cs.ubc.ca/~mariusm/uploads/FLANN/flann_manual-1.6.pdf) for more details on index types 1-7 and this paper for index type 0: ijcai.org/papers11/Papers/IJCAI11-222.pdf;
	Parameter:	'nn_fgnn_index_type'
		Description:
			ID of the type of FLANN index to be used to build GNN index; set to 0 to disable the use of FLANN with GNN; only matters if nn_index_type is set to 0;
	Parameter:	'nn_save_index'
		Description:
			save the dataset and index to a binary file so it can be loaded again in a later run to avoid rebuilding;
	 Parameter:	'nn_load_index'
		Description:
			load the dataset and index from a previously saved binary file; if the file does not exist, it will revert to building the dataset and index instead;
	 Parameter:	'nn_show_samples'
		Description:
			show the location of the samples used for building the dataset;	
			non zero values for it specify the no. of samples to be shown simultaneously while zero disables it; 
			program execution will be paused after drawing these many samples and can be continued by pressing any key; 
			pressing space will disable the pausing after each time these many samples are drawn while escape will turn off the showing samples option;
	 Parameter:	'nn_add_points'
		Description:
			add the searched sample in each frame to the dataset indexed with the SSM parameters corresponding to its nearest neighbour;
			does not work with GNN at present;
	 Parameter:	'nn_remove_points'
		Description:
			remove the sample corresponding to the nearest neighbour found in each frame;
			does not work with GNN at present;
			
Multi Layer NN:
================
	 Parameter:	'nnk_n_layers'
		Description:
			no. of layers in the cascade setup;
	 Parameter:	'nnk_ssm_sigma_ids'
		Description:
			similar to nn_ssm_sigma_ids except it must be provided for each layer of the tracker separately;

For Particle Filter (PF) search method:
=======================================
	 Parameter:	'pf_max_iters'
		Description:
			maximum no. of iterations per frame 
	 Parameter:	'pf_n_particles'
		Description:
			no. of particles used for searching for the optimal SSM parameters; 
	 Parameter:	'pf_ssm_sigma_ids'
		Description:
			one or more numeric IDs separated by commas (NO spaces) that correspond to the list of "ssm_sigma" vectors specified at the end of modules.cfg; must be <= no. of such vectors;
			these specify the standard deviations of the joint Gaussian distributions used for generating the random SSM parameters for each particle;			
			normally a vector of the same size as the SSM state vector but may have special meaning/format depending on the specific SSM being used;
			if the number of IDs specified < that for "pf_ssm_mean_ids", the last one is used for all remaining distributions so that the number of samplers used is maximum of the lengths of these two arguments; 
			only matters if pf_pix_sigma is <=0;
			only NT version of PF supports multiple samplers so the standard version will simply use the first value specified here;
	 Parameter:	'pf_ssm_mean_ids'
		Description:
			one or more numeric IDs separated by commas (NO spaces) that correspond to the list of "ssm_mean" specified at the end of modules.cfg (if any);
			these specify the means of the joint Gaussian distributions used for generating the random SSM parameters for each particle;			
			normally a vector of the same size as the SSM state vector but may have special meaning/format depending on the specific SSM being used;
			any ID that is < 0 or > no. of specified ssm_mean vectors will be ignored; 
			if the number of IDs specified < that for "pf_ssm_sigma_ids", the last one is used for all remaining distributions; 
			therefore if no valid ID is specified here (either all IDs < 0 or no "ssm_mean" specified), all distributions will be zero mean;
	 Parameter:	'pf_pix_sigma'
		Description:
			one or more standard deviations for displacement of corners in pixels produced by each sampler used for generating the random SSM samples corresponding to different particles; 
			a gradient based method is current used for converting this to the sigma for each SSM parameter and may not work well for some SSMs (especially more complex ones with higher DOFs);
			overrides the values specified in 'pf_ssm_sigma' unless it is set to 0;
			only NT version of PF supports multiple samplers so the standard version will simply use the first value specified here;
	 Parameter:	'pf_update_sampler_wts'
		Description:
			update the proportion of samples taken from different sampler in each frame according to the weights of the samples generated by each
	 Parameter:	'pf_min_particles_ratio'
		Description:
				fraction of the total particles that will always be evenly distributed between the samplers; only the proportion of remaining particles will be adjusted dynamically;
				only matters if "pf_update_sampler_wts" is enabled
	 Parameter:	'pf_measurement_sigma'
		Description:
			standard deviation for introducing randomness to the measurement function;
			only matters if pf_likelihood_func is set to 1;
	 Parameter:	'pf_n_particles'
		Description:
			no. of particles used for searching for the optimal SSM parameters; 
	 Parameter:	'pf_dynamic_model'
		Description:
			dynamic model used for generating random perturbed SSM parameters for different particles;
		Possible Values:
			0:	Random Walk
			1:	First Order Auto Regression1
	 Parameter:	'pf_update_type'
		Description:
			method used for combining a state perturbation with existing SSM parameter values to update the particle states;
		Possible Values:
			0:	Additive
			1:	Compositional
	 Parameter:	'pf_likelihood_func'
		Description:
			method used for generating the likelihood of the patch corresponding to each particle;
		Possible Values:
			0:	use the getLikelihood() function of the AM
			1:	Gaussian - use an exponential function on the negative similarity
			2:	Reciprocal of the similarity
	 Parameter:	'pf_resampling_type'
		Description:
			method used for re sampling the particles to avoid degeneration
			refer this paper for more details on these methods: R. Douc and O. Cappe, "Comparison of resampling schemes for particle filtering," ISPA 2005. Proceedings of the 4th International Symposium on Image and Signal Processing and Analysis, 2005., 2005, pp. 64-69. 
		Possible Values:
			0:	Disable re sampling
			1:	Binary Multinomial
			2:	Linear Multinomial			
			3:	Residual	
	 Parameter:	'pf_mean_type'
		Description:
			method used for generating the weighted mean of all the particles which serves as the overall state of the tracker;
		Possible Values:
			0:	No averaging is done and the state of the particle with the highest weight is simply used instead
			1:	use the dedicated function provided by the SSM
			2:	take the mean of the bounding box corners corresponding to all the particles as the overall bounding box corners specifying the state of the tracker
	 Parameter:	'pf_reset_to_mean'
		Description:
			reset the states of all particles to their mean state for each frame;
	 Parameter:	'pf_mean_of_corners'
		Description:
			use the mean of corners corresponding to different particles to compute the mean location and use the corresponding SSM parameters as the mean state;	
			if disabled, the function "estimateMeanOfSamples()" provided by the SSM is used instead;
	 Parameter:	'pf_show_particles'
		Description:
			show the location of the patch corresponding to each particle;	
			non zero values for it specify the no. of particles to be shown simultaneously while zero disables it; 
			program execution will be paused after drawing the locations of these many particles and can be continued by pressing any key; 
			pressing 'space' will disable the pausing after each time these many particle locations are drawn while 'escape' will turn off the showing samples option;

Multi Layer PF:
================
	 Parameter:	'pfk_n_layers'
		Description:
			no. of layers in the cascade setup;
	 Parameter:	'pfk_ssm_sigma_ids'
		Description:
			similar to 'pf_ssm_sigma_ids' except it must be provided for each layer of the tracker separately;
			
For GridTracker and RKLT:
========================= 
	 Parameter:	'grid_sm' / 'grid_am' / 'grid_ssm'
		Description:
			Search method, appearance model and state space model for the individual patch trackers used by the Grid Tracker
				providing 'cv' for grid_sm will run GridTrackerCV that uses OpenCV KLT trackers instead of MTF trackers as patch trackers
				providing 'pyr' for grid_sm will cause each patch tracker to run on an image pyramid, the settings for which will be taken from the parameters for PyramidalTracker; 
					a time saving measure employed in this case is to construct the image pyramid only once and share it amongst all the patch trackers;
			
	 Parameter:	'grid_grid_res'
		Description:
			resolution of the grid into which the object is divided so that no. of patch trackers = grid_grid_res*grid_grid_res
			
	 Parameter:	'grid_patch_size'
		Description:
			sampling resolution for each patch tracker	
			
	 Parameter:	'grid_dyn_patch_size'
		Description:
			set to 1 to dynamically adjust the patch sizes based on the size of the overall object bounding box by dividing it evenly; 
			the individual patches in this case are no longer rectangular;
		
	 Parameter:	'grid_init_at_each_frame'
		Description:
			set to 1 to reinitialize the patch trackers at each frame based on the estimated location of the larger bounding box
			
	 Parameter:	'grid_show_trackers'
		Description:
			set to 1 to show the locations of all the patch trackers within the larger object patch	where each is marked by the location of its centroid
			
	 Parameter:	'grid_show_tracker_edges'
		Description:
			set to 1 to also show the edges of the bounding box representing each patch tracker (in addition to its centroid)
			
	 Parameter:	'grid_use_tbb'
		Description:
			set to 1 to enable parallelization 	of the patch trackers using Intel TBB library
			
For RKLT:
=========
	 Parameter:	'rkl_sm'
		Description:
			SM for the template tracker used by RKLT; the corresponding AM and SSM are specified by 'mtf_am' and 'mtf_ssm' respectively;
			
	 Parameter:	'rkl_enable_spi'
		Description:
			enable selective pixel integration where the template tracker is updated using only those pixels that are deemed inliers by the robust estimation method;
			this only works if both the AM and SSM of the template tracker support SPI;	
			if enabled, the sampling resolution of the template tracker is set equal to the grid size (as specified by 'gt_grid_res');
			
	 Parameter:	'rkl_enable_feedback'
		Description:
			reset the grid tracker to the location of the template tracker at each frame;
			
	 Parameter:	'rkl_failure_detection'
		Description:
			set to 1 to check if the template tracker has failed and ignore its output if so; 
			this check is done by comparing the L2 norm of the difference in corners provided by the template tracker and the grid tracker with the failure threshold (specified by 'rkl_failure_thresh') 			
	 
	 Parameter:	'rkl_failure_thresh'
		Description:
			threshold to decide if the template tracker has failed; only matters if 'rkl_failure_detection' is enabled
			
In addition to these parameters, the performance of GridTracker/RKLT is also affected by the following SSM estimator parameters.
		
For SSM Estimator:
==================
	 Parameter:	'est_method'
		Description:
			method used to estimate the best fit SSM parameters between the two sets of points representing the centroids of the locations of the patch trackers in two consecutive frames
		Possible Values:
			0: RANSAC
			1: Least Median
			2: LeastSquares
			
	 Parameter:	'est_ransac_reproj_thresh'
		Description:
			re-projection error threshold for a point to be considered an outlier by the OpenCV RANSAC  algorithm; 
			only matters if this method is selected for grid_estimation_method;
			
	 Parameter:	'est_n_model_pts'
		Description:
			no. of corresponding points to use for least square estimation of SSM parameters
			
	 Parameter:	'est_max_iters'
		Description:
			maximum iterations for the robust estimator (RANSAC or Least Median)
			
	 Parameter:	'est_confidence'
		Description:
			confidence threshold for the robust estimator 
			
	 Parameter:	'est_refine'
		Description:
			refine the parameters estimated by the robust method using a few iterations of Levenberg Marquardt algorithm 
			
	 Parameter:	'est_lm_max_iters'
		Description:
			no. of iterations to use for the optional Levenberg Marquardt refinement step if it is enabled; 

			
For NSSD AM:
============
	 Parameter:	'norm_pix_min' / 'norm_pix_max'
		Description:
			minimum and maximum values within which to normalize the pixel values

For SCV/RSCV/LSCV/LRSCV AMs:
=========================== 
	 Parameter:	'scv_use_bspl'
		Description:
			use BSpline kernel of order 3 while computing the joint histogram that is used for computing the sum of conditional variance; the Dirac Delta function is used otherwise;
			
	 Parameter:	'scv_n_bins'
		Description:
			number of bins in the joint histogram
			
	 Parameter:	'scv_preseed'
		Description:
			value with which to preseed the histograms; this can be set to non zero values while using BSpline histograms to avoid some numerical issues associated with empty bins that can sometimes occur;
			
	 Parameter:	'scv_pou'
		Description:
			strictly enforce the partition of unity constraint for border bins while computing the BSpline joint histogram
			
For MI AM:
==========
	 Parameter:	'mi_n_bins' / 'mi_preseed' / 'mi_pou'
		Description:
			only for MI appearance model; 
			meaning is same as the corresponding parameters for SCV;
			
For Pyramidal Tracker:
======================
	 Parameter:	'pyr_sm'
		Description:
			SM for the underlying tracker; its AM and SSM are taken from mtf_am and mtf_ssm respectively;
			
	 Parameter:	'pyr_no_of_levels'
		Description:
			no. of levels in the image pyramid;
			
	 Parameter:	'pyr_scale_factor'
		Description:
			ration between the image sizes in consecutive levels of the pyramid;
			for instance if the main image is 800x600 and there are 3 levels in the pyramid, a scale factor of 0.5 means that level 2 and 3 have image sizes of 400x300 and 200x150 respectively;
			
	 Parameter:	'pyr_show_levels'
		Description:
			show the image for each level in the pyramid annotated with th tracker's location;
	 
	