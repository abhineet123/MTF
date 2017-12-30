Input parameters can be specified in 6 plain text files called **mtf.cfg**, **modules.cfg**, **examples.cfg**, **thirdparty.cfg**, **sigma.cfg** and **multi.cfg** (located in this folder by default) where each line specifies the value of one parameter as: `<param_name><tab><param_val>`. If these files are present in some other folder, its path can be specified at runtime as `runMTF config_dir <directory containing the cfg files>`

Note that all parameters described here or present in **mtf.cfg**, **modules.cfg**, **examples.cfg** and **thirdparty.cfg** can be specified in either of these files since **all files are read sequentially** - this split is done only for convenience so that mtf.cfg contains parameters relevant to the tracking task in general or common to multiple modules/examples/trackers and the other three files contain parameters specific to MTF modules, example applications and third party trackers respectively; if a parameter is specified in multiple files, its value in a later file will override that in the earlier ones;
**sigma.cfg** specifies a list of standard deviations and means along with respective IDs that can be used as input arguments (in modules.cfg and examples.cfg) for stochastic modules like NN, PF and RegNet to specify the Gaussian distributions from where these will draw samples;

**multi.cfg** specifies configurations for individual trackers in multi tracker setups like CascadeTracker/ParallelTracker for single object tracking or when tracking multiple objects simultaneously.
The parameters that can be specified here are same as in the last two files and will override the values specified there for each specific tracker thus enabling different trackers to have independent settings.
The settings for two trackers should be separated by an empty line.  Also note that the changes made by specifying parameters in **multi.cfg** are global, i.e. if the value specified for a specific parameter for one tracker will be used for all subsequent trackers too unless overridden again.

**Note: If the value of any parameter is prefixed by #, it is ignored and its default value in parameters.h, if any, is used instead. Similarly any line that starts with # is treated as a comment and hence ignored. Also argument names are supposed to start with a letter so any numbers or special characters at the beginning of any argument name (except # of course) are ignored and only the part starting at the first letter is considered as the name. This can be used to assign numeric IDs to arguments if needed (e.g. as done in sigma.cfg)**

The parameters can also be specified from the command line through a list of argument pairs as follows:

	runMTF <arg_name_1> <arg_val_1> <arg_name_2> <arg_val2> .... <arg_name_n> <arg_val_n>
	
where the valid values of `arg_name` and `arg_val` are same as in the cfg files - these arguments will override the values specified in those files in case both are provided;
any invalid values for `arg_name` will be ignored along with its `arg_val`;

Following are some of important parameters, their brief descriptions and possible values:

Input parameters:
=================
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
				USB camera option(u) can be used to access Firewire cameras with OpenCV as long as no USB cameras are attached too)
			p: PointGrey Firewire camera (accessed using the FlyCapture SDK - only ViSP pipelines; 
			
	 Parameter:	'actor_id'
		Description:
			integral index of the dataset (or "actor") to use out of the possible datasets hard coded in datasets.h;
			used in conjunction with the parameter 'seq_id'  to get the source name;
			only matters if both are non negative
		Possible Values:
				0: TMT
				1: UCSB
				2: LinTrack
				3: PAMI
				4: TFMT
				5: METAIO
				6: CMT
				7: VOT
				8: VOT16
				9: VTB
				10: VIVID
				11: TrakMark
				12: LinTrackShort
				13: Mosaic
				14: Misc
				15: Synthetic
				16: Live
			Note: the first 10 datasets in a ready to use format can be downloaded from the MTF website: http://webdocs.cs.ualberta.ca/~vis/mtf/

	 Parameter:	'db_root_path'
		Description:
			location of the root directory that contains the files for all datasets (or 'actors');
			for JPEG file the full path is constructed as: db_root_path/actor/seq_name/*.seq_fmt;
			for MPEG/Video file the full path is constructed as: db_root_path/actor/seq_name.seq_fmt;
				
	 Parameter:	'seq_id'/'source_id'
		Description:
			integral index of the sequence name to use out of the sequences hard coded in datasets.h;
			used in conjunction with the parameter 'actor_id' to get the sequence name;
			only matters if both are non negative
		Possible Values:
			refer datasets.h for details of what each index means for each actor type; 
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
				
	 Parameter:	'seq_name'/'source_name'
		Description:
			name of the input video file (for MPEG source) or folder (for JPEG source); 
				overridden if both the parameters 'seq_id' and 'actor_id' are non-negative;
				ignored if a camera stream is being used;
			
	 Parameter:	'seq_path'/'source_path'
		Description:
			only matters for Xvision pipeline and camera streams; 
			specifies the path of the camera device to use
		Possible Values:
			depend on the number and types of camera attached to the system;
			following are common values for the two camera types:
				/dev/video0:	USB
				/dev/fw1:	Firewire
				for OpenCV pipeline with USB camera source, it should be an integer ID specifying the camera (e.g. 0 for the first camera, 1 for the second camera and so on.)
			
	 Parameter:	'seq_fmt'/'source_fmt'
		Description:
			file extension for image and video file streams;
			any special formatting strings to be passed to the Xvision initializer for camera streams;
		Possible Values:
			jpg:	JPEG image files
			mpg:	MPEG video file
			avi:	AVI	video file (only OpenCV pipeline)
			if XVision pipeline is used with a camera source, this can be used to pass any formatting strings specifying the resolution, FPS and other relevant options.		

			
	 Parameter:	'pre_proc_type'
		Description:
			type of filtering to use to pre process the raw input images before feeding them to the trackers
		Possible Values:
			-1/raw:	Disable pre processing - raw images acquired by input pipeline are passed to the tracker unchanged 
			    this will only work if the raw image format (typically CV_8UC3 or 3 channel 8 bit unsigned integral) is compatible with that required by the tracker;
			    even when no filtering is used (pre_proc_type = 0), the pre processing module still converts the input images to the correct channels and precision;
			0/none:	No filtering
			1/gauss:	Gaussian filtering
			2/med:	Median filtering
			3/box:	Box Average filtering
			4/bil:	Bilinear filtering	
			5/aniso:	Anisotropic Diffusion (also called Perona-Malik Diffusion: https://en.wikipedia.org/wiki/Anisotropic_diffusion)	
			6/sobel:	Sobel filtering	
			
	 Parameter:	'pre_proc_hist_eq'
		Description:
			perform histogram equalization as part of pre processing
		Possible Values:
			0: Disable (default)
			1: Enable		
			
	 Parameter:	'uchar_input'
		Description:
			use 8 bit unsigned integral images of type CV_8UC1/CV_8UC3 as input to trackers rather than 32 bit floating point variants (CV_32FC1/CV_32FC3) 
			enabling this can provide a small increase in speed but sometimes at the cost of a slight loss in precision especially if filtering or gray scale conversion is enabled
		Possible Values:
			0: Disable (default)
			1: Enable
	 
	 Parameter:	'invert_seq'
		Description:
			invert the input sequence, i.e. read its images in the reverse order with the last image being read first and the first one last;
			ignored if a camera stream is being used;
		Possible Values:
			0: Disable (default)
			1: Enable
			
	 Parameter:	'img_resize_factor'
		Description:
			factor by which the input images are resized before being used for tracking; this is the size ratio of the tracked image to the input image so that a value smaller than 1 means that the tracked image will be smaller than the input one.

	 Parameter:	'input_buffer_size'
		Description:
			no. of frames read and stored in the buffer in advance.		
			
	 Parameter:	'read_obj_from_file'
		Description:
			read initial location of the object to be tracked from the text file specified by 'read_obj_fname' where they were previously written to by enabling 'write_objs';
			this is meant to avoid having to specify a custom initialization locations for one or more trackers repeatedly
		Possible Values:
			0: Disable (default)
			1: Enable
			
	 Parameter:	'read_obj_fname'
		Description:
			name of the text file where the initial location of the object to be tracked will be read from;
			only matters if read_objs is 1
			
	 Parameter:	'read_obj_from_gt'
		Description:
			read initial object location from a ground truth file present in the same directory as the input source file;
			matters only if a file stream is being used; 
			the format of this file should be identical to the ground truth files for the TMT dataset available here:
				http://webdocs.cs.ualberta.ca/~vis/trackDB/firstpage.html
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'use_opt_gt'
		Description:
			use optimized low DOF ground truth instead of the normal one;
			this can be generated from the normal ground truth using generateGTByOptimization.py or generateReinitGTByOptimization.py scripts in PTF;
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'opt_gt_ssm'
		Description:
			SSM corresponding to the optimized low DOF ground truth that is to be used;
			only matters if 'use_opt_gt' is enabled;
			
	 Parameter:	'use_reinit_gt'
		Description:
			use reinitialization ground truth instead of the normal one;
			this only differs from normal ground truth is optimized low DOF ground truth is being used, i.e. if use_opt_gt is enabled too;
			this can be generated from the normal ground truth using generateReinitGTByOptimization.py script in PTF;
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'reinit_gt_from_bin'
		Description:
			read reinitialization ground truth from binary file instead of ASCII text files;
			this can be generated from the normal ground truth using generateReinitGTByOptimization.py script in PTF;
		Possible Values:
			0: Disable
			1: Enable
		Applies to:
			runMTF
			
	 Parameter:	'sel_quad_obj'
		Description:
			select an arbitrary quadrilateral as the bounding box that defines the object to be tracked; this involves selecting the 4 corners by clicking at each; if disables, a rectangular object is selected instead by clicking on its two opposite corners;
			only matters if a live/camera video stream is used or object selection from ground truth is disabled;
		Applies to:
			all examples
			
ViSP pipeline:
==============
			
	 Parameter:	'vp_usb_n_buffers'
		Description:
			No. of buffers to use for USB camera source
			
	 Parameter:	'vp_usb_res'
		Description:
			Image resolution for USB camera source
		Possible Values:
			0: Default
			1: 640 x 480
			2: 800 x 600
			3: 1024 x 768
			4: 1280 x 720
			5: 1920 x 1080
			
	 Parameter:	'vp_usb_fps'
		Description:
			Video speed in frames per second (FPS) for USB camera source
		Possible Values:
			0: Default
			1: 25
			2: 50
			
	 Parameter:	'vp_fw_fps'
		Description:
			Video speed in frames per second (FPS) for Firewire or PointGrey camera source
		Possible Values:
			0: Default
			1: 15
			2: 30
			3: 60
			4: 120
			5: 240
			6: 7.5 (only PointGrey)
			7: 3.75 (only PointGrey)
			8: 1.875 (only PointGrey)
			
	 Parameter:	'vp_fw_res'
		Description:
			Image resolution for Firewire or PointGrey camera source
		Possible Values:
			0: Default
			1: 640 x 480
			2: 800 x 600
			3: 1024 x 768
			4: 1280 x 960
			5: 1600 x 1200
			
	 Parameter:	'vp_fw_depth'
		Description:
			Image color depth / format for Firewire or PointGrey camera source			
		Possible Values:
			0: Default
			1: RGB
			2: YUV422
			3: Mono 8 bit 
			4: Mono 16 bit
			
	 Parameter:	'vp_fw_iso'
		Description:
			ISO mode for Firewire source			
		Possible Values:
			0: Default
			1: ISO100
			2: ISO200
			3: ISO400
			4: ISO800
			5: ISO1600
			6: ISO3200
			
	 Parameter:	'vp_fw_print_info'
		Description:
			Print detailed info for Firewire or PointGrey camera source	
			
	 Parameter:	'vp_pg_fw_shutter_ms'
		Description:
			Shutter speed in ms for PointGrey camera source			
		Possible Values:
			0: Default
			< 0: auto
			> 0: manual
			
	 Parameter:	'vp_pg_fw_gain'
		Description:
			Gain for PointGrey camera source			
		Possible Values:
			0: Default
			< 0: auto
			> 0: manual
			
	 Parameter:	'vp_pg_fw_exposure'
		Description:
			Exposure for PointGrey camera source			
		Possible Values:
			0: Default
			< 0: auto
			> 0: manual	
			
	 Parameter:	'vp_pg_fw_brightness'
		Description:
			Brightness for PointGrey camera source
		Possible Values:
			0: Default
			< 0: auto
			> 0: manual				
		
Output parameters:
==================

	 Parameter:	'show_cv_window'
		Description:
			show the result of tracking from frame to frame in an OpenCV window;
			if a single object is being tracked, this is shown as a red bounding box; subsequent objects are shown in other colours defined in Utilities/src/objUtils.cc;
			disabling it can speed up the overall tracking speed by eliminating the delay caused by drawing the object locations on the current frame;
			useful for benchmarking and batch mode testing;
		Possible Values:
			0: Disable
			1: Enable	
			
	 Parameter:	'show_xv_window'
		Description:
			show the result of tracking from frame to frame in an Xvision window;
			only matters if Xvision is enabled during compilation and Xvision pipeline is used;
		Possible Values:
			0: Disable
			1: Enable	
			
	 Parameter:	'show_ground_truth'
		Description:
			show the current location of the object in the ground truth in an OpenCV window;
			this is always shown as a green color bounding box;
			only matters if show_cv_window is enabled and the conditions required for ground truth to be available as specified for show_tracking_error are satisfied;
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'show_tracking_error'
		Description:
			show the the tracking error between the tracking result and the ground truth in the OpenCV window; 
			the metric used for computing this error is specified in tracking_err_type;
			only matters if read_objs_from_gt is enabled and a file input source (video or image) is used; 
			a valid text file containing the ground truth for all the frames in the source should also be present;
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'show_jaccard_error'
		Description:
			compute and show the the Jaccard error between the tracking result and the ground truth in the OpenCV window; 
			only matters if show_tracking_error is enabled and the conditions specified for it to work are satisfied; 
			this can be useful if both MCD/CLE and Jaccard error need to be shown together for comparative analysis;
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'show_proc_img'
		Description:
			Show the pre processed images in a separate window.
		Possible Values:
			0: Disable
			1: Enable
			
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
		
	 Parameter:	'print_corners'
		Description:
			show the x,y coordinates of the corners of the bounding region (usually a box with 4 corners) representing the tracker location on the terminal		
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'print_fps'
		Description:
			show the current tracking speed in frames per second (FPS)	on the terminal	
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'write_tracking_data'
		Description:
			write tracking result, i.e. the coordinates of the bounding box/region representing the tracked object location in each frame to a text file;
			only matters if a single object is being tracked;
			only supported in runMTF;
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'overwrite_gt'
		Description:
			overwrite ground truth file with the tracking result; only matters if 'write_tracking_data' is enabled and a single object is being tracked;
			can be useful when tracking is used to generate or refine ground truth (e.g. in coordination with refineGroundTruth.py script in PTF);
			only supported in runMTF;
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'tracking_data_fname'
		Description:
			name of the file to which the tracking result is written; if not specified, then a name is generated from the SM, AM and SSM used to construct the tracker;
			only matters if 'write_tracking_data' is enabled, a single object is being tracked and 'overwrite_gt' is disabled;
			only supported in runMTF;
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'write_objs'
		Description:
			write the manually selected initial object location to the text file specified by 'write_obj_fname';
			only matters if manual selection is enabled by disabling both 'read_objs' and 'read_obj_from_gt';
			this is meant to avoid having to specify a custom initialization locations for one or more trackers repeatedly
		Possible Values:
			0: Disable (default)
			1: Enable
			
	 Parameter:	'write_obj_fname'
		Description:
			name of the text file where the initial location of the object to be tracked will be written to;
			only matters if manual selection is enabled by disabling both 'read_objs' and 'read_obj_from_gt' and enabling 'write_objs'
			
	 Parameter:	'write_tracking_error'
		Description:
			write all 3 types of tracking errors - MCD, CLE and Jaccard - to a text file;
			only matters if a single object is being tracked and ground truth is available;
			only supported in runMTF;
		Possible Values:
			0: Disable
			1: Enable
	 
	 Parameter:	'record_frames'
		Description:
			record the frames containing the output of the executable into a video file - this is currently used only in runMTF and showGroundTruth
		Possible Values:
			0: Disable
			1: Enable		
			
	 Parameter:	'record_frames_fname'
		Description:
			name of the video file into which the output frames are recorded; if this is not specified, a name is constructed from some relevant parameters
			
	 Parameter:	'record_frames_dir'
		Description:
			path of the directory where the recorded video file is saved; if this is not specified, a folder called "log" in the current working folder is used; this folder is created automatically if it does not exist
			
	 Parameter:	'tracker_labels'
		Description:
			optional label to attach to each tracked object; if multiple objects are being tracked, labels can be provided for each separated by commas; if not specified, then the tracker name is used by default;
			only used in runMTF;
			
	 Parameter:	'line_thickness'
		Description:
			thickness of the line used for drawing the bounding box that represents the current location of the tracker and/or the ground truth
		Applies to:
			all examples
			
	 Parameter:	'show_corner_ids'
		Description:
			show the numerical ID of each bounding box corner along with drawing the box itself; these IDs are 0-based and increase anti clockwise starting from the top left corner;
			note that this convention is defined with respect to the initial orientation of the object; if the object rotates in the course of being tracked, the corner with ID 0 might no longer remain in the top left of its new orientation;
		Applies to:
			all examples
			
General parameters for example executables:
===========================================

	 Parameter:	'debug_mode'
		Description:
			enable debug messages in some of the modules - this is a rather obsolete parameter that will soon be removed; several modules have their own debug mode parameters in modules.cfg
	 
	 Parameter:	'enable_nt'
		Description:
			use the non templated (NT) implementation of the SM; this is enabled automatically if the templated versions are disabled during compilation;
	 
	 Parameter:	'frame_gap'
		Description:
			gap between consecutive frames that are used for tracking; this can be used to skip frames from the input pipeline, for example, to simulate fast motion;
			
	 Parameter:	'invalid_state_check'
		Description:
			enable checking if the location provided by the tracker is valid where a valid location is defined as one that does not contain any NaN or Inf values and, if the ground truth is available, whose error w.r.t. the ground truth location is smaller than the value specified in 'invalid_state_err_thresh'; the program exits if tracking state is found to be invalid;
			
	 Parameter:	'invalid_state_err_thresh'
		Description:
			maximum error of the tracked location w.r.t. the ground truth for the tracking state to be considered valid; the method used for computing the error is specified in 'tracking_err_type';
			
	 Parameter:	'init_frame_id'
		Description:
			id of the frame at which the tracker is to be initialized in case tracking is desired to be started in the middle of the sequence rather than the beginning;
		Possible Values:
			should be between 0 and no_of_frames-1	
			
	 Parameter:	'start_frame_id'
		Description:
			id of the frame after which the tracking actually starts; can be used to start tracking in the middle of the sequence but still initialize in the first frame;
			only matters if it is greater than init_frame_id;
			only works with trackers that have setRegion function implemented (none of the third party trackers currently do); also the ground truth must be available at least up to the starting frame;
		Possible Values:
			should be between 0 and no_of_frames-1	
			
	 Parameter:	'end_frame_id'
		Description:
			id of the frame at which tracking is terminated
		Possible Values:
			should be greater than init_frame_id and start_frame_id and less than no_of_frames;		
			
	 Parameter:	'tracking_err_type'
		Description:		
			method used for computing the tracking error; 
		Possible Values:
			0: Mean Corner Distance or MCD error - mean euclidean distance between the corners of the two bounding boxes
			1: Center Location Error or CLE - euclidean distance between the centroids of the two bounding boxes
			2: Jaccard Error - ratio of intersection to union between the two bounding boxes
			
	 Parameter:	'reinit_at_each_frame'
		Description:
			reinitialize tracker from ground truth at each frame so that tracking is only done from frame to frame;
			if it is > 1, reinitialization is done after the specified number of frames rather than at each frame;
			only works when a dataset sequence is used and its ground truth is available;			
			normally used for testing tracker on synthetic sequences or for filling in incomplete ground truth;
			
	 Parameter:	'reset_at_each_frame'
		Description:
			reset tracker to the ground truth at each frame; 
			if it is > 1, resetting is done after the specified number of frames rather than at each frame;
			unlike the previous option, here the template remains unchanged;
			only works when a dataset sequence is used and its ground truth is available;
			only matters if reinit_at_each_frame is 0;
			
	 Parameter:	'reset_to_init'
		Description:
			reset tracker to the ground truth location at each frame; 
			if it is > 1, resetting is done after the specified number of frames rather than at each frame;
			unlike the previous option, here the template remains unchanged;
			only works when a dataset sequence is used and its ground truth is available;
			only matters if reinit_at_each_frame is 0;
		Applies to:
			runMTF
			
	 Parameter:	'reinit_on_failure'
		Description:
			reinitialize tracker when it fails, i.e. when its MCD/Jaccard/CL error goes above err_thresh; 
			only works when a dataset sequence is used and its ground truth is available;	
		Possible Values:
			0: Disable
			1: Enable
		Applies to:
			runMTF
			
	 Parameter:	'reinit_err_thresh'
		Description:
			tracking error threshold at which the tracker is reinitialized;
			only matters if 'reinit_on_failure' is enabled;
		Applies to:
			runMTF
			
	 Parameter:	'reinit_frame_skip'
		Description:
			no. of frames to skip before reinitializing tracker when it fails;
			only matters if 'reinit_on_failure' is enabled;
		Applies to:
			runMTF
			
	 Parameter:	'reinit_with_new_obj'
		Description:
			delete the old tracker object and create a new one when reinitializing the tracker;
			"tracker object" here refers to the instance of class TrackerBase (or one of its derived classes) that was used for performing the tracking;
			some third party trackers have been known to crash randomly when an existing instance is reinitialized so this solution can be used with them;
		Possible Values:
			0: Disable
			1: Enable
		Applies to:
			runMTF	
			
	 Parameter:	'reset_template'
		Description:
			update the template with the patch under the current location of the tracker; this effectively reinitializes the tracker with its own current location; in some cases, this option can help to continue tracking an object even when its appearance undergoes significant changes; more often, however, it is likely to lead to gradual tracker drift as small inaccuracies in the tracked location get compounded when this location is used to reinitialize the tracker;
			if a value > 1 is specified, the template resetting only happens once every given number of frames rather than at every frame;
		Applies to:
			runMTF, mexMTF, pyMTF
			
	 Parameter:	'n_trackers'
		Description:
			number of trackers to run; if this is more than 1, the configuration of all trackers are read from multi.cfg;
			
	 Parameter:	'track_single_obj'
		Description:
			track the same object using all the trackers (if n_trackers > 1); if this is disabled a different object will have to be selected for each tracker;
		Possible Values:
			0: Disable
			1: Enable			

	 Parameter:	'patch_size'
		Description:
			if non-zero, the this is taken to be the size of the object to be tracked so that the user can simply click at the center to select a square bounding box of this size around the clicked point; if this is zero, the user must provide all 4 corners of the bounding box; only matters if a user-selected object is to be tracked, i.e. when using live/camera input or when the ground truth is not available or if read_obj_from_file and read_obj_from_gt are disabled;

			
Tracker specific parameters:
============================
	 Parameter:	'mtf_sm'
		Description:
			Search method to use for the MTF tracker or the name of the third party tracker
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
			feat:	Feature Tracker
				only works if this was enabled during compilation ()with feat=1 switch)
			rkl/rklt:	RKLT (Grid tracker + template tracker with SPI and failure detection)
			hrch:	Hierarchical SSM tracker - uses same SM ('hrch_sm') and AM with four different SSMs - 2, 4, 6 and 8 dof that are run in a cascade
			if third party trackers are not disabled during compilation:
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
			if PFSL3 is enabled during compilation:
				pfsl3: 8 DOF PF based tracker that uses SL3 parameterization
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
			        mcpca/pca3: RGB variant
				dfm: Deep Feature Maps
				
			
	 Parameter:	'mtf_ssm'
		Description:
			State space model to use for the MTF tracker
		Possible Values:
			hom/8:	Homography (8 dof)
			lhom/l8:	Homography with Lie parametrization(8 dof)
			sl3: Homography with an alternative Lie parametrization using a different basis
			cbh/c8:	Corner based Homography (8 dof)
			aff/6:	Affine (6 dof)	
			laff/l6:	Affine with Lie parametrization(6 dof)	
			asrt/5:	ASRT (anisotropic scaling + rotation + translation) (5 dof)
			sim/4:	Similarity (isotropic scaling + rotation + translation)(4 dof)
			ast/4s:	AST (anisotropic scaling + translation) (4 dof)
			iso/3:	Isometry (rotation + translation)(3 dof)
			ist/3s:	IST (isotropic Scaling  + translation) (3 dof)
			trans/2:	Translation (2 dof)	
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

			
	 Parameter:	'enable_learning'
		Description:
			enable online learning in AMs that support it - only SSD and NCC currently do; online learning means that the template changes over time based on the observed appearances of the tracked object; this is automatically enabled in AMs, like PCA, that involve learning as an essential part;
			
	 Parameter:	'learning_rate'
		Description:
			rate at which the template is updated;
			varies between 0 and 1 - 0 means that there is no learning; 1 means that the template is updated to the latest patch in each frame;
			
	 Parameter:	'likelihood_alpha'
		Description:
			multiplicative factor for computing the exponential factor in the likelihood value for an AM; the actual formulation for computing the likelihood will depend on the specific AM and not all AMs may use this parameter;
			
	 Parameter:	'likelihood_beta'
		Description:
			additive factor for computing the exponential factor in the likelihood value for an AM; the actual formulation for computing the likelihood will depend on the specific AM and not all AMs may use this parameter;
			
	 Parameter:	'dist_from_likelihood'
		Description:
			compute the distance measure (e.g. as used by the NN search method) using the likelihood value; this can help to decrease the range of distance values so that the corresponding plot has a sharp peak;
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'pix_mapper'
		Description:
			pixel mapper to use with one of the AMs that support it including SSIM and SPSS - this is one of the SSD-like or L2 AMs that define the similarity between two patches as the L2 norm of mapped versions of the two patches; by selecting one of them here, the same mapping can be combined with a non-L2 norm based AM; this is rather obsolete and has not been tested for a while so might possibly be buggy;
			
		Possible Values:
			scv, rscv, zncc
		
	 Parameter:	'max_iters'
		Description:
			maximum no. of iterations for which the iterative LK type SMs are allowed to run on each frame;
			
	 Parameter:	'epsilon'
		Description:
			minimum Euclidean distance between the object locations in consecutive iterations to be used as the termination criterion for iterative LK type SMs - the iterations are terminated if the distance becomes smaller than this;
	
	 Parameter:	'grad_eps'
		Description:
			offset used for computing the numerical estimate of the first order image gradient (or the Jacobian); this is the distance(in x or y direction) between the pixel locations that are used in the method of central differences; a value of <1 will probably not work with nearest neighbour interpolation method as rounding off errors will cause the gradient to vanish;
			
	 Parameter:	'hess_eps'
		Description:
			offset used for computing the numerical estimate of the second order image gradient (or the Hessian); this is the distance(in x or y direction) between the pixel locations that are used in the method of central differences; values that are <1 have not been found to provide stable results irrespective of the interpolation method;
			
	 Parameter:	'sec_ord_hess'
		Description:
			use second order Hessian in Lucas Kanade type SMs that use some variant of the Newton's method; if disabled, the first order approximation is used where the terms involving second order image and SSM gradients are dropped leading to much faster performance; in most cases, the first order Hessian also performs better;
		Possible Values:
			0: Disable
			1: Enable
			
	 Parameter:	'leven_marq'
		Description:
			use Levenberg-Marquardt formulation for computing the Hessian in Lucas Kanade type SMs that use some variant of the Newton's method; if disabled, the simpler Gauss-Newton formulation is used instead;
		Possible Values:
			0: Disable
			1: Enable
			
Affine SSM:
===========
	 Parameter:	'aff_normalized_init'
		Description:
			use normalized initial bounding box with respect to which all subsequent transformations are computed;
			the normalized box is a unit square centered at the origin;
			using this can sometimes produce better performance with some LK type SMs;
			
	 Parameter:	'aff_pt_based_sampling'
		Description:
			use point based sampling for stochastic SMs; this is performed by adding a random perturbation to the x,y coordinates of three points - bottom left, bottom right and top center of the bounding box - and then computing the corresponding affine transformation w.r.t. to the original points using DLT;
			if this is disabled, then geometric warping is used where the affine transformation is decomposed into six constituent transforms and a random perturbation is added to each;
		Possible Values:
			0: Disable (use geometric sampling)
			1: Enable (use standard point based sampling)
			2: modified version of point based sampling where the perturbation is performed in two steps: first 6 different random numbers generated from the same distribution are added to the x,y coordinates of the three points, then 2 numbers generated from a second distribution are added to all points to produce a consistent translation; 
		Additional References:
			This paper presents details of the geometric sampling process: D. A. Ross, J. Lim, R.-S. Lin, and M.-H. Yang, “Incremental Learning for Robust Visual Tracking,” IJCV, vol. 77, no. 1-3, pp. 125–141, 2008

Anisotropic Diffusion Preprocessor:
===================================
	 Parameter:	'aniso_lambda'
		Description:
			integration constant
			
	 Parameter:	'aniso_kappa'
		Description:
			gradient modulus threshold that controls the conduction
			
	 Parameter:	'aniso_n_iters'
		Description:
			number of iterations
			
	 Additional References:
		Paper:
			'Scale-Space and Edge Detection using Anisotropic Diffusion', Pietro Perona and Jitendra Malik, IEEE Transactions on Pattern Analysis and Machine Intelligence, VOL. 12, NO. 7, JULY 1990
		Web:
			http://ishankgulati.github.io/posts/Anisotropic-(Perona-Malik)-Diffusion/
			https://www.mathworks.com/matlabcentral/fileexchange/14995-anisotropic-diffusion--perona---malik-

Anisotropic Scaling, Rotation and Translation (ASRT) SSM:
=========================================================
	 Parameter:	'asrt_normalized_init'
		Description:
			use normalized initial bounding box with respect to which all subsequent transformations are computed;
			refer 'aff_normalized_init' for more details;
		
	 Parameter:	'asrt_pt_based_sampling'
		Description:
			use point based sampling for stochastic SMs; refer 'aff_pt_based_sampling' for details;
			geometric sampling in this case involves adding different perturbation directly to the 5 state parameters of this SSM unlike affine where a specialized geometric representation is used instead;

Bilateral Filtering Preprocessor:
================================
	 Parameter:	'bil_diameter'
		Description:
			diameter of each pixel neighbourhood that is used during filtering. If it is non-positive, it is computed from bil_sigma_space.
			
	 Parameter:	'bil_sigma_col'
		Description:
			filter sigma in the color space. A larger value of the parameter means that farther colors within the pixel neighborhood will be mixed together, resulting in larger areas of semi-equal color.
			
	 Parameter:	'bil_sigma_space'
		Description:
			filter sigma in the coordinate space. A larger value of the parameter means that farther pixels will influence each other as long as their colors are close enough. When bil_diameter>0 , it specifies the neighborhood size regardless of bil_sigma_space. Otherwise, bil_diameter is proportional to bil_sigma_space.
			
	 Additional References:
		https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html#bilateralfilter
		
		
Normalized Box Filtering Preprocessor:
======================================
	 Parameter:	'box_kernel_size'
		Description:
			filter kernel size		
		
	 Additional References:
		https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html#blur
		
Cascade Tracker/SM:
===================
	 Parameter:	'casc_n_trackers'
		Description:
			no. of trackers in the cascade
			
	 Parameter:	'casc_enable_feedback'
		Description:
			feed the output of the last tracker in the cascade back to the first tracker to use as the starting point in the next frame;
			
	 Parameter:	'casc_auto_reinit'
		Description:
			enable tracker failure detection and automatic reinitialization; tracking failure is assumed to have occurred if the result of any tracker contains non-finite values or the MCD error between the outputs of any two consecutive layer exceeds a threshold;
			reinitialization is done using the tracker location in a previous frame as specified by casc_reinit_frame_gap;
			
	 Parameter:	'casc_reinit_err_thresh'
		Description:
			MCD error threshold between two consecutive trackers in the cascade above which tracking failure is assumed to have occured;
			only matters if casc_auto_reinit is enabled;
			
	 Parameter:	'casc_reinit_frame_gap'
		Description:
			no. of frames before the one in which failure is detected where the tracker is reinitialized;

Corner Based Homography (CBH) SSM:
==================================
	 Parameter:	'cbh_grad_eps'
		Description:
			offset used for computing the numerical estimate of the first gradient (or Jacobian) of the transformation w.r.t. pixel locations; this is the distance(in x or y direction) between the pixel locations that are used in the method of central differences; 
			
	 Parameter:	'cbh_normalized_init'
		Description:
			use normalized initial bounding box with respect to which all subsequent transformations are computed;
			refer 'aff_normalized_init' for more details;
			
Cross Cumulative Residual Entropy (CCRE) AM:
============================================
	 Parameter:	'ccre_n_bins'
		Description:
			no. of bins in the histograms used internally - dimensionality of the CCRE error vector will be n_bins * n_bins; 
			if partition_of_unity ('ccre_pou') is enabled, this should be 2 more than the desired no. of bins (w.r.t normalized pixel range);
			this is because the actual range within which the pixel values are normalized is 2 less than this value to avoid boundary conditions while computing the contribution of each pixel to different bins by ensuring that pixels with the maximum and minimum values contribute to all 4 bins required by the b-spline function of degree 3 used here;
			
	 Parameter:	'ccre_pre_seed'
		Description:
			value with which each histogram bin is pre-seeded to avoid empty bins and the resultant numerical stability issues;
			
	 Parameter:	'ccre_pou'
		Description:
			decides whether the partition of unity constraint has to be strictly observed for border bins;
			if enabled, the pixel values will be normalized in the range [1, n_bins-2] so each pixel contributes to all 4 bins.
			
	 Parameter:	'ccre_symmetrical_grad'
		Description:
			decides if the model is to be symmetrical with respect to initial and current pixel values as far as the gradient and hessian computations are concerned;	
		Additional References:
			section 5.2.3.1 of the thesis
			
	 Parameter:	'ccre_n_blocks'
		Description:
			no. of blocks in which to divide pixel level computations to get better performance with parallelization libraries like TBB and OpenMP; only matters if these are enabled during compilation;
			if set to 0 (default), this is set equal to the no. of pixels so that each block contains a single pixel
	 
	 Additional References:
		Wang, F. & Vemuri, B. C. Non-rigid multi-modal image registration using cross-cumulative residual entropy IJCV, Springer, 2007, 74, 201-215			

Deep Feature Maps (DFM) AM:
===========================
	 Parameter:	'dfm_nfmaps'
		Description:
			no. of feature maps 
			
	 Parameter:	'dfm_layer_name'
		Description:
			TBA
			
	 Parameter:	'dfm_vis'
		Description:
			TBA
			
	 Parameter:	'dfm_zncc'
		Description:
			TBA
			
	 Parameter:	'dfm_model_f_name'
		Description:
			path of the .prototxt file from where trained model is to be loaded
			
	 Parameter:	'dfm_params_f_name'
		Description:
			path of the .caffemodel file from where Caffe parameters are to be loaded
			
	 Parameter:	'dfm_mean_f_name'
		Description:
			path of the .binaryproto file
			
	 Additional References:
		Mennatullah Siam, "CNN Based Appearance Model with Approximate Nearest Neigbour Search", Project Report, 2016
		http://webdocs.cs.ualberta.ca/~vis/mtf/dfm_report.pdf
		
Efficient Second order Minimization (ESM) SM:
=============================================
	 Parameter:	'esm_jac_type'
		Description:
			type of Jacobian to be used with ESM
		Possible Values:
			0: Original formulation where it is computed using the mean of gradients
			1: Extended formulation where it is the difference between the forward and inverse Jacobians
			
	 Parameter:	'esm_hess_type'
		Description:
			type of Hessian to be used with ESM
		Possible Values:
			0:	Inverse/initial Self (or extended Gauss Newton) Hessian
			1:	Forward/current Self (or extended Gauss Newton) Hessian
			2:	Sum of forward and inverse Self (or extended Gauss Newton) Hessians
			3:	Original formulation where it is computed using the mean of gradients
			4:	Sum of forward and inverse Newton Hessians
			5:	Forward Newton Hessian
			
	 Parameter:	'esm_chained_warp'
		Description:
			use chain rule to compute pixel Jacobian and Hessian
			
	 Parameter:	'esm_spi_enable'
		Description:
			Enable selective pixel integration by rejecting pixels whose residual is more than the given fraction of the maximum residual

	 Parameter:	'esm_spi_thresh'
		Description:
			Fraction of the maximum residual used as threshold for rejecting pixels;
			only matters if esm_spi_enable	is enabled;	
			
Forward Additive Lucas Kanade (FALK) SM:
========================================
	 Parameter:	'fa_hess_type'
		Description:
			type of Hessian to be used with FALK
		Possible Values:
			0:	Inverse/initial Self (or extended Gauss Newton) Hessian
			1:	Forward/current Self (or extended Gauss Newton) Hessian
			2:	Standard or Forward Newton Hessian
			
	 Parameter:	'fa_show_grid'
		Description:
			show the sampled grid of points warped using the current transformation;
			these are the (integral approximation to) points at which pixel values are extracted;
			only works with NT version of the SM;
			
	 Parameter:	'fa_show_patch'
		Description:
			show the image patch corresponding to the tracked object in the top left corner of the image;
			only matters if 'fa_show_grid' is enabled;
			only works with NT version of the SM;

			Parameter:	'fa_patch_resize_factor'
		Description:
			multiplicative factor by which the image patch is resized before being drawn onto the image;
			by default, the patch size is equal to the sampling resolution so this parameter can be used to make the patch more visible;
			only matters if 'fa_show_grid' and 'fa_show_patch' are enabled;	
			only works with NT version of the SM;
			
	 Parameter:	'fa_write_frames'
		Description:
			write the tracked frames with the point grid and image patch drawn onto it to JPEG files;
			only matters if 'fa_show_grid' is enabled;
			only works with NT version of the SM;
			
Forward Compositional Lucas Kanade (FCLK) SM:
=============================================
	 Parameter:	'fc_hess_type'
		Description:
			type of Hessian to be used with FCLK
		Possible Values:
			0:	Inverse/initial Self (or extended Gauss Newton) Hessian
			1:	Forward/current Self (or extended Gauss Newton) Hessian
			2:	Standard or Forward Newton Hessian
			
	 Parameter:	'fc_chained_warp'
		Description:
			use chain rule to compute pixel Jacobian and Hessian
			
	 Parameter:	'fc_show_grid'/'fc_show_patch'/'fc_patch_resize_factor'
		Description:
			same as the corresponding parameters for FALK;
			only work with NT version of the SM;

	 Parameter:	'fc_write_ssm_updates'
		Description:
			write the SSM state update computed in each iteration of each frame to a text file named 'fc_ssm_updates.txt' in a sub directory called 'log' in the current working directory;
			only works with NT version of the SM;
			
	 Parameter:	'fc_debug_mode'
		Description:
			write additional debugging data to a text file named 'fc_debug.txt' in a sub directory called 'log' in the current working directory;
			only works with NT version of the SM;
			
Feature Tracker:
================
	 Parameter:	'feat_detector_type'
		Description:
			feature detector type
		Possible Values:
			0:	None - an equally spaced grid of points is used instead
			1:	SIFT
			2:	SURF
			3:	BRISK
			4:	ORB
			5:	FAST
			6:	MSER
			7:	GFTT
			Only OpenCV 3:
			8:	AGAST
			
	 Parameter:	'feat_descriptor_type'
		Description:
			feature descriptor type
		Possible Values:
			0:	SIFT
			1:	SURF
			2:	BRISK
			3:	ORB
			
	 Parameter:	'feat_max_dist_ratio'
		Description:
			maximum ratio between the distances of each point from the first and second best matched points from the previous frame for this point to be considered for computing the best fit transformation;
			if this is <0, all points are considered;
			
	 Parameter:	'feat_min_matches'
		Description:
			minimum no. of good matching key points found for the best fit transformation to be computed otherwise matching is considered to have failed;
			
	 Parameter:	'feat_rebuild_index'
		Description:
			rebuild the FLANN index in each frame with the latest feature descriptors;
			only matters if FLANN is enabled during compilation and feat_use_cv_flann is disabled;
			
	 Parameter:	'feat_use_cv_flann'
		Description:
			use OpenCV FLANN wrapper for keypoint matching;
			if this is disabled, the FLANN library is used directly but that will work only if FLANN is enabled during compilation;
			
	 Parameter:	'feat_show_keypoints'
		Description:
			show all detected keypoints overlaid on each tracked image;
			
	 Parameter:	'feat_show_matches'
		Description:
			show the matches between the keypoints in the current and previous frames
			
	 Parameter:	'feat_debug_mode'
		Description:
			enable printing and writing of debugging data	
			
	 Note:	
		these Grid Tracker parameters are shared by the feature tracker: 'grid_res', 'grid_patch_size', 'grid_reset_at_each_frame';
		in addition, 'max_iters' and 'epsilon' are also shared;
			
Gaussian Filtering Preprocessor:
================================
	 Parameter:	'gauss_kernel_size'
		Description:
			Gaussian kernel size; this is a single number so that only square kernels are supported;
			
	 Parameter:	'gauss_sigma_x'
		Description:
			Gaussian kernel standard deviation in X direction
			
	 Parameter:	'gauss_sigma_y'
		Description:
			Gaussian kernel standard deviation in Y direction
			
	 Additional References:
		https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html#gaussianblur	
		
Gain & Bias (GB) ILM:
=====================
	 Parameter:	'gb_additive_update'
		Description:
			use additive instead of compositional updates for updating the ILM parameters;
			
Forward Compositional Steepest Descent:
=======================================
	 Parameter:	'sd_learning_rate'
		Description:
			learning rate for iteratively computing the SSM state update from the Jacobian;
			
			
GridTracker and RKLT:
===================== 
	 Parameter:	'grid_sm' / 'grid_am' / 'grid_ssm' / 'grid_ilm'
		Description:
			Search method, appearance model, state space model and illumination model for the individual patch trackers used by the Grid Tracker;
				providing 'cv' for grid_sm will run GridTrackerCV that uses OpenCV KLT trackers instead of MTF trackers as patch trackers
				providing 'flow' for grid_sm will run GridTrackerFlow that uses LK Optical flow implementation provided by the AM; currently supported only by SSD and NCC;
				providing 'pyr' for grid_sm will cause each patch tracker to run on an image pyramid, the settings for which will be taken from the parameters for PyramidalTracker; 
					a time saving measure employed in this case is to construct the image pyramid only once and share it amongst all the patch trackers;
			
	 Parameter:	'grid_res'
		Description:
			resolution of the grid into which the object is divided so that no. of patch trackers = grid_grid_res*grid_grid_res
			
	 Parameter:	'grid_patch_size'
		Description:
			size of the patch used for each sub tracker	in the grid
			
	 Parameter:	'grid_patch_res'
		Description:
			sampling resolution for each sub tracker; if it is set to 0, the sampling resolution is set equal to the patch size	
			
	 Parameter:	'grid_dyn_patch_size'
		Description:
			set to 1 to dynamically adjust the patch sizes based on the size of the overall object bounding box by dividing it evenly; 
			the individual patches in this case are no longer rectangular;
		
	 Parameter:	'grid_reset_at_each_frame'
		Description:
			specify the level of resetting applied to the patch trackers at each frame based on the estimated location of the larger bounding box;
		Possible Values:
			0:	No resetting
			1:	Reset both current location and template, i.e. reinitialize the trackers
			2:	Reset only current location	(not available for GridTrackerCV and GridTrackerFlow)	
			
	 Parameter:	'grid_patch_centroid_inside'
		Description:
			set to 1 to initialize/reset all patch trackers such that their centroids lie completely inside the larger bounding box;
			if set to 0, centroids of some of the trackers will lie on the edges of the bounding box so that part of the corresponding patches will be outside it;
			
	 Parameter:	'grid_fb_err_thresh'
		Description:
			error threshold for deciding tracking failures using the forward backward method;
			first the points are tracked from the previous to the current frame;
			results of this are then tracked back to the previous frame;
			error is measured between the initial points during forward tracking and the final points during backward tracking;
			setting this to <=0 disables this method of failure detection;		
			
	 Parameter:	'grid_fb_reinit'
		Description:
			reinitialize trackers in the current frame before tracking backwards to the last one;
			does not apply to OpenCV grid tracker;		
			
	 Parameter:	'grid_show_trackers'
		Description:
			set to 1 to show the locations of all the patch trackers within the larger object patch	where each is marked by the location of its centroid
			
	 Parameter:	'grid_show_tracker_edges'
		Description:
			set to 1 to also show the edges of the bounding box representing each patch tracker (in addition to its centroid)
			does not apply to OpenCV grid tracker;
			
	 Parameter:	'grid_use_tbb'
		Description:
			set to 1 to enable parallelization 	of the patch trackers using Intel TBB library
			
	 Parameter:	'grid_rgb_input'
		Description:
			set to 1 to use 3 channel RGB images as input to the OpenCV grid tracker;
			only matters if 'grid_sm' is set to 'cv';
			
	 Parameter:	'grid_pyramid_levels'
		Description:
			number of levels in the image pyramids used by the OpenCV grid tracker;
			only matters if 'grid_sm' is set to 'cv';
			
	 Parameter:	'grid_use_min_eig_vals'
		Description:
			use minimum eigen values as an error measure in the OpenCV grid tracker;	
			only matters if 'grid_sm' is set to 'cv';
			
	 Parameter:	'grid_min_eig_thresh'
		Description:
			threshold for minimum eigen value of a 2x2 normal matrix of optical flow equations to filter out grid points in the OpenCV grid tracker;
			only matters if 'grid_sm' is set to 'cv';
			more details of this and the previous parameter can be found at: http://docs.opencv.org/2.4/modules/video/doc/motion_analysis_and_object_tracking.html#calcopticalflowpyrlk

	 Parameter:	'grid_use_const_grad'
		Description:
			maintain the image gradient constant across iterations while computing the optical flow;
			only matters if 'grid_sm' is set to 'flow';
			
RKLT:
=====
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
		
SSM Estimator:
==============
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
			
	 Parameter:	'est_max_subset_attempts'
		Description:
			maximum attempts made to generate each valid point subset from which a candidate warp is estimated; 
			if none is found in these many attempts, the RANSAC or LMS procedure will be terminated and the best warp found so far will be returned;	
			
	 Parameter:	'est_use_boost_rng'
		Description:
			Use the random number generator in boost library rather than the one in OpenCV;		
			
	 Parameter:	'est_confidence'
		Description:
			confidence threshold for the robust estimator 
			
	 Parameter:	'est_refine'
		Description:
			refine the parameters estimated by the robust method using a few iterations of Levenberg Marquardt algorithm 
			
	 Parameter:	'est_lm_max_iters'
		Description:
			no. of iterations to use for the optional Levenberg Marquardt refinement step if it is enabled; 

Homography SSM:
===============
	 Parameter:	'hom_normalized_init'
		Description:
			use normalized initial bounding box with respect to which all subsequent transformations are computed;
			refer 'aff_normalized_init' for more details;
		
	 Parameter:	'hom_corner_based_sampling'
		Description:
			use corner based sampling for stochastic SMs; similar to 'aff_pt_based_sampling' except that the perturbations are applied to the 4 bounding box corners instead of the 3 points as in affine;
			also, geometric sampling in this case involves adding different perturbation directly to the 8 state parameters of this SSM unlike affine where a specialized geometric representation is used instead;
			
Hierarchical SSM tracker:
=========================
	 Parameter:	'hrch_sm' / 'hrch_sm'
		Description:
			AM and SSM for the hierarchical SSM tracker	

Inverse Additive Lucas Kanade (IALK) SM:
========================================
	 Parameter:	'ia_hess_type'
		Description:
			type of Hessian to be used with IALK
		Possible Values:
			0:	Inverse/initial Self (or extended Gauss Newton) Hessian
			1:	Forward/current Self (or extended Gauss Newton) Hessian
			2:	Standard or approximate Forward Newton Hessian
			
Inverse Compositional Lucas Kanade (ICLK) SM:
=============================================
	 Parameter:	'ic_hess_type'
		Description:
			type of Hessian to be used with ICLK
		Possible Values:
			0:	Inverse/initial Self (or extended Gauss Newton) Hessian
			1:	Forward/current Self (or extended Gauss Newton) Hessian
			2:	Standard or Inverse Newton Hessian
			
	 Parameter:	'ic_chained_warp'
		Description:
			use chain rule to compute pixel Jacobian and Hessian
			
	 Parameter:	'ic_update_ssm'
		Description:
			enable updating the SSM gradient in the setRegion function;
			only works with the templated version of the SM;
			
Isometry SSM:
=============
	 Parameter:	'iso_pt_based_sampling'
		Description:
			use point based sampling for stochastic SMs; refer 'aff_pt_based_sampling' for details;
			here the perturbations are applied to the two opposite bounding box corners and geometric sampling involves adding different perturbation directly to the 5 state parameters of this SSM unlike affine where a specialized geometric representation is used instead;
			
Lie Affine SSM:
===============
	 Parameter:	'laff_normalized_init'
		Description:
			use normalized initial bounding box with respect to which all subsequent transformations are computed;
			refer 'aff_normalized_init' for more details;
		
	 Parameter:	'laff_grad_eps'
		Description:
			offset used for computing the numerical estimate of the first gradient (or Jacobian) of the transformation w.r.t. pixel locations; this is the distance(in x or y direction) between the pixel locations that are used in the method of central differences; 
	
Lie Homography SSM:
===================
	 Parameter:	'lhom_normalized_init'
		Description:
			use normalized initial bounding box with respect to which all subsequent transformations are computed;
			refer 'aff_normalized_init' for more details;
		
	 Parameter:	'lhom_grad_eps'
		Description:
			offset used for computing the numerical estimate of the first gradient (or Jacobian) of the transformation w.r.t. pixel locations; this is the distance(in x or y direction) between the pixel locations that are used in the method of central differences; 
			
Localized Kullback-Leibler Divergence (LKLD) AM:
================================================
	 Parameter:	'lkld_n_bins' / 'lkld_pre_seed' / 'lkld_pou'
		Description:
			meaning is same as the corresponding parameters for CCRE;
			
	 Parameter:	'lkld_sub_regions'
		Description:
			size of the grid of subregions into which the image patch is divided so that the KL-Divergence is computed between each pair of corresponding sub-patches;
			e.g. 'lkld_sub_regions' of 3 means that the patch will be divided into a 3 x 3 grid of sub-regions for a total of 9 sub-patches
			
	 Parameter:	'lkld_spacing'
		Description:
			gap in pixels between adjacent subregions in both x and y directions, i.e. subregions in the same row are separated by this gap in the x direction while those in adjacent rows are separated by this gap in the y direction;

Levenberg Marquardt SM:
=======================
	 Parameter:	'lm_delta_init'
		Description:
			value with which the delta is initialized in each frame;
			
	 Parameter:	'lm_delta_update'
		Description:
			multiplicative factor by which the delta is modified in each iteration, i.e. if the current value leads to better performance then the delta is multiplied by this factor and if it leads to poorer performance, then it is divided by this factor;
			
	 Additional Reference:
		Baker, S. & Matthews, I., 'Lucas-Kanade 20 Years On: A Unifying Framework', IJCV, Kluwer Academic Publishers, 2004, 56, 221-255
	
Median Filtering Preprocessor:
==============================
	 Parameter:	'med_kernel_size'
		Description:
			Median filter kernel size; this is a single number so that only square kernels are supported;
		
	 Additional Reference:
		https://docs.opencv.org/2.4/modules/imgproc/doc/filtering.html#medianblur	
		
Normalized Cross Correlation (NCC) AM:
======================================
	 Parameter:	'ncc_fast_hess'
		Description:
			use an approximate version of the self Hessian that is faster to compute; this was originally implemented as a buggy version of the correct self Hessian but was found to perform just as well at lower computational cost;
			
Normalized Gradient Fields (NGF) AM:
====================================
	 Parameter:	'ngf_eta'
		Description:
			estimate of noise level in the image

	 Parameter:	'ngf_use_ssd'
		Description:
			use SSD formulation of NGF (not implemented completely yet); 
			
	 Additional References:
		Haber, E. & Modersitzki, J., 'Beyond mutual information: A simple and robust alternative', Bildverarbeitung für die Medizin 2005, Springer, 2005, 350-354
		Jan Modersitzki: FAIR - Flexible Algorithms for Image Registration, SIAM 2009
		https://github.com/C4IR/FAIR.m		
			
Mutual Information (MI) AM:
===========================
	 Parameter:	'mi_n_bins' / 'mi_preseed' / 'mi_pou'
		Description:
			meaning is same as the corresponding parameters for CCRE;
			
	 Additional Reference:
		Dame, A., 'A unified direct approach for visual servoing and visual tracking using mutual information', University of Rennes, 2010
			
Normalized Sum of Squared Differences (NSSD) AM:
================================================
	 Parameter:	'norm_pix_min' / 'norm_pix_max'
		Description:
			minimum and maximum values within which to normalize the pixel values		
		
Nearest Neighbour (NN) SM:
==========================
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
		Additional References:
			Muja, M. & Lowe, D. G., 'Fast Approximate Nearest Neighbors with Automatic Algorithm Configuration', VISAPP (1), 2009, 2, 331-340
			FLANN manual(www.cs.ubc.ca/~mariusm/uploads/FLANN/flann_manual-1.6.pdf) [index types 1-7]
			Hajebi, K.; Abbasi-Yadkori, Y.; Shahbazi, H. & Zhang, H., 'Fast approximate nearest-neighbor search with k-nearest neighbor graph', IJCAI Proceedings-International Joint Conference on Artificial Intelligence, 2011, 22, 1312 (ijcai.org/papers11/Papers/IJCAI11-222.pdf) [index type 0]
			
	Parameter:	'nn_fgnn_index_type'
		Description:
			ID of the type of FLANN index to be used to build GNN index; set to 0 to disable the use of FLANN with GNN; only matters if nn_index_type is set to 0;
			
	 Parameter:	'nn_search_type'
		Description:
			ID of the search type used for the FLANN index			
		Possible Values:
			0:	K-Nearest Neighbours (KNN) search
			1:	Radius search
		Additional References:
			sections 3.1.6 and 3.1.7 of FLANN manual (www.cs.ubc.ca/~mariusm/uploads/FLANN/flann_manual-1.6.pdf)
			
	Parameter:	'nn_save_index'
		Description:
			save the dataset and index to a binary file so it can be loaded again in a later run to avoid rebuilding;
			
	 Parameter:	'nn_load_index'
		Description:
			load the dataset and index from a previously saved binary file; if the file does not exist, it will revert to building the dataset and index instead;
			
	 Parameter:	'nn_additive_update'
		Description:
			use additive method to update SSM parameters instead of compositional one;
			
	 Parameter:	'nn_show_samples'
		Description:
			show the location of the samples used for building the dataset;	
			non zero values for it specify the no. of samples to be shown simultaneously while zero disables it; 
			program execution will be paused after drawing these many samples and can be continued by pressing any key; 
			pressing space will disable the pausing after each time these many samples are drawn while escape will turn off the showing samples option;
			
	 Parameter:	'nn_add_samples_gap'
		Description:
			gap between frames at which the index is updated with new samples;
			setting this to  0 disables the addition of samples;
			does not work with GNN at present;
			
	 Parameter:	'nn_n_samples_to_add'
		Description:
			no. of samples added to the index at each update;
			only matters if nn_add_samples_gap > 0;
			
	 Parameter:	'nn_remove_samples'
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
			similar to nn_ssm_sigma_ids except it must be provided for each layer of the tracker separately;er of steps for which the search for the best matching sample within the graph is carried out;
			
Graph based NN (GNN):
=====================
	 Parameter:	'nn_gnn_degree'
		Description:
			no. of neighbouring nodes with which each node in the graph is connected
			
	 Parameter:	'nn_gnn_max_steps'
		Description:
			Maximum number of steps for which the search for the best matching sample within the graph is carried out;
			
	 Parameter:	'nn_gnn_cmpt_dist_thresh'
		Description:
			minimum number of samples in the index for which the distance is computed during initialization between each pair of samples;
			if the number of samples is large enough, then this approach can help to reduce real-time computation costs by computing all possible distances at once, thus avoiding having to compute the distances between the same pair multiple times;
	
	 Parameter:	'nn_gnn_random_start'
		Description:
			always start each search at a random node within the graph
			if this is disabled, then the search is started and the node corresponding to the result of the previous search;
			for sequential tasks like tracking where the transformation of the object patch in each frame is likely to be very similar to that in the previous frame, disabling this can help to reduce the search time since the target nodes in consecutive frames are likely to be close to each other in the graph;

	 Parameter:	'nn_gnn_verbose'
		Description:
			print detailed debugging and other state related information at runtime	
			
	 Additional Reference:	
		Hajebi, K.; Abbasi-Yadkori, Y.; Shahbazi, H. & Zhang, H., 'Fast approximate nearest-neighbor search with k-nearest neighbor graph', IJCAI Proceedings-International Joint Conference on Artificial Intelligence, 2011, 22, 1312 (ijcai.org/papers11/Papers/IJCAI11-222.pdf)
			
			
FLANN based GNN (FGNN):
=======================
	 Parameter:	'nn_fgnn_index_type'
		Description:
			FLANN index type of the approximate NN search method used for building the graph;
			if set to 0, then FLANN based graph building in GNN is disabled;
			only matters if nn_index_type is set to 0 and templated version of NN is used;
			
	 Additional Reference:	
		section 4.2.1.1 of the thesis (http://webdocs.cs.ualberta.ca/~vis/mtf/mtf_thesis.pdf)
				
Principal Component Analysis (PCA) AM:
======================================
	 Parameter:	'pca_n_eigenvec'
		Description:
			no. of eigen vectors
			
	 Parameter:	'pca_batchsize'
		Description:
			no. of frames in each batch used for updating the basis
			
	 Parameter:	'pca_f_factor'
		Description:
			forgetting factor for updating the basis
			
	 Parameter:	'pca_show_basis'
		Description:
			show the image patches corresponding to the basis
			
	Additional References:
		D. A. Ross, J. Lim, R.-S. Lin, and M.-H. Yang, “Incremental Learning for Robust Visual Tracking,” IJCV, vol. 77, no. 1-3, pp. 125–141, 2008
			
Particle Filter (PF) SM:
========================
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
			refer this paper for more details on these methods:
			R. Douc and O. Cappe, "Comparison of resampling schemes for particle filtering," ISPA 2005. Proceedings of the 4th International Symposium on Image and Signal Processing and Analysis, 2005., 2005, pp. 64-69. 
		Possible Values:
			0:	Disable re sampling
			1:	Binary Multinomial
			2:	Linear Multinomial			
			3:	Residual	
			
	 Parameter:	'pf_adaptive_resampling_thresh'
		Description:
				maximum ratio between the number of effective particles and the	total particles for resampling to be performed;
				setting it to <=0 or >1 disables adaptive resampling;
				refer section III.D of this paper for details about adaptive resampling:
				Grisetti, Giorgio, Stachniss, Cyrill, and Burgard, Wolfram. “Improved techniques for grid mapping with Rao-Blackwellized particle filters.” IEEE transactions on Robotics 23.1 (2007): 34-46			
				
	 Parameter:	'pf_mean_type'
		Description:
			method used for generating the weighted mean of all the particles which serves as the overall state of the tracker;
		Possible Values:
			0:	No averaging is done and the state of the particle with the highest weight is simply used instead
			1:	use the dedicated function provided by the SSM
			2:	take the mean of the bounding box corners corresponding to all the particles as the overall bounding box corners specifying the state of the tracker
			
	 Parameter:	'pf_update_distr_wts'
		Description:
				update the proportion of samples taken from different sampler according to the weights of the samples generated by each;
				only works with NT version of the SM;
				
	 Parameter:	'pf_min_distr_wt'
		Description:
				fraction of the total particles that will always be evenly distributed between the samplers;
				only works with NT version of the SM;	
			
	 Parameter:	'pf_reset_to_mean'
		Description:
			reset the states of all particles to their mean state for each frame;
			
	 Parameter:	'pf_mean_of_corners'
		Description:
			use the mean of corners corresponding to different particles to compute the mean location and use the corresponding SSM parameters as the mean state;	
			if disabled, the function "estimateMeanOfSamples()" provided by the SSM is used instead;
			
	 Parameter:	'pf_jacobian_as_sigma'	 
		Description:
				compute the standard deviation of the Gaussian distribution from the Jacobian of the AM w.r.t. SSM parameters;
				only works with NT version of the SM;			
			
	 Parameter:	'pf_show_particles'
		Description:
			show the location of the patch corresponding to each particle;	
			non zero values for it specify the no. of particles to be shown simultaneously while zero disables it; 
			program execution will be paused after drawing the locations of these many particles and can be continued by pressing any key; 
			pressing 'space' will disable the pausing after each time these many particle locations are drawn while 'escape' will turn off the showing samples option;
			
	 Parameter:	'pf_debug_mode'
		Description:
			write additional debugging data to a text files named in a sub directory called 'log' in the current working directory as well as print some of it onto the terminal;
			only works with NT version of the SM;

Multi Layer PF:
================
	 Parameter:	'pfk_n_layers'
		Description:
			no. of layers in the cascade setup;
			
	 Parameter:	'pfk_ssm_sigma_ids'
		Description:
			similar to 'pf_ssm_sigma_ids' except it must be provided for each layer of the tracker separately;
			
Piecewise Gain & Bias (PGB) ILM:
================================
	 Parameter:	'pgb_additive_update'
		Description:
			use additive instead of compositional updates for updating the ILM parameters;	
			not implemented yet;
			
	 Parameter:	'pgb_sub_regions_x' / 'pgb_sub_regions_y'
		Description:
			size of the grid of subregions in x and y directions into which the image patch is divided;	
			
Parallel Tracker:
=================
	 Parameter:	'prl_n_trackers'
		Description:
			number of trackers to be run in parallel;
			
	 Parameter:	'prl_estimation_method'
		Description:
			method used for estimating the overall object location from the locations of the individual trackers;
			only works with ParallelSM;			
		Possible Values:
			0:	Mean Of Corners - overall bounding box corners are computed as a mean of the individual bounding boxes 
			1:	Mean Of State - overall SSM estate is computed as the element wise mean of the SSM states produced by the individual trackers;
			
	 Parameter:	'prl_reset_to_mean'
		Description:
			reset the locations of all the trackers to the overall object location in each frame;
			
	 Parameter:	'prl_auto_reinit' / 'prl_reinit_err_thresh' / 'prl_reinit_frame_gap'
		Description:
			same meaning as the corresponding parameters of cascade tracker;
	 
Pyramidal Tracker:
==================
	 Parameter:	'pyr_sm'
		Description:
			SM for the underlying tracker; its AM and SSM are taken from mtf_am and mtf_ssm respectively;
			
	 Parameter:	'pyr_no_of_levels'
		Description:
			no. of levels in the image pyramid;
			
	 Parameter:	'pyr_scale_factor'
		Description:
			ratio between the image sizes in consecutive levels of the pyramid;
			for instance if the main image is 800x600 and there are 3 levels in the pyramid, a scale factor of 0.5 means that level 2 and 3 have image sizes of 400x300 and 200x150 respectively;
			
	 Parameter:	'pyr_scale_res'
		Description:
			scale the sampling resolution for the tracker at each level of the pyramid;
	 
	 Parameter:	'pyr_show_levels'
		Description:
			show the image for each level in the pyramid annotated with the tracker's location;	 

Radial Basis Function (RBF) ILM:
================================
	 Parameter:	'rbf_additive_update'
		Description:
			use additive instead of compositional updates for updating the ILM parameters;	
			not implemented yet;
			
	 Parameter:	'rbf_n_ctrl_pts_x' / 'rbf_n_ctrl_pts_y'
		Description:
			size of the grid of control points in x and y directions that is used for computing the RBF surface;	
			
Regression networks (RG) SM:
============================
	 Parameter:	'rg_n_samples' / 'rg_load_index' / 'rg_save_index' / 'rg_ssm_sigma_ids' / 'rg_ssm_mean_ids' / 'rg_pix_sigma' / 'rg_additive_update' / 'rg_add_points' / 'rg_remove_points'
		Description:
			same as the corresponding parameters for NN
			
	 Parameter:	'rg_nepochs'
		Description:
			number of the epochs in training the network
			
	 Parameter:	'rg_bs'
		Description:
			number of the epochs in training the network
			
	 Parameter:	'rg_train'
		Description:
			path of the .caffemodel file from where Caffe parameters are to be loaded
			
	 Parameter:	'rg_solver'
		Description:
			path of the .prototxt file from where trained model is to be loaded
			
	 Parameter:	'rg_mean'
		Description:
			path of the .binaryproto file			
			
	 Parameter:	'rg_preproc'
		Description:
			enable preprocessing			
			
Sum of Conditional Variance (SCV)/Reversed SCV (RSCV)/Localized SCV (LSCV)/Localized RSCV (LRSCV) AMs:
===================================================================================================== 
	 Parameter:	'scv_n_bins'
		Description:
			number of bins in the joint histogram
			
	 Parameter:	'scv_preseed'
		Description:
			value with which to preseed the histograms; this can be set to non zero values while using BSpline histograms to avoid some numerical issues associated with empty bins that can sometimes occur;
			
	 Parameter:	'scv_pou'
		Description:
			strictly enforce the partition of unity constraint for border bins while computing the BSpline joint histogram
			
	 Parameter:	'scv_approx_dist_feat'
		Description:
			decides if true SCV will be computed between the distance features; if it is turned off then the joint distribution will be computed from the two arguments to the distance functor itself after which the first argument will be mapped according to the expectation formula; 
			note that using a KD tree index with this turned off will not produce the desired results because in that case this AM is no longer KD Tree compatible;
			not supported in LRSCV;
			
	 Parameter:	'scv_weighted_mapping'
		Description:
			enable this to map each intensity to the weighted average of the two entries of the intensity map corresponding to the floor and ceil of that intensity;
			if disabled, it will be mapped to the entry corresponding to its floor leading to some information loss due to the fractional part that was discarded
			
	 Parameter:	'scv_mapped_gradient'
		Description:
			enable this to automatically update the initial pixel gradient and hessian using the latest intensity map whenever the current pixel gradient is updated assuming that the current pixel values and thus the intensity map must have changed since the last time the initial pixel gradient was computed;
			only supported in SCV and RSCV;
			
	 Additional Reference:
		Richa, R.; Sznitman, R.; Taylor, R. & Hager, G. 'Visual tracking using the sum of conditional variance', IROS, 2011, 2953-2958

SCV:
====
	 Parameter:	'scv_hist_type'
		Description:
			method used for computing the joint histogram:
		Possible Values:
			0:	Dirac delta function that uses nearest neighbor interpolation
			1:	Bilinearr interpolation
			2:  Parzen density estimation with BSpline function of order 3 is used as the kernel function
RSCV:
=====
	 Parameter:	'scv_use_bspl'
		Description:
			use BSpline kernel of order 3 while computing the joint histogram that is used for computing the sum of conditional variance; the Dirac Delta function is used otherwise;
			
LSCV and LRSCV:
===============
	 Parameter:	'scv_affine_mapping'
		Description:
			use affine or linear mapping instead of the standard one;
			only supported in LSCV and LRSCV;
			
	 Parameter:	'scv_once_per_frame'
		Description:
			update the template only once per frame so that it remains constant across all iterations in that frame;
			this can help to speed up processing a bit at the cost of slight inaccuracy;
			only supported in LSCV and LRSCV;
			
	 Parameter:	'lscv_sub_regions' / 'lscv_spacing'
		Description:
			same meaning as the corresponding parameters for LKLD;
			
	 Parameter:	'lscv_show_subregions'
		Description:
			show the sub regions overlaid on the patch
			
	 Additional Reference:
		Richa, R.; Souza, M.; Scandaroli, G.; Comunello, E. & von Wangenheim, A. 'Direct visual tracking under extreme illumination variations using the sum of conditional variance', Image Processing (ICIP), 2014 IEEE International Conference on, 2014, 373-377			
			
SIFT feature detector and descriptor:
=====================================
	 Parameter:	'sift_n_features'
		Description:
			number of best features to retain. The features are ranked by their scores (measured in SIFT algorithm as the local contrast)
			
	 Parameter:	'sift_n_octave_layers'
		Description:
			number of layers in each octave. 3 is the value used in D. Lowe paper. The number of octaves is computed automatically from the image resolution.	 
			
	 Parameter:	'sift_contrast_thresh'
		Description:
			contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions. The larger the threshold, the less features are produced by the detector.	 
			
	 Parameter:	'sift_edge_thresh'
		Description:
			threshold used to filter out edge-like features. Note that the its meaning is different from the contrastThreshold, i.e. the larger the edgeThreshold, the less features are filtered out (more features are retained).	 
			
	 Parameter:	'sift_sigma'
		Description:
			sigma of the Gaussian applied to the input image at the octave #0. If your image is captured with a weak camera with soft lenses, you might want to reduce the number.	 
			
	 Additional References:
		https://docs.opencv.org/2.4/modules/nonfree/doc/feature_detection.html#sift-sift
		Lowe, D. G., 'Distinctive image features from scale-invariant keypoints', International journal of computer vision, Springer, 2004, 60, 91-110

SURF feature detector and descriptor:
=====================================
	 Parameter:	'surf_hessian_threshold'
		Description:
			Threshold for hessian keypoint detector used in SURF.
			
	 Parameter:	'surf_n_octaves'
		Description:
			Number of pyramid octaves the keypoint detector will use. 
			
	 Parameter:	'surf_n_octave_layers'
		Description:
			Number of octave layers within each octave.	 
			
	 Parameter:	'surf_extended'
		Description:
			Extended descriptor flag (true - use extended 128-element descriptors; false - use 64-element descriptors).	 
			
	 Parameter:	'surf_upright'
		Description:
			Up-right or rotated features flag (true - do not compute orientation of features; false - compute orientation).	 
			
	 Additional References:
		https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_surf_intro/py_surf_intro.html
		https://docs.opencv.org/2.4/modules/nonfree/doc/feature_detection.html#surf
		Bay, H. and Tuytelaars, T. and Van Gool, L. “SURF: Speeded Up Robust Features”, 9th European Conference on Computer Vision, 2006

BRISK feature detector and descriptor:
======================================
	 Parameter:	'brisk_thresh'
		Description:
			FAST/AGAST detection threshold score
			
	 Parameter:	'brisk_octaves'
		Description:
			detection octaves. Use 0 to do single scale.
			
	 Parameter:	'brisk_pattern_scale'
		Description:
			apply this scale to the pattern used for sampling the neighbourhood of a keypoint.
			
	 Additional References:
		https://docs.opencv.org/2.4/modules/features2d/doc/feature_detection_and_description.html#brisk
		Stefan Leutenegger, Margarita Chli and Roland Siegwart: BRISK: Binary Robust Invariant Scalable Keypoints. ICCV 2011: 2548-2555.

    nfeatures – 
    scaleFactor – 
    nlevels – 
    edgeThreshold –
    firstLevel – 
    WTA_K – 
    scoreType –
    patchSize – size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.

ORB feature detector and descriptor:
====================================
	 Parameter:	'orb_n_features'
		Description:
			The maximum number of features to retain.
			
	 Parameter:	'orb_scale_factor'
		Description:
			Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.
			
	 Parameter:	'orb_n_levels'
		Description:
			The number of pyramid levels;
			the smallest level will have linear size equal to input_image_linear_size/pow(orb_scale_factor, orb_n_levels).
			
	 Parameter:	'orb_edge_threshold'
		Description:
			 size of the border where the features are not detected;
			 it should roughly match the patchSize parameter.
			
	 Parameter:	'orb_first_level'
		Description:
			this should be 0 in the current implementation.
			
	 Parameter:	'orb_WTA_K'
		Description:
			number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).
			
	 Parameter:	'orb_score_type'
		Description:
			 The default HARRIS_SCORE means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.
		Possible Values:
			0:	Harris score
			1:	FAST score
			
	 Parameter:	'orb_patch_size'
		Description:
			size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.
			
	 Parameter:	'orb_fast_threshold'
		Description:
			threshold for the FAST descriptor if it is used;
			only applies to OpenCV 3;
			
	 Additional References:
		https://docs.opencv.org/2.4/modules/features2d/doc/feature_detection_and_description.html#orb
		Ethan Rublee, Vincent Rabaud, Kurt Konolige, Gary R. Bradski: ORB: An efficient alternative to SIFT or SURF. ICCV 2011: 2564-2571.
		

MSER feature detector:
======================
	 Parameter:	'mser_delta'
		Description:
			TBA
			
	 Parameter:	'mser_min_area'
		Description:
			TBA.
			
	 Parameter:	'mser_max_area'
		Description:
			TBA
			
	 Parameter:	'mser_max_variation'
		Description:
			 TBA
			
	 Parameter:	'mser_min_diversity'
		Description:
			this should be 0 in the current implementation.
			
	 Parameter:	'mser_max_evolution'
		Description:
			TBA
			
	 Parameter:	'mser_area_threshold'
		Description:
			 TBA
			
	 Parameter:	'mser_min_margin'
		Description:
			TBA
			
	 Parameter:	'mser_edge_blur_size'
		Description:
			TBA
			
	 Additional References:
		https://en.wikipedia.org/wiki/Maximally_stable_extremal_regions
		https://stackoverflow.com/questions/17647500/exact-meaning-of-the-parameters-given-to-initialize-mser-in-opencv-2-4-x
		J. Matas, O. Chum, M. Urban, and T. Pajdla. "Robust wide baseline stereo from maximally stable extremal regions." Proc. of British Machine Vision Conference, pages 384-396, 2002.
		
Similitude SSM:
===============
	 Parameter:	'sim_normalized_init'
		Description:
			use normalized initial bounding box with respect to which all subsequent transformations are computed;
			refer 'aff_normalized_init' for more details;
	 
	 Parameter:	'sim_geom_sampling'
		Description:
			use geometric sampling for stochastic SMs;
			
	 Parameter:	'sim_pt_based_sampling'
		Description:
			use point based sampling for stochastic SMs; refer 'aff_pt_based_sampling' for details;
			here the perturbations are applied to the two opposite bounding box corners;
			only matters if 'sim_geom_sampling' is disabled;
			if both sim_geom_sampling and sim_pt_based_sampling are disabled, perturbations are applied directly to the state parameters;
			
SL3 SSM:
========
	 Parameter:	'sl3_normalized_init'
		Description:
			use normalized initial bounding box with respect to which all subsequent transformations are computed;
			refer 'aff_normalized_init' for more details;	
			
	 Parameter:	'sl3_iterative_sample_mean'
		Description:
			use iterative method for computing the mean of SL3 samples;
			if disabled, mean is computed as the simple element wise mean of the state vectors;
			
	 Parameter:	'sl3_sample_mean_max_iters' / 'sl3_sample_mean_eps'
		Description:
			maximum iterations and epsilon used for controlling the iterative SL3 mean estimation process;		
			
	 Parameter:	'sl3_debug_mode'
		Description:
			enable printing and writing of debugging data
			
Sobel Filtering Preprocessor:
=============================
	 Parameter:	'sobel_kernel_size'
		Description:
			Sobel filter kernel size; this is a single number so that only square kernels are supported;
		
	 Parameter:	'sobel_normalize'
		Description:
			normalize the X and Y Sobel derivative images by their squared norm before computing their average;
		
	 Additional Reference:
		https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/sobel_derivatives/sobel_derivatives.html
		
Selective pixel integration (SPI):
==================================
	 Parameter:	'spi_type'
		Description:
			SPI model
		Possible Values:
			0:	None (disable SPI)
			1:	Pixel difference
			2:  Pixel gradient norm
			3: Good Features To Track (GFTT)
	 
	 Note:	SPI can only be used if it is enabled during compilation; also SSD and NCC are the only AMs for which SPI support has been implemented though all SSMs do support it;
			
Pixel difference SPI model:
===========================			
	 Parameter:	'spi_pix_diff_thresh'
		Description:
			pixel difference threshold between the template and the current patch so that pixels with difference above this value are turned off
			
Pixel gradient norm SPI model:
==============================
	 Parameter:	'spi_grad_thresh'
		Description:
			pixel gradient norm threshold so that pixels with gradient norm less than value this are turned off
			
	 Parameter:	'spi_grad_use_union'
		Description:
			use the union of the SPI masks computed using the initial and current gradients as the overall SPI mask so that only pixels that are turned on in both masks will be turned on in the final mask
			
GFTT SPI model:
===============
	 Parameter:	'spi_gftt_max_corners'
		Description:
			Maximum number of corners to return. If there are more corners than are found, the strongest of them is returned.
			
	 Parameter:	'spi_gftt_quality_level'
		Description:
			parameter characterizing the minimal accepted quality of image corners;
			the parameter value is multiplied by the best corner quality measure, which is the minimal eigenvalue or the Harris function response;
			the corners with the quality measure less than the product are rejected;
			for example, if the best corner has the quality measure = 1500, and the qualityLevel=0.01 , then all the corners with the quality measure less than 15 are rejected.
			
	 Parameter:	'spi_gftt_min_distance'
		Description:
			Minimum possible Euclidean distance between the returned corners.
			
	 Parameter:	'spi_gftt_block_size'
		Description:
			size of an average block for computing a derivative covariation matrix over each pixel neighbourhood.
			
	 Parameter:	'spi_gftt_use_harris_detector'
		Description:
			parameter indicating whether to use a Harris detector
			
	 Parameter:	'spi_gftt_k'
		Description:
			Free parameter of the Harris detector;
			only matters if 'spi_gftt_use_harris_detector' is enabled
			
	 Parameter:	'spi_gftt_use_union'
		Description:
			use the union of the SPI masks computed using the initial and current gradients as the overall SPI mask so that only pixels that are turned on in both masks will be turned on in the final mask
			
	 Parameter:	'spi_gftt_neigh_offset'
		Description:
			size of the neighbourhood around each detected corner or feature point point in which all pixels are turned on;
			this is a single number so that only square neighbourhoods are supported;  
			
	 Additional Reference:
		https://docs.opencv.org/2.4/modules/imgproc/doc/feature_detection.html#goodfeaturestotrack

Spline SSM:
===========
	 Parameter:	'spl_control_size'
		Description:
			resolution of spline control patch, i.e. each spline control point influences pixels in a neighbourhood of size spl_control_size x spl_control_size  around it;
	 
	 Parameter:	'spl_control_overlap'
		Description:
			overlap in pixels between the influence regions of neighbouring spline control points;
			
	 Parameter:	'spl_interp_type'
		Description:
			interpolation method to compute the displacements of individual pixels based on those of the spline control points;
			
	 Parameter:	'spl_static_wts'
		Description:
			keep the weights associated with control points constant during tracking
			
	 Parameter:	'spl_debug_mode'
		Description:
			enable printing and writing of debugging data	
			
	 Note:	this SSM is not completely implemented yet so might not work with some SMs

Sum of Pixel wise Structural Similarity (SPSS) AM:
==================================================
	 Parameter:	'spss_k'
		Description:
			constant added to the numerator and denominator to avoid numerical instability issues caused by too small denominators;
	 Additional Reference:
		Singh, A.; Siam, M. & Jagersand, M., 'Unifying Registration based Tracking: A Case Study with Structural Similarity', WACV, 2017 			

Structural Similarity (SSIM) AM:
================================
	 Parameter:	'ssim_k1' / 'ssim_k2'
		Description:
			constants added to the two terms in the numerator and denominator to avoid numerical instability issues caused by too small denominators;
			
	 Additional References:
		Wang, Z.; Bovik, A.; Sheikh, H. & Simoncelli, E., 'Image quality assessment: from error visibility to structural similarity Image Processing', IEEE Transactions on, 2004, 13, 600-6
		Singh, A.; Siam, M. & Jagersand, M., 'Unifying Registration based Tracking: A Case Study with Structural Similarity', WACV, 2017 	

Sum of Squared Differences (SSD) AM:
====================================
	 Parameter:	'ssd_show_template'
		Description:
			show the image patch corresponding to the template
			
Sum of AMs:
===========
	 Parameter:	'sum_am1' / 'sum_am2'
		Description:
			the two AMs whose sum will make up the composite AM	
		Possible Values:
			refer 'mtf_am' for a list of AMs
			
	 Note:	this composite AM is not completely implemented yet so might not work with some combinations of AMs and SMs
