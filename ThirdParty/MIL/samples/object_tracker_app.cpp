#include <iostream>
#include <stdio.h>

// OpenCV Includes
#include <mtf/ThirdParty/MIL/object_tracker.h>

cv::Rect bestFitRectangle(const cv::Mat &corners){
	double center_x = (corners.at<double>(0, 0) + corners.at<double>(0, 1) +
		corners.at<double>(0, 2) + corners.at<double>(0, 3)) / 4;
	double center_y = (corners.at<double>(1, 0) + corners.at<double>(1, 1) +
		corners.at<double>(1, 2) + corners.at<double>(1, 3)) / 4;

	double mean_half_width = (abs(corners.at<double>(0, 0) - center_x) + abs(corners.at<double>(0, 1) - center_x)
		+ abs(corners.at<double>(0, 2) - center_x) + abs(corners.at<double>(0, 3) - center_x)) / 4.0;
	double mean_half_height = (abs(corners.at<double>(1, 0) - center_y) + abs(corners.at<double>(1, 1) - center_y)
		+ abs(corners.at<double>(1, 2) - center_y) + abs(corners.at<double>(1, 3) - center_y)) / 4.0;

	cv::Rect best_fit_rect;
	best_fit_rect.x = center_x - mean_half_width;
	best_fit_rect.y = center_y - mean_half_height;
	best_fit_rect.width = 2 * mean_half_width;
	best_fit_rect.height = 2 * mean_half_height;

	return best_fit_rect;
}

std::vector<cv::Mat> readGT(std::string source_name, std::string source_path,
	int n_frames = 1, int init_frame_id = 0, int debug_mode = 0,
	bool use_opt_gt = false, std::string opt_gt_ssm = "2"){
	std::vector<cv::Mat> ground_truth;
	std::string gt_filename;
	if(use_opt_gt){
		gt_filename = source_path + "/OptGT/" + source_name + "_" + opt_gt_ssm + ".txt";
	} else{
		gt_filename = source_path + "/" + source_name + ".txt";
	}

	std::cout << "Reading object location from ground truth file: " << gt_filename << "\n";

	std::ifstream fin(gt_filename.c_str(), std::ios::in);
	if(!fin) {
		printf("Could not open ground truth file for reading object location.\n");
		return ground_truth;
	}
	char header[500];
	fin.getline(header, 500);
	//if(n_frames <= 0){
	//	int curr_pos = fin.tellg();
	//	std::string line;
	//	n_frames = 0;
	//	while(std::getline(fin, line)){ ++n_frames; }
	//	fin.seekg(curr_pos, std::ios::beg);
	//}

	std::cout << "n_frames: " << n_frames << "\n";
	std::cout << "init_frame_id: " << init_frame_id << "\n";

	int frame_id = 0;
	while(fin.good()) {
		fin >> header;
		if(!fin.good()){
			printf("Ground truth file has ended unexpectedly - only %d out of %d entries were read\n",
				frame_id, n_frames);
			break;
		}
		float ulx, uly, urx, ury, lrx, lry, llx, lly;
		fin >> ulx >> uly >> urx >> ury >> lrx >> lry >> llx >> lly;
		if(!fin){
			printf("Invalid formatting in line %d of the ground truth file. Aborting...\n", frame_id + 1);
			std::cout << ulx << "\t" << uly << "\t" << urx << "\t" << ury << "\t" << lrx << "\t" << lry << "\t" << llx << "\t" << lly << "\n";
			return ground_truth;
		}
		cv::Mat curr_gt(2, 4, CV_64FC1);
		curr_gt.at<double>(0, 0) = ulx;
		curr_gt.at<double>(0, 1) = urx;
		curr_gt.at<double>(0, 2) = lrx;
		curr_gt.at<double>(0, 3) = llx;
		curr_gt.at<double>(1, 0) = uly;
		curr_gt.at<double>(1, 1) = ury;
		curr_gt.at<double>(1, 2) = lry;
		curr_gt.at<double>(1, 3) = lly;
		ground_truth.push_back(curr_gt);
		++frame_id;
	}
	fin.close();
	return ground_truth;
}

int
main(int argc, char** argv)
{
  // Setup the parameters to use OnlineBoosting or MILTrack as the underlying tracking algorithm
  cv::ObjectTrackerParams params;
#if 0
  params.algorithm_ = cv::ObjectTrackerParams::CV_ONLINEBOOSTING;
  //params.algorithm_ = cv::ObjectTrackerParams::CV_SEMIONLINEBOOSTING;
  params.num_classifiers_ = 100;
  params.overlap_ = 0.99f;
  params.search_factor_ = 2;
#else
  params.algorithm_ = cv::ObjectTrackerParams::CV_ONLINEMIL;
  params.num_classifiers_ = 50;
  params.num_features_ = 250;
#endif

  // Instantiate an object tracker
  cv::ObjectTracker tracker(params);

  // Read in a sequence of images from disk as the video source
  const char* directory = "/home/abhineet/E/UofA/Thesis/Code/Datasets/TMT";
  std::string source_name = "nl_cereal_s3";
  if(argc>1){
	  source_name = std::string(argv[1]);
  }
  const int start = 1;
  const int stop = 462;
  const int delta = 1;
  const char* prefix = "frame";
  const char* suffix = "jpg";
  char filename[1024];

  std::vector<cv::Mat>  ground_truth = readGT(source_name, std::string(directory), -1);

  // Some book-keeping
  bool is_tracker_initialized = false;
  //CvRect init_bb = cvRect(122, 58, 75, 97); // the initial tracking bounding box
  cv::Rect init_bb = bestFitRectangle(ground_truth[0]);

  /* const char* cascade_name = "haarcascade_frontalface_alt_tree.xml";
   const int minsz = 20;
   if( Tracker::facecascade.empty() )
   Tracker::facecascade.load(cascade_name);

   cv::Mat gray;
   cv::cvtColor(frame, gray, CV_BGR2GRAY);
   cv::equalizeHist(gray, gray);

   std::vector<cv::Rect> faces;
   facecascade.detectMultiScale(gray, faces, 1.05, 3, CV_HAAR_DO_CANNY_PRUNING ,cvSize(minsz, minsz));

   bool is_good = false;
   cv::Rect r;
   for (int index = faces.size() - 1; index >= 0; --index)
   {
   r = faces[index];
   if (r.width < minsz || r.height < minsz || (r.y + r.height + 10) > frame.rows || (r.x + r.width) > frame.cols
   || r.y < 0 || r.x < 0)
   continue;
   is_good = true;
   break;
   }
   */

  cv::Rect theTrack;
  bool tracker_failed = false;

  // Read in images one-by-one and track them
  cv::namedWindow("Tracker Display", cv::WINDOW_NORMAL);
  for (int frame = start; frame <= stop; frame += delta)
  {
    sprintf(filename, "%s/%s/%s%05d.%s", directory, source_name.c_str(),
		prefix, frame, suffix);
    cv::Mat image = cv::imread(filename);
    if (image.empty())
    {
      std::cerr << "Error loading image file: " << filename << "!\n" << std::endl;
      break;
    }

    // Initialize/Update the tracker
    if (!is_tracker_initialized)
    {
      // Initialize the tracker
      if (!tracker.initialize(image, init_bb))
      {
        // If it didn't work for some reason, exit now
        std::cerr << "\n\nCould not initialize the tracker!  Exiting early...\n" << std::endl;
        break;
      }

      // Store the track for display
      theTrack = init_bb;
      tracker_failed = false;

      // Now it's initialized
      is_tracker_initialized = true;
      std::cout << std::endl;
      continue;
    }
    else
    {
      // Update the tracker
      if (!tracker.update(image, theTrack))
      {
        std::cerr << "\rCould not update tracker (" << frame << ")";
        tracker_failed = true;
      }
      else
      {
        tracker_failed = false;
      }
    }

    // Display the tracking box
    CvScalar box_color;
    if (tracker_failed)
    {
      box_color = cv::Scalar(255, 0, 0);
    }
    else
    {
      box_color = cv::Scalar(255, 255, 0);
    }
    cv::rectangle(image, cvPoint(theTrack.x, theTrack.y),
                  cvPoint(theTrack.x + theTrack.width - 1, theTrack.y + theTrack.height - 1), box_color, 2);

    // Display the new image
    cv::imshow("Tracker Display", image);

    // Check if the user wants to exit early
    int key = cv::waitKey(1);
    if (key == 'q' || key == 'Q')
    {
      break;
    }
  }

  // Exit application
  std::cout << std::endl;
  return 0;
}

