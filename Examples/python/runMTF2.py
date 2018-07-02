import os
import sys
import cv2
import numpy as np
import math
import time
import pyMTF2
from utilities import readGroundTruth, writeCorners, drawRegion
from datasets import sequences, actors

if __name__ == '__main__':

    config_root_dir = '../../Config'
    use_rgb_input = 0

    write_stats_to_file = 0
    show_tracking_output = 1

    arg_id = 1
    if len(sys.argv) > arg_id:
        write_stats_to_file = int(sys.argv[arg_id])
        arg_id += 1
    if len(sys.argv) > arg_id:
        show_tracking_output = int(sys.argv[arg_id])
        arg_id += 1

    # thickness of the bounding box lines drawn on the image
    thickness = 2
    # ground truth location drawn in green
    ground_truth_color = (0, 255, 0)
    # tracker location drawn in red
    result_color = (0, 0, 255)
	
	# initialize input pipeline to get images from USB camera
	
	pyMTF2.init()

    # initialize tracker with the first frame and the initial corners
	tracker_id = 	pyMTF2.create(init_img.astype(np.uint8), init_corners.astype(np.float64), config_root_dir)
    if not tracker_id:
        print 'Tracker creation/initialization was unsuccessful'
        sys.exit()

    if show_tracking_output:
        # window for displaying the tracking result
        window_name = 'Tracking Result'
        cv2.namedWindow(window_name)

    # lists for accumulating the tracking fps for all the frames
    tracking_fps = []

    tracker_corners = np.zeros((2, 4), dtype=np.float64)

    for frame_id in xrange(1, no_of_frames):
        ret, src_img = cap.read()
        if not ret:
            print "Frame ", frame_id, " could not be read"
            break
        actual_corners = [ground_truth[frame_id, 0:2].tolist(),
                          ground_truth[frame_id, 2:4].tolist(),
                          ground_truth[frame_id, 4:6].tolist(),
                          ground_truth[frame_id, 6:8].tolist()]
        actual_corners = np.array(actual_corners).T

        if not use_rgb_input:
            src_img_disp = src_img.copy()
            src_img = cv2.cvtColor(src_img, cv2.COLOR_RGB2GRAY)
        else:
            src_img_disp = src_img

        start_time = time.clock()

        # update the tracker with the current frame
        success = pyMTF.getRegion(src_img.astype(np.uint8), tracker_corners, tracker_id)

        if not success:
            print 'Tracker update was unsuccessful'
            sys.exit()

        # print('tracker_corners before:\n {}'.format(tracker_corners))
        # success = pyMTF.setRegion(tracker_corners, tracker_id)
        # print('tracker_corners after:\n {}\n\n'.format(tracker_corners))

        end_time = time.clock()

        # compute the tracking fps
        current_fps = 1.0 / (end_time - start_time)
        tracking_fps.append(current_fps)

        # compute the tracking error
        current_error = math.sqrt(np.sum(np.square(actual_corners - tracker_corners)) / 4)
        tracking_errors.append(current_error)

        if show_tracking_output:
            # draw the ground truth location
            drawRegion(src_img_disp, actual_corners, ground_truth_color, thickness)
            # draw the tracker location
            drawRegion(src_img_disp, tracker_corners, result_color, thickness)
            # write statistics (error and fps) to the image
            cv2.putText(src_img_disp, "{:5.2f} {:5.2f}".format(current_fps, current_error), (5, 15),
                        cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))
            # display the image
            cv2.imshow(window_name, src_img_disp)

            if cv2.waitKey(1) == 27:
                break
	
	pyMTF.remove(tracker_id)
	
    mean_error = np.mean(tracking_errors)
    mean_fps = np.mean(tracking_fps)

    print 'mean_error: ', mean_error
    print 'mean_fps: ', mean_fps
