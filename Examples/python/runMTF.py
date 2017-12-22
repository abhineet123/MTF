import os
import sys
import cv2
import numpy as np
import math
import time
import pyMTF
from utilities import readGroundTruth, writeCorners, drawRegion
from datasets import sequences, actors

if __name__ == '__main__':

    config_root_dir = '../../Config'
    db_root_dir = '../../../../../Datasets';
    actor_id = 0
    seq_id = 0
    seq_fmt = 'jpg';
    use_rgb_input = 0;

    write_stats_to_file = 0
    show_tracking_output = 1

    arg_id = 1
    if len(sys.argv) > arg_id:
        seq_id = int(sys.argv[arg_id])
        arg_id += 1
    if len(sys.argv) > arg_id:
        write_stats_to_file = int(sys.argv[arg_id])
        arg_id += 1
    if len(sys.argv) > arg_id:
        show_tracking_output = int(sys.argv[arg_id])
        arg_id += 1

    if seq_id >= len(sequences):
        print 'Invalid dataset_id: ', seq_id
        sys.exit()

    actor = actors[actor_id]
    seq_name = sequences[actor][seq_id]
    print 'seq_id: ', seq_id
    print 'seq_name: ', seq_name

    src_fname =  '{:s}/{:s}/{:s}/frame%05d.{:s}'.format(db_root_dir, actor, seq_name, seq_fmt)
    ground_truth_fname =  '{:s}/{:s}/{:s}.txt'.format(db_root_dir, actor, seq_name)
    result_fname = seq_name + '_res.txt'

    if not pyMTF.create(config_root_dir):
        print 'Tracker creation was unsuccessful'
        sys.exit()

    cap = cv2.VideoCapture()
    if not cap.open(src_fname):
        print 'The video file ', src_fname, ' could not be opened'
        sys.exit()

    # thickness of the bounding box lines drawn on the image
    thickness = 2
    # ground truth location drawn in green
    ground_truth_color = (0, 255, 0)
    # tracker location drawn in red
    result_color = (0, 0, 255)

    # read the ground truth
    ground_truth = readGroundTruth(ground_truth_fname)
    no_of_frames = ground_truth.shape[0]

    print 'no_of_frames: ', no_of_frames

    ret, init_img = cap.read()
    if not ret:
        print "Initial frame could not be read"
        sys.exit(0)

    # extract the true corners in the first frame and place them into a 2x4 array
    init_corners = [ground_truth[0, 0:2].tolist(),
                    ground_truth[0, 2:4].tolist(),
                    ground_truth[0, 4:6].tolist(),
                    ground_truth[0, 6:8].tolist()]
    init_corners = np.array(init_corners).T
    # write the initial corners to the result file

    # initialize tracker with the first frame and the initial corners
    if not use_rgb_input:
        init_img = cv2.cvtColor(init_img, cv2.COLOR_RGB2GRAY)
    if not pyMTF.initialize(init_img.astype(np.uint8), init_corners.astype(np.float64)):
        print 'Tracker initialization was unsuccessful'
        sys.exit()

    if show_tracking_output:
        # window for displaying the tracking result
        window_name = 'Tracking Result'
        cv2.namedWindow(window_name)

    # lists for accumulating the tracking error and fps for all the frames
    tracking_errors = []
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
            src_img = cv2.cvtColor(src_img, cv2.COLOR_RGB2GRAY)
        start_time = time.clock()

        # update the tracker with the current frame
        pyMTF.update(src_img.astype(np.uint8), tracker_corners)

        if not isinstance(tracker_corners, np.ndarray):
            print 'Tracker update was unsuccessful'
            sys.exit()

        end_time = time.clock()

        # compute the tracking fps
        current_fps = 1.0 / (end_time - start_time)
        tracking_fps.append(current_fps)

        # compute the tracking error
        current_error = math.sqrt(np.sum(np.square(actual_corners - tracker_corners)) / 4)
        tracking_errors.append(current_error)

        if show_tracking_output:
            # draw the ground truth location
            drawRegion(src_img, actual_corners, ground_truth_color, thickness)
            # draw the tracker location
            drawRegion(src_img, tracker_corners, result_color, thickness)
            # write statistics (error and fps) to the image
            cv2.putText(src_img, "{:5.2f} {:5.2f}".format(current_fps, current_error), (5, 15), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255))
            # display the image
            cv2.imshow(window_name, src_img)

            if cv2.waitKey(1) == 27:
                break

    mean_error = np.mean(tracking_errors)
    mean_fps = np.mean(tracking_fps)

    print 'mean_error: ', mean_error
    print 'mean_fps: ', mean_fps