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
    # script parameters
    show_tracking_output = 1

    # MTF parameters
    config_dir = '../../Config'
    db_root_path = '../../../Datasets'
    pipeline = 'c'
    img_source = 'u'
    actor_id = 1
    seq_id = 16
    seq_fmt = 'jpg'
    init_frame_id = 1

    param_str = 'db_root_path {:s}'.format(db_root_path)
    param_str = '{:s} pipeline {:s}'.format(param_str, pipeline)
    param_str = '{:s} img_source {:s}'.format(param_str, img_source)
    param_str = '{:s} actor_id {:d}'.format(param_str, actor_id)
    param_str = '{:s} seq_id {:d}'.format(param_str, seq_id)
    param_str = '{:s} seq_fmt {:s}'.format(param_str, seq_fmt)
    param_str = '{:s} init_frame_id {:d}'.format(param_str, init_frame_id)
    nargin = len(sys.argv) - 1
    if nargin % 2 != 0:
        raise IOError('Optional arguments must be specified in pairs')
    # parse optional arguments
    arg_id = 1
    while arg_id <= nargin:
        arg_name = sys.argv[arg_id]
        arg_val = sys.argv[arg_id + 1]
        if arg_name == 'config_dir':
            config_dir = arg_val
        elif arg_name == 'show_tracking_output':
            show_tracking_output = arg_val
        else:
            param_str = '%s %s %s'.format(param_str, arg_name, arg_val)
        arg_id += 2
    param_str = 'config_dir %s %s'.format(config_dir, param_str)

    # thickness of the bounding box lines drawn on the image
    thickness = 2
    # tracker location drawn in red
    result_color = (0, 0, 255)

    # initialize input pipeline
    if not pyMTF2.init(param_str):
        raise SystemError('MTF input pipeline creation was unsuccessful')
    else:
        print('MTF input pipeline created successfully')

    tracker_id = pyMTF2.createTracker(param_str)

    if not tracker_id:
        raise SystemError('Tracker creation was unsuccessful')
    else:
        print('Tracker created successfully')

    if show_tracking_output:
        # window for displaying the tracking result
        window_name = 'Tracking Result'
        cv2.namedWindow(window_name)

    while True:
        src_img = pyMTF2.getFrame()
        if curr_img is None:
            print('Frame extraction was unsuccessful')
            break

        curr_corners = pyMTF2.getRegion(tracker_id)
        if curr_corners is None:
            print('Tracker update was unsuccessful')
            sys.exit()

        if show_tracking_output:
            # draw the tracker location
            drawRegion(src_img_disp, curr_corners, result_color, thickness)
            # display the image
            cv2.imshow(window_name, src_img_disp)

            if cv2.waitKey(1) == 27:
                break

    pyMTF2.removeTracker(tracker_id)
    pyMTF2.quit()
