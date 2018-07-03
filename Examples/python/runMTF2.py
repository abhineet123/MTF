import os
import sys
import cv2
import numpy as np
import math
import time
import pyMTF2
from utilities import drawRegion
from matplotlib import pyplot as plt

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
    py_visualize = 0
    num_trackers = 1

    param_str = 'db_root_path {:s}'.format(db_root_path)
    param_str = '{:s} pipeline {:s}'.format(param_str, pipeline)
    param_str = '{:s} img_source {:s}'.format(param_str, img_source)
    param_str = '{:s} actor_id {:d}'.format(param_str, actor_id)
    param_str = '{:s} seq_id {:d}'.format(param_str, seq_id)
    param_str = '{:s} seq_fmt {:s}'.format(param_str, seq_fmt)
    param_str = '{:s} init_frame_id {:d}'.format(param_str, init_frame_id)
    param_str = '{:s} init_frame_id {:d}'.format(param_str, py_visualize)

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
            show_tracking_output = int(arg_val)
        elif arg_name == 'num_trackers':
            num_trackers = int(arg_val)
        else:
            param_str = '{:s} {:s} {:s}'.format(param_str, arg_name, arg_val)
        arg_id += 2
    param_str = 'config_dir {:s} {:s}'.format(config_dir, param_str)

    print('param_str: ', param_str)

    # thickness of the bounding box lines drawn on the image
    thickness = 2
    # tracker location drawn in red
    result_color = (0, 0, 255)

    # initialize input pipeline
    if not pyMTF2.init(param_str):
        raise SystemError('MTF input pipeline creation was unsuccessful')
    else:
        print('MTF input pipeline created successfully')

    tracker_ids = []
    for i in range(num_trackers):
        tracker_id = pyMTF2.createTracker(param_str)
        if not tracker_id:
            raise SystemError('Tracker {} creation was unsuccessful'.format(i + 1))
        else:
            print('Tracker {} created successfully'.format(i + 1))
        tracker_ids.append(tracker_id)

    if show_tracking_output:
        # plt.ion()
        # plt.show()

        # window for displaying the tracking result
        window_name = 'Tracking Result'
        cv2.namedWindow(window_name)

    while True:
        # print('getting frame')
        src_img = pyMTF2.getFrame()
        if src_img is None:
            print('Frame extraction was unsuccessful')
            break

        # src_img = np.asarray(src_img)
        # print('got frame {}/{}'.format(src_img.shape, src_img.dtype))

        for i in range(num_trackers):

            curr_corners = pyMTF2.getRegion(tracker_ids[i])
            if curr_corners is None:
                print('Tracker update was unsuccessful')
                break

            # print('curr_corners: ', curr_corners)
            if show_tracking_output:
                drawRegion(src_img, curr_corners, result_color, thickness)

        if show_tracking_output:
            # plt.imshow(src_img)
            # plt.draw()
            # plt.pause(0.00001)

            cv2.imshow(window_name, src_img)
            if cv2.waitKey(1) == 27:
                break

    pyMTF2.removeTrackers()
    pyMTF2.quit()
