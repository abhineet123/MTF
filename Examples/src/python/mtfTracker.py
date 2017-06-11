__author__ = 'abhineet'

import pyMTF

from TrackerBase import *


class mtfTracker(TrackerBase):
    def __init__(self, config_root_dir=None):
        print "Initializing pyMTF Tracker"
        self.text = 'pyMTF'
        self.config_root_dir=config_root_dir

    def initialize(self, img, region):
        if len(img.shape) != 3:
            raise SystemExit('Error in mtfTracker: '
                             'Expected multi channel image but found a single channel one')
        self.proposal = np.copy(region)
        self.size_x = abs(region[0, 0] - region[0, 2])
        self.size_y = abs(region[1, 0] - region[1, 2])

        print 'region: ', region

        pyMTF.initialize(img.astype(np.uint8), region.astype(np.float64), self.config_root_dir)

        self.initialized = True

    def update(self, img):
        self.proposal = pyMTF.update(img.astype(np.uint8))

    def get_region(self):
        return self.proposal


    def set_region(self, corners):
        pyMTF.setRegion(corners.astype(np.float64))


    def cleanup(self):
        pass


    def is_initialized(self):
        return self.initialized








