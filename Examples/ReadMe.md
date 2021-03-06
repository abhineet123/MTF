mexMTF
------

mexMTF is the Matlab interface to MTF.

mexMTF commands are invoked using the MATLAB function `[success, out1,
out2, ...] = mexMTF('command',p1,p2,...)` where `p1, p2, ...` are the
parameters required for the command and `out1, out2, ...` are the outputs
produced, if any. The first output `success` is a flag that indicates
whether or not the command invocation was successful. \
 mexMTF has both single and multi threaded versions where the latter,
called `mexMTF2`, is recommended as being faster and more stable. A
complete example of using it can be found in
`matlab/runMTF2.m` (or `runMTF.m` for the single threaded version).

Using mexMTF to capture images:
-------------------------------

-   In a terminal type:\
     `matlab`\
     to start Matlab
-   In Matlab, create an MTF input pipeline as: \
     `>>success = mexMTF2('init','pipeline v img_source f vp_fw_res
    640x480');` \
     for firewire camera at 640 x 480 resolution \
     OR\
     `>>success = mexMTF2('init','pipeline c img_source u');` \
     for USB webcams at 640 X 480 resolution assuming that no firewire
    cameras are attached.<br>
	This command only needs to be run once at the beginning of each Matlab session.

-   For initialization of firewire cameras at other resolutions and
    adjusting other settings like ISO, FPS and colour format, please
    refer to the description of MTF ViSP input pipeline on [MTF
    configuration
    page](http://webdocs.cs.ualberta.ca/~vis/mtf/params.html).
    -   As mentioned there, MTF parameters can be provided either in
        ASCII text files or as pairwise command line arguments. Both of
        these are supported by mexMTF too except that the latter are
        represented by a single optional string argument into which all
        pairs have been combined (as shown above).
-   Update pipeline, capture an image and show it: \
     `>>[success, im] = mexMTF2('get_frame');`\
     `>>figure`\
     `>>imshow(im);`

Using mexMTF to track objects:
------------------------------

-   Create a tracker using:\
     `>>tracker_id = mexMTF2('create_tracker','mtf_sm esm mtf_am
    ssd mtf_ssm 2', init_location);`
    -   This will create a tracker based on Efficient Second-order
        Minimization (ESM) search method, Sum of Squared Differences
        (SSD) appearance model and translation (2 DOF) state space
        model.
    -   For other types of trackers supported by MTF, refer to the
        documentation for `mtf_sm`, `mtf_am` and `mtf_ssm` on [its
        configuration
        page](http://webdocs.cs.ualberta.ca/~vis/mtf/params.html)
    -   `tracker_id` will be 0 if tracker creation was unsuccessful so it
        doubles as the success flag. An error message may also be
        produced.
    -   `init_location` is either a 1x4 or 2x4 matrix. In the former
        case, it specifies a rectangle as [x, y, width, height] where
        (x, y) are the coordinates of the top left corner of the
        rectangle. In the latter case, it specifies an arbitrary
        quadrilateral whose x, y coordinates are arranged as: [top left,
        top right, bottom left, bottom right] such that the x and y
        coordinates are in the first and second row respectively.
    -   If `init_location` is omitted, the user will be asked to select
        the initial object location interactively.
-   Update tracker and obtain the current location of the tracked
    object:\
     `>>[success, corners] = mexMTF2('get_region', tracker_id);` \
     Depending on the tracker settings, the raw RGB image obtained from
    the input pipeline might need to be converted to grayscale before
    passing it to the tracker.
-   Remove a tracker:\
     `>>success = mexMTF2('remove_tracker', tracker_id);`
-   Delete all pipelines and trackers:\
     `>>success = mexMTF2('quit');`
	 
	 
pyMTF
-----

pyMTF is the Python interface to MTF that supports both Python 2.7 and 3.x.
Its usage is demonstrated in _python/runMTF.py_.

It is very simple to use - there are just 2 main functions needed to track an object:

-   `tracker_id = pyMTF.create(image, corners, config_root_dir)` to create and initialize a new tracker
-   `success = pyMTF.getRegion(image, corners, id)` to update the tracker and get its new location

`tracker_id` is an unsigned integer that starts at 1 and increments by 1 every time `pyMTF.create` is called; `tracker_id=0` indicates that the tracker creation/initialization failed.  
`image` can be RGB or grayscale but must be of type `numpy.uint8` while `corners` must be a 2x4 matrix of type `numpy.float64`.  

In addition, the following function allows the internal state of a tracker to be modified so the object is located at the provided location instead of where it was after the last update:

-   `success = pyMTF.setRegion(corners, tracker_id)`  

Finally, this can be used to remove an existing tracker:

-   `success = pyMTF.remove(tracker_id)`

pyMTF2
------

Similar to `mexMTF2`, this is the multi threaded version that also supports acquiring images.
This is more suited to live tracking as the input pipeline continuously updates itself and so would quickly run out of images in a pre-recorded sequence.

Usage of `pyMTF2` is demonstrated in _python/runMTF2.py_.
It provides the following functions that closely mirror those of `mexMTF2` (`[]`indicates optional arguments):


-   `success = pyMTF2.init([params])`
    - initializes the input pipeline
    - `params` is a string containing pairwise arguments similar to those accepted by the main application `runMTF`
    - refer to the ReadMe in the _Config_ folder for a complete list of arguments
    - there are also a few arguments specific to this interface that start with `py_`	
    - only one pipeline at a time is currently supported so all calls to this function after the first one will be ignored till `quit` is called to clear the existing pipeline and all trackers attached to it

-   `tracker_id = pyMTF2.createTracker([corners, params])`
    - creates and initializes a new tracker
    - both  `corners`  and `params` are optional and keyword arguments
    - `corners` must be a 2x4 matrix of type `numpy.float64`
    - if `corners` is not provided, user will be asked to select the object to track interactively
    - `tracker_id` is an unsigned integer that starts at 1 and increments by 1 every time `pyMTF.create` is called
    - `tracker_id=0` indicates that the tracker creation/initialization failed.
	
-   `tracker_ids = pyMTF2.createTrackers([params])`
    - creates and initializes multiple new trackers
	- number of trackers is specified through the `n_trackers` MTF parameter within `params`
    - user will be asked to select the objects to track interactively
    - `tracker_ids` is a 1D array of unsigned integers and will be `None` if any tracker creation/initialization failed.	
	
-   `corners = pyMTF2.getRegion(tracker_id)`
    - returns the latest location of the given tracker
    - `corners` is a 2x4 matrix of type `numpy.float64` and will be `None` if tracker has stopped working	
	
-   `image = pyMTF2.getFrame()`
    - returns the latest frame captured by the input pipeline
    - `image` is an RGB image stored in a numpy array of type `numpy.uint8` and will be `None` if pipeline has stopped working
	
-   `success = pyMTF2.setRegion(corners, tracker_id)`
    - modifies internal state of the tracker so the object is located at the provided location instead of where it was at the time of this call
    - currently not supported by third party trackers

-   `init_status = pyMTF2.isInitialized()`
    - returns 1 if the input pipeline has been initialized, 0 otherwise	

-   `success = pyMTF2.removeTracker(tracker_id)`
    - delete the given tracker
	
-   `success = pyMTF2.removeTrackers()`
    - delete all trackers	
	
-   `success = pyMTF2.quit()`
    - delete the input pipeline and all existing trackers
	
mtf.VideoCapture
----------------

This is a convenience wrapper for the input pipeline in `pyMTF2` that provides an an interface similar to _cv2.VideoCapture_ but can be used, through ViSP pipeline, to access USB and firewire cameras with custom resolution and other settings that are unavailable in _cv2_.
It can be installed by running `python setup.py install` or `python3 setup.py install` from _Examples/python_ folder.

Usage:

```
from mtf import mtf
cap = mtf.VideoCapture(cap_id, cap_args)
img_width = cap.get(3) # or cap.get('width')
img_height = cap.get(4) # or cap.get('height')
while True:
	ret, img = cap.read()
	if not ret:
		print('Image could not be read')
		break
cap.release()		
```
-   `cap_id` can be an integer for camera, a directory containing an image sequence (only `image%06d` format is supported), a video file or empty string if source is specified directly through `cap_args`.

-   `cap_args` is a string containing pairwise arguments for passing directly to `pyMTF2`

- both `cap_id` and `mtf_args` are optional and keyword based calling is also supported





	
	

