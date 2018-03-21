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

pyMTF is the Python interface to MTF.

It is very simple to use - there are just 3 functions:

-   `success = pyMTF.create(config_root_dir)` to create a new tracker
-   `success = pyMTF.initialize(image, corners, id)` to initialize the tracker
-   `corners = pyMTF.update(image, id)` to update the tracker and get new location

`id` is an integer that starts at 0 and increments by 1 every time `pyMTF.create` is called.
`image` can be RGB or grayscale but must be of type `numpy.uint8` while `corners` must be a 2x4 matrix of type `numpy.float64`.
In addition, the following function allows the internal state of a tracker to be modified so the object is located at the provided location instead of where it was after the last update:

-   `success = pyMTF.setRegion(corners, id)`

There is currently no multi threaded version of pyMTF nor does it support acquiring images. These functionalities will be added soon.