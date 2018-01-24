mexMTF
------

mexMTF is the Matlab interface to MTF.\

mexMTF commands are invoked using the MATLAB function [success, out1,
out2, ...] = mexMTF('command',p1,p2,...) where p1, p2, ... are the
parameters required for the command and out1, out2, ... are the outputs
produced, if any. The first output success is a flag that indicates
whether or not the command invocation was successful. \
 mexMTF has both single and multi threaded versions where the latter,
called mexMTF2, is recommended as being faster and more stable. A
complete example of using it can be found in
\~vis/ugrad\_vision\_w13/src/MTF/Examples/matlab/runMTF2.m (or runMTF.m
for the single threaded version).

Preliminaries: Setting your environment variables and path
----------------------------------------------------------

-   Put this line in your .bashrc: \
     source \~vis/ugrad\_vision\_w13/scripts/vision\_exports\
     This will set up environment variables to access the MTF video and
    tracking software.\
     export MATLABPATH=\$MATLABPATH:\~vis/matlabdirs/ \
     This will set up mexMTF path for matlab.
-   It can be useful to test the camera with the linux software
    *coriander*. Start coriander in a shell: \
     \$coriander \
     Check that the camera name shows (e.g. "pyro" or "PtGrey
    Grasshopper") shows. Click the services tab. Click receive then
    display.\
     **When done with coriander, exit it.** This will let other programs
    use the camera (such as mexMTF).

Using mexMTF to capture images:
-------------------------------

-   In a terminal type:\
     \$ matlab\
     to start Matlab
-   In Matlab, create an MTF input pipeline as: \
     \>\>success = mexMTF2('init','pipeline v img\_source f vp\_fw\_res
    640x480'); \
     for "new" point-grey firewire cameras at 640 x 480 resolution \
     OR\
     \>\>success = mexMTF2('init','pipeline c img\_source u'); \
     for USB webcams at 640 X 480 resolution assuming that no firewire
    cameras are attached.\
    -   USB cameras have to be accessed using the OpenCV pipeline as the
        V4L2 module required by the ViSP pipeline is not installed
        correctly on lab machines. OpenCV pipeline does not allow
        detailed manipulation of camera settings so this is the only
        available mode.
    -   

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
    -   Note that the FlyCapture SDK based ViSP Point Grey firewire
        pipeline ( img\_source p ) does not work on lab machines so its
        corresponding parameters for setting camera shutter speed (
        vp\_pg\_fw\_shutter\_ms ), exposure ( vp\_pg\_fw\_exposure ),
        gain ( vp\_pg\_fw\_gain ) and brightness (
        vp\_pg\_fw\_brightness ) are not available either.
-   Update pipeline, capture an image and show it: \
     \>\>[success, im] = mexMTF2('get\_frame');\
     \>\>figure\
     \>\>imshow(im);\

Using mexMTF to track objects:
------------------------------

-   Create a tracker using:\
     \>\>tracker\_id = mexMTF2('create\_tracker','mtf\_sm esm mtf\_am
    ssd mtf\_ssm 2', init\_location); \
    -   This will create a tracker based on Efficient Second-order
        Minimization (ESM) search method, Sum of Squared Differences
        (SSD) appearance model and translation (2 DOF) state space
        model.
    -   For other types of trackers supported by MTF, refer to the
        documentation for mtf\_sm, mtf\_am and mtf\_ssm on [its
        configuration
        page](http://webdocs.cs.ualberta.ca/~vis/mtf/params.html)
    -   tracker\_id will be 0 if tracker creation was unsuccessful so it
        doubles as the success flag. An error message may also be
        produced.
    -   init\_location is either a 1x4 or 2x4 matrix. In the former
        case, it specifies a rectangle as [x, y, width, height] where
        (x, y) are the coordinates of the top left corner of the
        rectangle. In the latter case, it specifies an arbitrary
        quadrilateral whose x, y coordinates are arranged as: [top left,
        top right, bottom left and bottom right] such that the x and y
        coordinates are in the first and second row respectively.
    -   If init\_location is omitted, the user will be asked to select
        the initial object location interactively.
-   Update tracker and obtain the current location of the tracked
    object:\
     \>\>[success, corners] = mexMTF2('get\_region', tracker\_id); \
     Depending on the tracker settings, the raw RGB image obtained from
    the input pipeline might need to be converted to grayscale before
    passing it to the tracker.
-   Remove a tracker:\
     \>\>success = mexMTF2('remove\_tracker', tracker\_id);
-   Delete all pipelines and trackers:\
     \>\>success = mexMTF2('quit'); \