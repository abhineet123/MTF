Procedure for Installing Modular Tracking Framework (and OpenCV, etc.) on Ubuntu 18.04
-------------------------------------------------------------------
Bernard Llanos

January 17, 2019

------------
Remarks
------------
- The official installation instructions are at: https://github.com/abhineet123/MTF
- I omit 'sudo' in the commands listed below; Add it when it makes sense.
- My system has quite a few things installed for other programs, so it is possible that I have forgotten to list some of the dependencies for MTF.
- Tokens to search and replace in this file:
  - FILEPATH: An appropriate location in your filesystem.

-------------
Prerequisites
-------------

- Git: `apt install git`
- CMake: `apt install cmake`
- (For the Python 3 interface) Numpy: `apt install python3-numpy`
- Eigen3: `apt install libeigen3-dev`
- (For the MATLAB interface) MATLAB
  - I have R2018b.
- Boost: `apt install libboost-all-dev`
- HDF5: `apt install libhdf5-openmpi-100 libhdf5-dev`
  - I am not sure this is really a dependency, but the MTF build process did look for it. I might not have needed `libhdf5-dev`.
- `apt install liblz4-dev`

## OpenCV
- For MTF to work with MATLAB, you should build MTF with the same version of OpenCV that is packaged with MATLAB. (Feel free to build a different copy of MTF to use outside of MATLAB with a different version of OpenCV.)
  - You can find out which version of OpenCV MATLAB has by running `ls -l FILEPATH/R2018b/bin/glnxa64/*opencv*`.
- I haven't yet installed OpenCV's "contrib" modules; We'll see if they are necessary depending on which trackers you want to use from Modular Tracking Framework.

### Dependencies
- `apt install libtbb-dev python3-dev`
  - `python3-dev` may not be necessary; I probably already had the equivalent for Python 2.

### Download
- I retrieved version 3.4.0, as a source code archive, from https://opencv.org/releases.html

### Building
- Extract the source code archive.
- `cd FILEPATH/opencv-3.4.0`
- `mkdir build`
- `cd build`
- `cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=FILEPATH ..`
  - I had a compilation error regarding a missing header file that I solved by adding `-DWITH_NVCUVID=OFF` to the above command.
  - Here, you specify where the library should be installed.
  - (Optional) To use this version of OpenCV with Python, add what you set for `CMAKE_INSTALL_PREFIX` to your `PYTHONPATH`.
    - e.g. `export PYTHONPATH=$PYTHONPATH:$CMAKE_INSTALL_PREFIX/python`
- `make -j4`
  - This builds both Python 2 and Python 3 libraries.
- `make install`

## FLANN
- Download version 1.8.4 from http://www.cs.ubc.ca/research/flann/
- Extract the archive, and open the resulting directory in a shell session.
- (Probably optional) Let CMake know where to find MATLAB: `export PATH=FILEPATH/R2018b/bin${PATH:+:${PATH}}`
- `mkdir build`, `cd build`, `cmake ..`
- `make`
- `make install`

## VISP
- This is an optional dependency.
- The Ubuntu repositories now contain a version that is sufficiently up to date.
- `apt install libvisp-dev libvisp-doc visp-images-data`
- For some reason, I think I ended up not building MTF with VISP.

## Caffe
- Optional; I wouldn't install it unless you need the features that it provides. When I was building MTF, I think Caffe ended up being unused (even though I had told CMake where to find Caffe).
- `apt install caffe-cuda` (if you have NVIDIA CUDA; For CPU version: `apt install caffe-cpu`)
- Just in case, I installed some suggested packages: `apt install libcaffe-cuda-dev python3-netcdf4 vitables`

--------------------------
Modular Tracking Framework
--------------------------

## Building
- Clone the repository from: https://github.com/abhineet123/MTF
- I created a custom installation directory:
  'FILEPATH/installation'
  - Subdirectories: 'bin', 'include', 'lib', 'matlab', and 'python'.
- I added this directory to my environment variables in '~/.bashrc':
  ```
  export LIBRARY_PATH=$LIBRARY_PATH:/FILEPATH/installation/lib/
  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/FILEPATH/installation/lib/
  export C_INCLUDE_PATH=$C_INCLUDE_PATH:/FILEPATH/installation/include/
  export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:/FILEPATH/installation/include/
  export PYTHONPATH=$PYTHONPATH:/FILEPATH/installation/python
  ```
- Open the repository directory in a shell session.
- Edit 'CMakeLists.txt' so that the right version of OpenCV will be used:
  Replace "find_package(OpenCV REQUIRED)" with "find_package(OpenCV REQUIRED PATHS FILEPATH/opencv-3.4.0/install NO_DEFAULT_PATH)"
- `mkdir build && cd build`
- `cmake .. -DMTF_INSTALL_DIR=FILEPATH/installation -DPY_VER=3.6 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6 -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so -DMTF_PY_INSTALL_DIR=FILEPATH/installation/python -DMTF_MEX_INSTALL_DIR=FILEPATH/installation/matlab -DWITH_VISP=ON -DWITH_THIRD_PARTY=OFF`
  - I set `WITH_THIRD_PARTY=OFF` because third party modules can have problems with OpenCV 3.x.
  - I set `WITH_VISP=ON`, but CMake said it went unused.
- `make -j4`

## Testing the C++ library
- Run `FILEPATH/installation/bin/runMTF`

## Testing the Python interface
- Run `python3 FILEPATH/Examples/python/runMTF2.py`

## Further steps for setting up the MATLAB interface
- Before launching MATLAB, run
  `export LD_LIBRARY_PATH=FILEPATH/opencv-3.4.0/install/lib:FILEPATH/installation/lib`
  - This will make MATLAB link the MEX-file containing MTF's MATLAB interface with the version of OpenCV that MTF was built against, and with the MTF shared library.
- Launch MATLAB (`matlab &`).
- Add the folder containing the MEX-file ('mexMTF2.mexa64', for Linux systems) to your MATLAB path ('FILEPATH/installation/matlab').
- Test if you can call MTF:
  `>> success = mexMTF2('init','pipeline c img_source u');`

### If it didn't work
- If the error is "undefined symbol", you haven't set `LD_LIBRARY_PATH` correctly.
- If the error is "Gateway function is missing", the following steps solved the problem for me.

#### Building the MEX-file from within MATLAB
- Install header files that, for some reason, weren't needed earlier: `apt install libgflags-dev libgoogle-glog-dev libprotobuf-dev`
- Compile with most optional features disabled (do this inside the MATLAB shell):
  `>> mex -c FILEPATH/Examples/cpp/mexMTF2.cc -IFILEPATH/installation/include -I/usr/include/eigen3 -IFILEPATH/opencv-3.4.0/install/include -DDISABLE_THIRD_PARTY_TRACKERS -DDISABLE_XVISION -DDISABLE_DFM -DDISABLE_VISP -DDISABLE_REGNET`
  - The point is to specify include directories for the MTF, Eigen, and OpenCV header files.
  - This should create an object file, 'mexMTF2.o'.
- In MATLAB, run, to create the MEX-file:
  `>> mex mexMTF2.o -LFILEPATH/opencv-3.4.0/install/lib -LFILEPATH/installation/lib -Ll/usr/lib/x86_64-linux-gnu -L/usr/lib/x86_64-linux-gnu/hdf5/serial -lopencv_calib3d -lopencv_core -lopencv_cudaarithm -lopencv_cudabgsegm -lopencv_cudacodec -lopencv_cudafeatures2d -lopencv_cudafilters -lopencv_cudaimgproc -lopencv_cudalegacy -lopencv_cudaobjdetect -lopencv_cudaoptflow -lopencv_cudastereo -lopencv_cudawarping -lopencv_cudev -lopencv_dnn -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_video -lopencv_videoio -lopencv_videostab -lboost_random -lboost_filesystem -lboost_system -lboost_thread -lhdf5 -lpthread -lsz -lz -ldl -lmtf`
  - You can tell what libraries need to be linked by running `ldd mexMTF2.mexa64` (on the MEX-file which didn't work in MATLAB).
  - I have NVIDIA CUDA on my system. If you don't have it, you will probably not have to link the "\*cuda\*" libraries.
- I probably could have repeated the above build steps with VISP and DFM enabled if I had specified the appropriate include and library directories, and library linking directives.