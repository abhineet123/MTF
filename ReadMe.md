Welcome to the home of **Modular Tracking Framework (MTF)** - a highly efficient and extensible library for
[**registration**](https://en.wikipedia.org/wiki/Image_registration) based [**tracking**](https://www.youtube.com/watch?v=WzXhCCCcbTQ), also called [direct visual tracking](https://www.youtube.com/watch?v=mk1j7CTkXoo).
MTF utilizes a modular decomposition of trackers in this domain wherein each tracker comprises the following 3 modules:

1. **Search Method (SM)**: [ESM](http://far.in.tum.de/pub/benhimane2007ijcv/benhimane2007ijcv.pdf), [IC](http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=990652&url=http%3A%2F%2Fieeexplore.ieee.org%2Fiel5%2F7768%2F21353%2F00990652.pdf%3Farnumber%3D990652), [IA](http://dl.acm.org/citation.cfm?id=290123), [FC](http://link.springer.com/article/10.1023%2FA%3A1008195814169), [FA](http://dl.acm.org/citation.cfm?id=1623280), [LM](https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm), [NN](http://www.roboticsproceedings.org/rss09/p44.pdf), [PF](http://ieeexplore.ieee.org/document/6589599/?tp=&arnumber=6589599), [RANSAC](http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=7158323&url=http%3A%2F%2Fieeexplore.ieee.org%2Fiel7%2F7158225%2F7158304%2F07158323.pdf%3Farnumber%3D7158323) or [LMS](http://www.tandfonline.com/doi/abs/10.1080/01621459.1984.10477105)
2. **Appearance Model (AM)**: [SSD](http://dl.acm.org/citation.cfm?id=290123), [SAD](https://en.wikipedia.org/wiki/Sum_of_absolute_differences), [ZNCC](http://www.sciencedirect.com/science/article/pii/S0031320305000592), [SCV](http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=6094650&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D6094650), [LSCV](http://ieeexplore.ieee.org/document/7025074/), [NCC](http://link.springer.com/chapter/10.1007%2F978-3-642-33783-3_32), [MI](https://www.irisa.fr/lagadic/pdf/2010_ismar_dame.pdf), [CCRE](http://link.springer.com/article/10.1007%2Fs11263-006-0011-2), [KLD](https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence), [SSIM](http://link.springer.com/article/10.1007/s00138-007-0107-x), [SPSS](http://arxiv.org/abs/1607.04673), [RIU](http://www.ncbi.nlm.nih.gov/pubmed/1629424), [NGF](http://link.springer.com/chapter/10.1007%2F11866763_89), [PCA](http://link.springer.com/article/10.1007/s11263-007-0075-7) or [DFM](http://webdocs.cs.ualberta.ca/~vis/mtf/dfm_report.pdf)
    *  **Illumination Model (ILM)**: [GB](http://www.ncbi.nlm.nih.gov/pubmed/18988945), [PGB](http://ieeexplore.ieee.org/document/4270018/) and [RBF](http://ieeexplore.ieee.org/document/4270018/)
3. **State Space Model (SSM)**: [Spline](http://www.hpl.hp.com/techreports/Compaq-DEC/CRL-94-1.html) (50+ DOF), [TPS](http://dl.acm.org/citation.cfm?id=66134) (50+ DOF), [Homography](https://en.wikipedia.org/wiki/Homography) (8 DOF), [Affine](https://en.wikipedia.org/wiki/Affine_transformation) (6 DOF), ASRT (Anisotropic Scaling, Rotation and Translation - 5 DOF ), [Similitude](https://en.wikipedia.org/wiki/Similarity_%28geometry%29) (4 DOF), AST (Anisotropic Scaling and Translation - 4 DOF), [Isometry](http://mathworld.wolfram.com/Isometry.html) (3 DOF), IST (Isotropic Scaling and Translation - 3 DOF ) or [Translation](https://en.wikipedia.org/wiki/Translation_%28geometry%29) (2 DOF)

Please refer these papers:
[[**cviu**]](http://webdocs.cs.ualberta.ca/~vis/mtf/mtf_cviu.pdf)
[[**iros17**]](http://webdocs.cs.ualberta.ca/~vis/mtf/mtf_iros17.pdf)
for details on the system design, these:
[[**crv16**]](http://webdocs.cs.ualberta.ca/~vis/mtf/modular_crv16.pdf)[[**wacv17**]](http://webdocs.cs.ualberta.ca/~vis/mtf/ssim_wacv17.pdf)[[**crv17**]](http://webdocs.cs.ualberta.ca/~vis/mtf/rsst_crv17.pdf)
for some performance results and [**this thesis**](http://webdocs.cs.ualberta.ca/~vis/mtf/mtf_thesis.pdf) for a comprehensive description. There is also a [**dedicated website**](http://webdocs.cs.ualberta.ca/~vis/mtf/) where Doxygen documentation will soon be available along with detailed tutorials and examples. It also provides several datasets formatted to work with MTF.

The library is implemented entirely in **C++** though interfaces for **Python** and **MATLAB** are also provided to aid its use in research applications.
A simple interface for [**ROS**](http://www.ros.org/) is likewise provided for seamless integration with robotics projects.
In addition to the registration tracking modules, MTF comes bundled with several state of the art learning and detection based trackers whose C++ implementations are publicly available - [DSST](http://www.cvl.isy.liu.se/en/research/objrec/visualtracking/scalvistrack/index.html), [KCF](http://home.isr.uc.pt/~henriques/circulant/), [CMT](http://www.gnebehay.com/cmt/), [TLD](http://www.gnebehay.com/tld/), [RCT](http://www4.comp.polyu.edu.hk/~cslzhang/CT/CT.htm), [MIL](http://vision.ucsd.edu/~bbabenko/project_miltrack.html), [Struck](http://www.samhare.net/research/struck), [FragTrack](http://www.cs.technion.ac.il/~amita/fragtrack/fragtrack.htm), [GOTURN](https://github.com/davheld/GOTURN) and [DFT](http://cvlab.epfl.ch/page-107683-en.html).
It can thus be used as a general purpose tracking test bed too. 

MTF supports both **Unix** and **Windows** platforms. Though the Unix build system has been tested comprehensively only under Linux, specifically Ubuntu 14.04/16.04, it should work on Macintosh systems too (see [Compile/Runtime Notes](#compileruntime-notes) section below for resolving possible issues).
The Windows build system has been tested on Windows 8.1 with Visual Studio 2015 though it should work fine with any recent versions of the OS and IDE.

MTF is provided under [BSD license](https://opensource.org/licenses/BSD-3-Clause) and so is free for research and commercial applications. We do request, however, that [this paper](http://webdocs.cs.ualberta.ca/~vis/mtf/mtf_iros17.pdf) [[bibtex](http://webdocs.cs.ualberta.ca/~vis/mtf/mtf_iros_bibtex.txt)] [[arxiv version](https://arxiv.org/abs/1602.09130)] be cited by any publications resulting from projects that use MTF so more people can get to know about and benefit from it.
Finally, if any issues are encountered while installing or running the library, please create an entry in the [issues](https://github.com/abhineet123/MTF/issues) section and we will do our best to resolve it as soon as possible.

Installation:
-------------
* **General Prerequisites**:
    *  MTF uses some [C++11](https://en.wikipedia.org/wiki/C%2B%2B11) features so a supporting compiler is needed ([GCC 4.7](https://gcc.gnu.org/projects/cxx0x.html) or newer)
    * [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) should be installed and added to the C/C++ include paths. This can be done, for instance, by running the following commands, assuming that Eigen is installed in _/usr/include/eigen3_:  
	`echo "export C_INCLUDE_PATH=${C_INCLUDE_PATH}:/usr/include/eigen3" >> ~/.bashrc`  
	`echo "export CPLUS_INCLUDE_PATH=${CPLUS_INCLUDE_PATH}:/usr/include/eigen3" >> ~/.bashrc`
	    - version 3.2.10 or newer is needed
    * [OpenCV](http://opencv.org/) should be installed.
		- comprehensive testing has only been done with OpenCV 2.4.13 and 3.3.0; 
		- 2.4.x is recommended for widest compatibility as rare issues with OpenCV 3.x have been known to prevent successful compilation with some configurations (see [Compile/Runtime Notes](#compileruntime-notes) section below)
		- 3.3.x or 3.4.x is recommended for widest functionality
		- 4.x.x is **not** supported
		- the [nonfree](http://docs.opencv.org/2.4/modules/nonfree/doc/nonfree.html) / [contrib](https://github.com/opencv/opencv_contrib) module should be [installed too](http://stackoverflow.com/a/31097788) if the corresponding feature detectors and descriptors (SIFT and SURF) are to be available in the feature tracker
    * [Boost](http://www.boost.org/) should be installed
    * [FLANN](http://www.cs.ubc.ca/research/flann/) should be installed for the NN search method
	    - Compilation from source of version **1.8.4**  available on the [website](http://www.cs.ubc.ca/research/flann/#download) is recommended as the version in the Ubuntu (and git) repo might give linker errors ([Issue #3](https://github.com/abhineet123/MTF/issues/3)).
	    - NN can be disabled at compile time using `nn=0` switch if FLANN is not available (see [compile time switches](#compile-time-switches)).
    * [Intel TBB](https://www.threadingbuildingblocks.org/) / [OpenMP](http://openmp.org/wp/) should be installed if parallelization is to be enabled.
    * [ViSP](https://visp.inria.fr/) should be installed if its [template tracker module](https://visp.inria.fr/template-tracking/) or [input pipeline](http://visp-doc.inria.fr/doxygen/visp-3.0.0/group__group__io__video.html) is enabled during compilation (see [compile time switches](#compile-time-switches)).
	    - Note that [version 3.0.0](http://gforge.inria.fr/frs/download.php/latestfile/475/visp-3.0.0.zip)+ is required. The Ubuntu apt package is 2.8 and is therefore incompatible.
	    - ViSP is also needed for interactive operations in the multi threaded versions of the Matlab and Python interfaces as OpenCV window system has an unresolved bug/incompatibility with boost threading library which causes it to hangup or crash the program.
    * [Caffe](http://caffe.berkeleyvision.org/) is needed for some optional modules including FMaps, Regnet and GOTURN if these are enabled during compilation
	* [Xvision](https://github.com/abhineet123/Xvision2) should be installed if it is enabled during compilation (see [compile time switches](#compile-time-switches)).
	    - **Not recommended** as Xvision is very difficult to install and configure on modern systems
* Additional installation instructions for Ubuntu 18.04 with Matlab 2018b are available [here](Docs/MTF_setup_Ubuntu18_Matlab2018b.md). These may contain solutions for some issues on newer platforms that are not documented here.
* **Prerequisites in Windows**
	* Using CMake with Visual Studio (**Recommended**)
		- Install a recent version of Visual Studio
			- the installation has been tested comprehensively with Visual Studio 2015 but any recent version (2013 or newer) should work fine; 
			- it seems that the `OpenCVConfig.cmake` file that comes with the latest version of OpenCV simply does not recognize Visual C++ 2017 so cmake might not be able to find it and instead exit with the erroneous message: `Found OpenCV Windows Pack but it has not binaries compatible with your configuration`; a similar issue is exhibited by ViSP so it is best to use 2015 or older versions of Visual Studio; a possible way to resolve this issue is in the [Compile/Runtime Notes](#compileruntime-notes) section if a newer version must be used.		
			- the freely available [Express/Community edition](https://www.visualstudio.com/vs/visual-studio-express/) can be used too
		- Set `EIGEN3_ROOT`, `EIGEN3_DIR` and `EIGEN3_ROOT_DIR` [environment variables](http://www.computerhope.com/issues/ch000549.htm) to the folder containing the Eigen header files
		    - this folder should contain the sub folders `Eigen` and `unsupported` and a file called `signature_of_eigen3_matrix_library`
			- if cmake still fails to find Eigen, set `EIGEN_INCLUDE_DIR_WIN` variable in `CMakeLists.txt` (line 13) to the location of this folder
		- [Build OpenCV from source using Visual Studio](http://docs.opencv.org/2.4/doc/tutorials/introduction/windows_install/windows_install.html#installation-by-making-your-own-libraries-from-the-source-files) or download the [pre built binaries](http://sourceforge.net/projects/opencvlibrary/files/opencv-win/) built with the correct version of visual studio if available.	
		    - as mentioned before, version 2.4.x is recommended; if 3.x is used instead and compile time errors occur, try disabling third party trackers by passing the switch `-DWITH_THIRD_PARTY=0` to cmake
			- [set environment variable](http://www.computerhope.com/issues/ch000549.htm) `OpenCV_DIR` to the folder where the binaries have been built or downloaded; this should contain the `OpenCVConfig.cmake` file along with the `lib` and `bin` folders
			- add the location of the `bin` folder to the `Path` environment variable
			- if cmake still fails to find OpenCV, set `OPENCV_INCLUDE_DIR_WIN`, `OPENCV_LIB_DIR_WIN` and `OpenCV_SUFFIX` variables in `CMakeLists.txt` (line 14-16) to the respective values
		- [Build Boost from source using Visual Studio (`toolset=msvc`)](http://www.boost.org/doc/libs/1_63_0/more/getting_started/windows.html#prepare-to-use-a-boost-library-binary).
			- using the steps under [Simplified Build From Source](http://www.boost.org/doc/libs/1_63_0/more/getting_started/windows.html#simplified-build-from-source) should work too
			- set `BOOST_INCLUDEDIR` and `BOOST_LIBRARYDIR` environment variables to the locations of the header and library (*.lib) files respectively.
			- add the location of the `bin` folder containing the *.dll files to the `Path` environment variable
			- if cmake still fails to find Boost, set `BOOST_INCLUDE_DIR_WIN`, `BOOST_LIB_DIR_WIN` and `Boost_SUFFIX` variables in `CMakeLists.txt` (line 17-19) to the respective values		
		- If [FLANN](http://www.cs.ubc.ca/research/flann/) is needed, build it from source using Visual Studio and set `FLANN_ROOT` environment variable to the folder containing the compiled library and header files respectively
			- this folder should have sub folders called `lib`, `bin` and `include` that contain the *.lib, *.dll and the header files 
			- add the `bin` sub folder to the `Path` environment variable
		- If [ViSP](http://www.cs.ubc.ca/research/flann/) is needed, [build ViSP from source using Visual Studio](http://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-win10-msvc14.html) and set `VISP_DIR` environment variable to the folder containing the library and header files
			- this folder should have a file called `VISPConfig.cmake` along with sub folders called `x86/vc<version>/lib`, `x86/vc<version>/bin` and `include` that contain the *.lib, *.dll and the header files respectively.
			- add the path of the `bin` folder to the `Path` environment variable	
		- if the Python interface is to be built, install [Python 2.7.x](https://www.python.org/downloads/windows/) and [Numpy](http://www.numpy.org/).
		    - CMake should normally be able to find the needed Python/Numpy libraries and headers without any other setup
			- if any problems are encountered, please open an issue to let us know about it
		- MATLAB interface
			- the MATLAB finding module of cmake has a [limitation](https://cmake.org/Bug/view.php?id=15786) that it can only find a MATLAB installation of the same build type as the target build, i.e. a 32 bit target build (default) can only find a 32 bit installation of MATLAB and same for 64 bit
			- the recommended approach is to [install 32 bit version of MATLAB](https://se.mathworks.com/matlabcentral/answers/5407-install-both-32bit-and-64bit-versions-of-matlab-on-windows-7) but if that is not possible, a 64 bit CMake target build can be specified using, for instance the instructions given [here](https://stackoverflow.com/questions/28350214/how-to-build-x86-and-or-x64-on-windows-from-command-line-with-cmake)
				- 64 bit builds have not been tested so cannot be guaranteed to work;
				- all dependencies - OpenCV, Boost, FLANN and ViSP - must also be built using 64 bit builds for the linking to work
			- if multiple MATLAB versions are installed or if the installed version is not found, set variable `Matlab_ROOT_DIR` or `Matlab_ROOT_DIR_DEFAULT` in Examples.cmake to the MATLAB installation folder to help CMake find the correct version
			- the module is installed to `<Matlab_ROOT_DIR>/toolbox/local` by default; this location may need to be [added to the MATLAB path](https://se.mathworks.com/help/matlab/ref/addpath.html) for it to be found;
		
	* Using GNU Make with MinGW
	    - **Note**:  This method is not recommended due to the following issues:
		    - it is known to run into "out of memory" exceptions while compiling some of the executables
		    - importing pyMTF from a Python script results in an error: `ImportError: DLL load failed: A dynamic link library (DLL) initialization routine failed.` due to an obscure incompatibility issue between the python import system and third party modules compiled with MinGW.
		- Install [MinGW](http://www.mingw.org/), [Make for Windows](http://gnuwin32.sourceforge.net/packages/make.htm) and [CoreUtils for Windows](http://gnuwin32.sourceforge.net/packages/coreutils.htm)
		- [Build OpenCV from source using MinGW](http://kevinhughes.ca/tutorials/opencv-install-on-windows-with-codeblocks-and-mingw) and set `OPENCV_INCLUDE_DIRS` and `OPENCV_LIB_DIRS` in the makefile to the locations of header and library files respectively. 
		- [Build Boost from source using MinGW (`toolset=gcc`)](http://www.boost.org/doc/libs/1_63_0/more/getting_started/windows.html#prepare-to-use-a-boost-library-binary) and set `BOOST_LIB_DIRS` and `BOOST_INCLUDE_DIRS` in the makefile to the locations of the header and library (`*.dll.a`) files respectively.
		- Set `EIGEN_INCLUDE_DIRS` in the makefile to the location containing Eigen header files. 
		- Set appropriate values for `BOOST_LIBS_SUFFIX`, and `OPENCV_LIBS_SUFFIX` based on the installed versions of the respective libraries.
		- if the Python interface is to be built, install [Python 2.7.x](https://www.python.org/downloads/windows/) and [Numpy](http://www.numpy.org/) and set appropriate locations for `PYTHON_INCLUDE_DIR`, `PYTHON_LIBS_DIR`, `NUMPY_INCLUDE_DIR` and `MTF_PY_INSTALL_DIR` in `Examples/Examples.mak`.
		- Add the `bin` folders of the MinGW and GnuWin32 installations (e.g. `C:\MinGW\bin`, `C:\Program Files (x86)\GnuWin32\bin`) along with folders containing the `*.dll` files of OpenCV and Boost installations (e.g. `C:\OpenCV\build\x86\mingw\bin`) to the [`PATH` environment variable](https://www.java.com/en/download/help/path.xml) and reboot the system for the changes to take effect.
		
* **Download** the source code as zip file or clone using:  
    `git clone https://github.com/abhineet123/MTF.git`
	
* **Recommended Installation**  
 MTF comes with both a [make](https://www.gnu.org/software/make/) and a [cmake](https://cmake.org/) build system but the latter is recommended for most users of the library and can be used as follows:
    * use the [standard method](https://cmake.org/runningcmake/) (i.e. ``mkdir build && cd build && cmake ..``) to create the makefile for Unix or Visual Studio solution for Windows
	* **Unix**: use `make` to compile (`make -j<n>` for _n_-threaded compilation) and `sudo make install` to install
    * **Windows**: open `MTF.sln` in Visual Studio and run `Build->Build Solution`
	    - if needed, change the solution configuration to `Release` from `Debug` to build the faster version
		- right click on `Solution Explorer->INSTALL` and select `Build` to install the library, header and executable files to the installation directory (`C:/MTF` by default); add `<installation_folder>/bin` to `Path` environment variable to be able to run the executables from any location;
		
	The make system might be used by developers/contributors as it can potentially offer a finer level of control. Note, however, that it is not tested as well as the cmake system so is not guaranteed to work in all configurations or be up to date with the latest changes.
		
* **Advanced Installation**  
Following are commands and switches for both make and cmake systems to customize the installation by disabling parts of the library or to only compile specific components. It also contains general information about the Python and Matlab wrappers and example applications. Not all of the make commands may work with the cmake system.
    * `make` or `make mtf` : compiles the shared library (_libmtf.so_) to the build directory (_Build/Release_)
    * `make install` : compiles the shared library if needed and copies it to _/usr/local/lib_; also copies the headers to _/usr/local/include_; (use `make install_lib` or `make install_header` for only one of the two); if third party trackers are enabled, their respective library files will be installed too;
	    - this needs administrative (sudo) privilege; if not available, the variable `MTF_INSTALL_DIR` in the makefile can be modified to install elsewhere. This can be done either by editing the file itself or providing it with the make/cmake command as:  
		`make install MTF_INSTALL_DIR=<installation_dir>`  
		`cmake .. -D MTF_INSTALL_DIR=<installation_dir>`
		    - the `lib` and `include` sub folders within this folder should be present in LD_LIBRARY_PATH and C_INCLUDE_PATH/CPLUS_INCLUDE_PATH environment variables respectively so any application using MTF can find the library and headers.
    * `make exe`/`make install_exe` : compile/install the main example file _Examples/cpp/runMTF.cc_ to create an executable called `runMTF` that uses this library to track objects. 
	    - library should be installed before running this, otherwise linking will not succeed
		- installation folder is _/usr/local/bin_ by default; this needs administrative privilege too - change `MTF_INSTALL_DIR` as above if this is not available
    * `make mtfi` : all of the above 
    * `make py`/`make install_py` (`make py2`/`make install_py2`) : compile/install the **Python interface** to MTF (or its multi threaded version) - this creates a Python module called _pyMTF_ (_pyMTF2_) that serves as a front end for using MTF from Python.
	    - usage of this module is demonstrated in _Examples/python/runMTF.py_ (_runMTF2.py_) and function specification is in the ReadMe in the _Examples_ folder 
		- supports both Python 2.7 and 3.x but only a particular version is built at a time - this can be specified using the `PY_VER`/`py_ver` cmake/make variable (set to 2.7 by default) (e.g. `-D PY_VER=3.5` for Python 3.5)
		- if multiple Python versions are installed on the system, cmake might not be able to find the desired one even if `PY_VER` is provided; in this case Python header and library locations must be provided though the variables `PYTHON_INCLUDE_DIR` and `PYTHON_LIBRARY` (e.g. `-D PYTHON_INCLUDE_DIR=/usr/include/python3.5 -D PYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.5m.so` for Python 3.5)
		- neither cmake nor make systems currently support inferring the correct install location for _pyMTF_ from Python library/header paths so this must be specified through `MTF_PY_INSTALL_DIR` (in _Examples.mak/cmake_) if using a custom Python version (defaults to _/usr/local/lib/python2.7/dist-packages/_)
		- refer to _cmake/cmake.md_ for an example cmake command to compile _pyMTF_ with Python 3.5;
		- _pyMTF2_ needs ViSP to be installed and enabled during compilation for its interactive operations
		- there is also an additional wrapper for _pyMTF2_ input pipeline called _mtf.VideoCapture_ that provides an interface similar to _cv2.VideoCapture_ but can be used, through ViSP pipeline, to access USB and firewire cameras with custom resolution and other settings that are unavailable in _cv2_.
		It can be installed by running `python setup.py install` or `python3 setup.py install` from _Examples/python_ folder.
		Its usage is demonstrated in the ReadMe in _Examples_ folder.
    * `make mex` (`make mex2`) : compile the **MATLAB interface** to MTF (or its multi threaded version) - this creates a MATLAB module called _mexMTF_ (_mexMTF2_) that serves as a front end for using MTF from MATLAB.
	    - set `MATLAB_DIR` variable in _Examples.mak_ to the root of the MATLAB installation folder 
		- usage of this module is demonstrated in _Examples/matlab/runMTF.m_ (_runMTF2.m_) and function specification is in the ReadMe in the _Examples_ folder
	    -  installation location can be specified through `MTF_MEX_INSTALL_DIR` in _Examples.mak/cmake_ (defaults to _<MATLAB_DIR>/toolbox/local_) 
		- if CMake is being used under Unix, the restriction of matching build target and MATLAB installation type mentioned in the Windows installation section applies here too; however, it is possible to compile `mexMTF` in Unix even if MATLAB is not detected by CMake by running the command written to a file called `mtf_mex_cmd.txt` in the build folder;
			- the command can be run at the MATLAB prompt or even at the terminal if the location of the `mex` executable is present in the `PATH` environment variable;
			- a few changes need to be made to it first as detailed in the CMake message;
		- _mexMTF2_ needs ViSP to be installed and enabled during compilation for its interactive operations
    * `make uav`/`make install_uav` : compile/install an application called `trackUAVTrajectory` that tracks the trajectory of a UAV in a satellite image of the area over which it flew while capturing images from above
    * `make mos`/`make install_mos` : compile/install an application called `createMosaic` that constructs a live mosaic from a video of the region to be stitched
    * `make qr`/`make install_qr` : compile/install an application called `trackMarkers` that automatically detects one or more markers in the input stream and starts tracking them
	    - this uses the Feature tracker so will only compile if that is enabled (see below)
	    - this reads marker images from `Data/Markers` folder by default; this can be changed by adjusting `qr_root_dir` in `Config/examples.cfg` where the names of marker files and the number of markers can also be specified along with some other parameters;
    * `make all`/`make install_all` : compile/install all example applications that come with MTF along with the Python and Matlab interfaces
	* `make app app=<APPLICATION_NAME>`: build a custom application that uses MTF with its source code located in `<APPLICATION_NAME>.cc`; the compiled executable goes in the build directory;
	    - location of the source file (<APPLICATION_NAME>.cc) can be specified through `MTF_APP_SRC_DIR` (defaults to _Examples/cpp_)
	    - `make mtfa` will install it too - installation location can be specified through `MTF_APP_INSTALL_DIR` (defaults to the current folder)
    * <a name="compile-time-switches"/>**Compile time switches**</a> for all of the above commands (the equivalent cmake options, where such exist, are given within parenthesis):
	    - `only_nt=1`/`nt=1`(`WITH_TEMPLATED=OFF`) will enable only the **Non Templated (NT)** implementations of SMs and disable their templated versions that are extremely time consuming to compile though being faster at runtime (disabled by default)
		    - can be very useful for rapid debugging of AMs and SSMs where headers need to be modified;
		    - the NT implementation of NN only works with GNN since FLANN library needs its object to be templated on the AM; 
	    - `nn=0`(`WITH_FLANN=OFF`) will disable the templated implementation of NN search method (enabled by default).
		    - should be specified if FLANN is not available
		    - FLANN has some compatibility issues under Windows so this is disabled by default;
	    - `spi=1`(`WITH_SPI=ON`) will enable support for selective pixel integration in modules that support it (disabled by default)
		    -  currently only SSD and NCC AMs support this along with all the SSMs
		    -  this might decrease the performance slightly when not using SPI because some optimizations of Eigen cannot be used with SPI
	    - `grid=0`(`WITH_GRID_TRACKERS=OFF`) will disable the Grid trackers and RKLT (enabled by default).
	    - `feat=0`(`WITH_FEAT=OFF`) will disable the Feature tracker (enabled by default).
		    -  this uses optional functionality in the [nonfree](http://docs.opencv.org/2.4/modules/nonfree/doc/nonfree.html) / [contrib](https://github.com/opencv/opencv_contrib) module of OpenCV so this should be [installed too](http://stackoverflow.com/a/31097788) if these are to be available.
	    - `lt=0`(`WITH_THIRD_PARTY=OFF`) will disable the third party open source learning based trackers - [DSST](http://www.cvl.isy.liu.se/en/research/objrec/visualtracking/scalvistrack/index.html), [KCF](http://home.isr.uc.pt/~henriques/circulant/), [CMT](http://www.gnebehay.com/cmt/), [TLD](http://www.gnebehay.com/tld/), [RCT](http://www4.comp.polyu.edu.hk/~cslzhang/CT/CT.htm), [MIL](http://vision.ucsd.edu/~bbabenko/project_miltrack.html), [Struck](http://www.samhare.net/research/struck), [FragTrack](http://www.cs.technion.ac.il/~amita/fragtrack/fragtrack.htm), [GOTURN](https://github.com/davheld/GOTURN) and [DFT](http://cvlab.epfl.ch/page-107683-en.html) - that are also bundled with this library (in _ThirdParty_ subfolder) (enabled by default except MIL, DFT and GOTURN).
		    - several third party trackers that have a CMake build system do not compile under Windows yet and are thus not available;
	    - `gtrn=1`(`WITH_GOTURN=ON`) will enable [GOTURN](https://github.com/davheld/GOTURN) deep learning based tracker (disabled by default)
			- requires [Caffe]((http://caffe.berkeleyvision.org/)) to be installed and configured
			    * if not installed in `~/caffe/build/install`, specify `CAFFE_INSTALL_DIR` either at compile time or by editing `ThirdParty/GOTURN/GOTURN.mak`
			- optional data for use with it is available [here](http://webdocs.cs.ualberta.ca/~vis/mtf/mtf_data.zip) - this should be extracted inside the MTF root folder before running GOTURN
	    - `dft=1`(`WITH_DFT=ON`) will enable the [Descriptor Fields Tracker](http://cvlab.epfl.ch/page-107683-en.html) (disabled by default)
            * this is known to have issues with OpenCV 3.x so should not be enabled if that is being used
	    - `mil=1`(`WITH_MIL=ON`) will enable the [Multiple Instance Learning tracker](http://vision.ucsd.edu/~bbabenko/project_miltrack.html) (disabled by default)
            * this too is known to have issues with OpenCV 3.x so should not be enabled if that is being used
		- `vp=1`(`WITH_VISP=ON`) will enable ViSP template trackers and input pipeline (disabled by default).
            * if a runtime linking error is encountered, add ViSP library install path to `LD_LIBRARY_PATH` variable by running, for instance, `echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/x86_64-linux-gnu/" >> ~/.bashrc`
	    - `xv=1`(`WITH_XVISION=ON`) will enable [Xvision](http://www.cs.jhu.edu/CIPS/xvision/) trackers and input pipeline (disabled by default and not recommended).
	    - `o=0` will compile the library in debug mode (_libmtf_debug.so_) and disable optimization - the corresponding executables will be suffixed with 'd' ( for example _runMTFd_)
		- `ver=<version_postfix>` will postfix all compiled library and executable files with the provided postfix so that multiple versions may coexist on the same system without conflict; e.g. `ver=2` will create libmtf_v2.so and runMTF_v2 and same for all libraries created during MTF compilation (including those for third party trackers);
		- `header_only=1` can be used with `make exe` or `make mtfe` to build a stand alone version of _runMTF_ called _runMTFh_ that does not need to link with _libmtf.so_
            * this disables third party trackers automatically as several of those have their own libraries built from within MTF
			* it will still need to link against external libraries that it uses (like OpenCV, FLANN and boost)
			* it may take some time (20-30 minutes) to compile as the entire library has to be consolidated into a single executable
			* this version should be faster in theory as the compiler has access to all code at once and can thus optimize better but tests so far have not found any significant difference in speed from the standard version
    * Clean up commands:
	    * `make clean` : removes all the .o files and the .so file created during compilation from the Build folder.
        * `make mtfc` : also removes the executable.

Compile/Runtime Notes:
----------------------
* Using Visual Studio 2017 or newer to build OpenCV 3.x.x might lead to an error like `Found OpenCV Windows Pack but it has not binaries compatible with your configuration` in cmake. This can be resolved using the following steps. These are for 64 bit build of OpenCV 3.4.1 with Visual Studio 2019 but adapting for other versions should be straightforward.
    * add 
    
        ```
          elseif(MSVC_VERSION MATCHES "^192[0-9]$")
           set(OpenCV_RUNTIME vc16)
        ```
    right after

        ```
        elseif(MSVC_VERSION EQUAL 1900)
        set(OpenCV_RUNTIME vc14)
        elseif(MSVC_VERSION MATCHES "^191[0-9]$")
        set(OpenCV_RUNTIME vc15)
        ```

        in `OpenCVConfig.cmake`.    
    * move bin, lib and include folders to `x64/vc16/` after creating the same.
* if an error like `cannot find -l<name of library>` (for instance `cannot find -lhdf5` or `cannot find -lmtf`) occurs at compile time, location of the corresponding library should be added to **LIBRARY_PATH** and **LD_LIBRARY_PATH** environment variables in the **.bashrc** file. 
    * For instance HDF5 installs in _/usr/local/hdf5/lib_ by default so  following commands can be run to add its path there:  
	`echo "export LIBRARY_PATH=$LIBRARY_PATH:/usr/local/hdf5/lib" >> ~/.bashrc`  
	`echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/hdf5/lib" >> ~/.bashrc`  
	Similarly, MTF installs to _/usr/local/lib_ by default so following can be run for it:   
	`echo "export LIBRARY_PATH=$LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc`  
	`echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib" >> ~/.bashrc`  
	* The bashrc file should be reloaded by either restarting the terminal or running `. ~/.bashrc` and compilation should be tried again.
* similar compile time errors pertaining to missing header files may be encountered in which case the location of the `include` folder of the corresponding library should be added to **C_INCLUDE_PATH** and **CPLUS_INCLUDE_PATH** variables. For instance, following commands can be run for HDF5:  
	`echo "export C_INCLUDE_PATH=$C_INCLUDE_PATH:/usr/local/hdf5/include" >> ~/.bashrc`  
	`echo "export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:/usr/local/hdf5/include" >> ~/.bashrc`  
* if a runtime error like `cannot find libmtf` or something similar is encountered while running `runMTF`, _/usr/local/lib_ should be added to **LIBRARY_PATH** and **LD_LIBRARY_PATH** environment variables as above (assuming the default MTF installation directory has not been changed - otherwise the new path should be used instead).
* if a compile time error similar to this occurs: `/usr/include/eigen3/Eigen/src/Core/util/BlasUtil.h:233:98: error: no matching function for call to ‘Eigen::internal::blas_traits`, please update Eigen to the latest version;
* if a compile time error similar to this occurs:  
`/usr/bin/ld: cannot find -lQt5::Core`  
`/usr/bin/ld: cannot find -lQt5::Gui`  
`/usr/bin/ld: cannot find -lQt5::Widgets`  
`/usr/bin/ld: cannot find -lQt5::Test`  
`/usr/bin/ld: cannot find -lQt5::Concurrent`  
`/usr/bin/ld: cannot find -lQt5::OpenGL`  
there is probably something wrong with the OpenCV installation. Reinstalling that should fix it (refer to issue #13).
* ViSP cmake system has a bug in Ubuntu 16.04 so if a cmake error of type `Found ViSP for Windows but it has no binaries compatible with your
  configuration. You should manually point CMake variable VISP_DIR to your build of ViSP library.` occurs, add `-DVISP_DIR=<path to ViSP build folder>` to the cmake command and rerun after removing the existing cmake files
* A compile time error similar to this:  
`error: no matching function for call to vpDisplay::init(vpImage<vpRGBa>&, int, int, std::string&)`  
probably indicates that an outdated version of ViSP is installed. This can be resolved by either updating it to 3.0.1 or newer or using `-DWITH_VISP=0`/`vp=0` cmake/make option to disable ViSP.
* A compile time error similar to this:  
`undefined reference to LZ4_compress_HC_continue`  
occurs when the cmake module for one of the dependencies is not configured properly and does not return one or more libraries it needs to be linked against (`liblz4` in this case).
    * This can be resolved by adding `-D MTF_LIBS=<name_of_missing_library>` (in this case: `-D MTF_LIBS=lz4`) to the cmake command.
    * The corresponding library might also need to be installed, for instance, by using `sudo apt-get install liblz4-dev`
* if a run time error similar to this:  
`undefined symbol: _ZN3cmt7Matcher10initializeERKSt6vectorIN2cv6Point_IfEESaIS4_EENS2_3MatERKS1_IiSaIiEES9_S4_`  
suddenly occurs or running `runMTF` causes segmentation fault with no messages **right after installing ROS**, it is probably because MTF is linking with an incorrect version of OpenCV installed by ROS.
    * The only known solution guaranteed to work is to uninstall either the original or the ROS version of OpenCV and then recompile MTF.
    * Possible ways to have both versions installed and have MTF link with the correct one are suggested [here](https://stackoverflow.com/questions/29479379/opencv-linking-problems-with-ros). These have not been tested with MTF so might not work.	

* **Using MTF on Macintosh Systems**
	* if make build system is used, some third party modules might not compile successfully if cmake uses clang instead of gcc as the default compiler; in such cases compiling with `lt=0` to disable all third party modules is the best option.
    * if cmake build system is used, then following commands must be run so that cmake uses gcc during compilation:  
	`export CC=/usr/bin/gcc`  
	`export CXX=/usr/bin/g++` 
	* if FLANN based NN is enabled with cmake, environment variables may need to be set for hdf5 to be found using, for instance:  
	`export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/Applications/HDF_Group/HDF5/1.10.0-patch1/`
* **Using MTF with OpenCV 3.x**
    * MTF has been tested and found to work on Ubuntu 14.04 with OpenCV 3.1 installed using [this method (option 2)](http://milq.github.io/install-opencv-ubuntu-debian/) and OpenCV 3.3 installed using [this method](https://www.learnopencv.com/install-opencv3-on-ubuntu/)  but general compatibility with all system configurations is not guaranteed.
    * third party trackers in particular are likely to have issues with OpenCV 3.x as many use legacy versions so these should be disabled using `lt=0`/`-DWITH_THIRD_PARTY=OFF` if compilation or linking errors pertaining to these are found.
    * if a linking error of type `/usr/bin/ld: cannot find -lippicv` occurs, remove `-ippicv` from opencv pkg-config configuration file which is usually located at `/usr/local/lib/pkgconfig/opencv.pc` or follow the procedures suggested [here](http://answers.opencv.org/question/84265/compiling-error-with-lippicv/)
* **only make build system** : if MTF had earlier been compiled with `only_nt=0` (or `nn=1`) and is recompiled with `only_nt=1` (or `nn=0`) such that the **executable does not get recompiled**, a runtime error of the type: `symbol lookup error: runMTF: undefined symbol:` may occur on running the executable; to resolve this, remove `runMTF` (or whatever executable is causing the error) from the build folder (Build/Release or Build/Debug) by running, for instance `rm Build/Release/runMTF` and recompile; this happens because using `only_nt=1` or `nn=0` turns off part of the code in the library but the executable will continue to expect these to be present till it is recompiled;
* if compilation is taking too long and optional components are not needed, following cmake/make commands can be used to build a minimal version of MTF with full core functionality:  
	`cmake .. -DWITH_THIRD_PARTY=OFF -DWITH_TEMPLATED=OFF`  
	`make mtfall lt=0 nt=1`  
    * if FLANN based NN is not needed (graph based NN will still be available), `-DWITH_FLANN=OFF`/`nn=0` can be specified too 
    * if feature tracker is not needed, `-DWITH_FEAT=OFF`/`feat=0` can be specified too 

Setting Parameters:
-------------------
MTF parameters can be specified either in the _cfg_ files present in the _Config_ sub folder or from the command line.
Please refer the ReadMe in the **_Config_** sub folder or [this configuration page](http://webdocs.cs.ualberta.ca/~vis/mtf/params.html) for detailed instructions.

Some **preconfigured cfg files** are provided here to get the system up and running quickly. Config files involving dataset sequences assume that all datasets are located in the `Datasets` sub folder under the active directory. The zip file should be extracted in the `Config` folder to replace the existing cfg files there (if any):

* [RKLT](http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=7158323&url=http%3A%2F%2Fieeexplore.ieee.org%2Fiel7%2F7158225%2F7158304%2F07158323.pdf%3Farnumber%3D7158323) with a 10x10 grid of trackers using [Efficient Second order Minimization](http://far.in.tum.de/pub/benhimane2007ijcv/benhimane2007ijcv.pdf) SM, [Normalized Cross Correlation](http://link.springer.com/chapter/10.1007%2F978-3-642-33783-3_32) AM and 2 DOF (translation) SSM followed by an 8 DOF (homography) tracker with the same SM and AM - this is the current state of the art in 8 DOF tracking to the best of our knowledge
    * [live sequence from USB or Firewire camera](http://webdocs.cs.ualberta.ca/~vis/mtf/rkl_esmlm_ncc_10r_25p_usb.zip)
    * [`nl_cereal_s3` sequence from TMT dataset](http://webdocs.cs.ualberta.ca/~vis/mtf/rkl_esmlm_ncc_10r_25p_tmt_3.zip)
* [Nearest Neighbour](http://www.comp.nus.edu.sg/~haoyu/rss/rss09/p44.html) with 1000 samples followed by [Inverse Compositional](http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=990652&url=http%3A%2F%2Fieeexplore.ieee.org%2Fiel5%2F7768%2F21353%2F00990652.pdf%3Farnumber%3D990652) [Levenberg Marquardt](https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm) SM with [Mutual Information](https://www.irisa.fr/lagadic/pdf/2010_ismar_dame.pdf) AM and 8 DOF (homography) SSM 
    * [live sequence from USB or Firewire camera](http://webdocs.cs.ualberta.ca/~vis/mtf/nniclm_1k_mi_usb.zip)	
* [Particle Filter](http://ieeexplore.ieee.org/document/6589599/?tp=&arnumber=6589599) with 500 particles followed by [Forward Compositional](http://link.springer.com/article/10.1023%2FA%3A1008195814169) [Levenberg Marquardt](https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm) SM with [Structural Similarity](http://link.springer.com/article/10.1007/s00138-007-0107-x) AM and 8 DOF (homography) SSM 
    * [`towel` sequence from LinTrack dataset](http://webdocs.cs.ualberta.ca/~vis/mtf/pffclm_500_ssim_lintrack_2.zip)

Running the demo application:
-----------------------------

Use either `make run` or `runMTF` to start the tracking application using the settings specified as above.

Building the ROS package:
----------------------

A simple ROS package called `mtf_bridge` that demonstrates the integration of MTF with ROS applications is included in the `ROS` sub directory.
Details about building and using this package are in the ReadMe in this folder.

Building a new application that uses MTF:
-----------------------------------------
cmake
-----
The build process generates a `mtfConfig.cmake` file in the build folder with the main defines.
There are two ways to use this file:

* The standard `find_package` command can be used but this requires `MTF_DIR` variable to be set to the MTF build folder while running the cmake command.
This can be done, for instance, as:  
`cmake .. -DMTF_DIR=<path to MTF build folder>`

* The `mtfConfig.cmake` file can be copied into the project tree and included in the _CMakeLists.txt_ file. Then, the defined variables can be used to obtain the header files, linked libraries and compile time definitions. More details can be found in the comments section of issue #11.

An example _CMakeLists.txt_ file for a standalone project that uses either of the above methods is included in _cmake/CMakeLists.txt.example_.

If neither of the above methods work, a manually created `FindMTF.cmake` file is also included in the `cmake/Modules` folder. It has not been widely tested but should work with most standard configurations.

Windows
-------
If MTF needs to be used within an existing VS project that was built without using cmake, the required preprocessor definitions and linked libraries can be [added manually](https://stackoverflow.com/questions/5100283/how-do-i-setup-visual-studio-to-register-some-defines-globally?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa).

* Right click on one of the executables in MTF solution (e.g. `runMTF`), go to `Properties -> C/C++ -> Preprocessor -> Preprocessor Definitions` and copy all flags there to the same location in your own project.

* Repeat this with `Properties -> Linker -> Input -> Additional Dependencies` to add all required libraries for linking.
	
make
----
Use the `make app app=<APPLICATION_NAME>` command as detailed in the make commands section.


**For Developers**
==================

Adding a new Appearance Model (AM) or State Space Model (SSM):
-------------------------------------------------------------

cmake
-----
1. Modify the .cmake files in the respective sub directories:
    - Modify the AM/AM.cmake file to add the name of the AM to set(APPEARANCE_MODELS …)
    - Modify the SSM/SSM.cmake file to add the name of the SSM to set(STATE_SPACE_MODELS …)

2. Modify "mtf.h" to add a new AM/SSM option in the overloaded function `getTrackerObj` (starting lines 498 and 533 respectively) that can be used with your config file to pick the AM/SSM, and create an object with this selected model.
3. Modify "Macros/register.h" to add the new AM/SSM class name under `_REGISTER_TRACKERS_AM/_REGISTER_HTRACKERS_AM` or `_REGISTER_TRACKERS/_REGISTER_TRACKERS_SSM/_REGISTER_HTRACKERS/_REGISTER_HTRACKERS_SSM` respectively

make
----
1. Modify the .mak file in the respective sub directory to:
    - add the name of the AM/SSM to the variable `APPEARANCE_MODELS` or `STATE_SPACE_MODELS` respectively
	- add rule to compile the .o of the new AM/SSM that is dependent on its source and header files - simplest method would be to copy an existing command and change the names of the model.
2. Modify makefile to add any extra dependencies that the new files need to include under variable `FLAGS64`
and extra libraries to link against under `LIB_MTF_LIBS`

All steps are identical for adding a new Search Method (SM) too except the last one which is not needed. Instead this header needs to be included and the appropriate macro needs to be called from its source file to register the SM with all existing AMs and SSMs. Refer to the last 2 lines of the .cc file of any existing SM to see how to do this.



Example: Implementing a minimalistic AM that can be used with Nearest Neighbour SM:
-----------------------------------------------------------------------------------

You need to create a new derived class from AppearanceModel.
Implement the following functions:

1. `initializePixVals/updatePixVals` : takes the current location of the object as a 2xN matrix where N = no. of sampled points and each column contains the x,y coordinates of one point. By default, it is simply supposed to extract the pixel values at these locations from the current image but there is no restriction by design and you are free to compute anything from these pixel values.
2. `initializeDistFeat/updateDistFeat` : these compute a distance feature transform from the current patch.
3. `getDistFeat`: returns a pointer to an array containing the distance feature vector computed by the above function. There is also an overloaded variant of updateDistFeat  that takes a pointer as input and directly writes this feature vector to the pre-allocated array pointed to by this pointer.
4. distance functor (`operator()`): computes a scalar that measures the dissimilarity or distance between two feature vectors (obtained using the previous two functions).

Example: Implementing a minimalistic AM that can be used with Particle Filter SM:
---------------------------------------------------------------------------------

You need to create a new derived class from AppearanceModel.
Implement the following functions:

1. `initializePixVals/updatePixVals` : Same as with NN. No need to overwrite if you just need to extract the pixel values
2. `getLikelihood`: get the likelihood/weights of the particle
3. `initializeSimilarity`: initialize some class variables like the initial features.
4. `updateSimilarity`: update the similarity between current patch and the template 
