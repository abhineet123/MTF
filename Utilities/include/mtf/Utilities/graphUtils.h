//------------------------------------------------------------------------------
// Graphing functions for OpenCV.	Part of "ImageUtils.cpp", a set of handy utility functions for dealing with images in OpenCV.
// by Shervin Emami (http://www.shervinemami.co.cc/) on 20th May, 2010.
// updated for newer versions of OpenCV while adapting for MTF
//------------------------------------------------------------------------------

#ifndef MTF_GRAPH_UTILS_H
#define MTF_GRAPH_UTILS_H

#include "mtf/Macros/common.h"
#include "opencv2/core/core.hpp"

#define DEFAULT(val) = val

namespace mtf{
	namespace utils{
		//------------------------------------------------------------------------------
		// Graphing functions
		//------------------------------------------------------------------------------

		// Draw the graph of an array of floats into imageDst or a new image, between minV & maxV if given.
		// Remember to free the newly created image if imageDst is not given.
		cv::Mat drawFloatGraph(const float *arraySrc, int nArrayLength, cv::Mat imageDst DEFAULT(cv::Mat()),
			float minV DEFAULT(0.0), float maxV DEFAULT(0.0), int width DEFAULT(0), int height DEFAULT(0),
			char *graphLabel DEFAULT(0), bool showScale DEFAULT(true));

		// Draw the graph of an array of ints into imageDst or a new image, between minV & maxV if given.
		// Remember to free the newly created image if imageDst is not given.
		cv::Mat drawIntGraph(const int *arraySrc, int nArrayLength, cv::Mat imageDst DEFAULT(cv::Mat()),
			int minV DEFAULT(0), int maxV DEFAULT(0), int width DEFAULT(0), int height DEFAULT(0),
			char *graphLabel DEFAULT(0), bool showScale DEFAULT(true));

		// Draw the graph of an array of uchars into imageDst or a new image, between minV & maxV if given.
		// Remember to free the newly created image if imageDst is not given.
		cv::Mat drawUCharGraph(const uchar *arraySrc, int nArrayLength, cv::Mat imageDst DEFAULT(cv::Mat()),
			int minV DEFAULT(0), int maxV DEFAULT(0), int width DEFAULT(0), int height DEFAULT(0),
			char *graphLabel DEFAULT(0), bool showScale DEFAULT(true));

		// Display a graph of the given float array.
		// If background is provided, it will be drawn into, for combining multiple graphs using drawFloatGraph().
		// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
		void showFloatGraph(const char *name, const float *arraySrc, int nArrayLength, int delay_ms DEFAULT(500),
			cv::Mat background DEFAULT(cv::Mat()));

		// Display a graph of the given int array.
		// If background is provided, it will be drawn into, for combining multiple graphs using drawIntGraph().
		// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
		void showIntGraph(const char *name, const int *arraySrc, int nArrayLength, int delay_ms DEFAULT(500),
			cv::Mat background DEFAULT(cv::Mat()));

		// Display a graph of the given unsigned char array.
		// If background is provided, it will be drawn into, for combining multiple graphs using drawUCharGraph().
		// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
		void showUCharGraph(const char *name, const uchar *arraySrc, int nArrayLength, int delay_ms DEFAULT(500),
			cv::Mat background DEFAULT(cv::Mat()));

		// Simple helper function to easily view an image, with an optional pause.
		void showImage(const cv::Mat img, int delay_ms DEFAULT(0), char *name DEFAULT(0));

		// Call 'setGraphColor(0)' to reset the colors that will be used for graphs.
		void setGraphColor(int index DEFAULT(0));
		// Specify the exact color that the next graph should be drawn as.
		void setCustomGraphColor(int R, int B, int G);
	}
}

#endif //end GRAPH_UTILS