//------------------------------------------------------------------------------
// Graphing functions for OpenCV.	Part of "ImageUtils.cpp", a set of handy utility functions for dealing with images in OpenCV.
// by Shervin Emami (http://www.shervinemami.co.cc/) on 20th May, 2010.
// updated for newer versions of OpenCV while adapting for MTF
//------------------------------------------------------------------------------

#include <stdio.h>
#include "mtf/Utilities/graphUtils.h"
#include "opencv2/highgui/highgui.hpp"
#if CV_MAJOR_VERSION == 3
#include "opencv2/imgproc/imgproc.hpp"
#endif

#ifndef UCHAR
typedef unsigned char UCHAR;
#endif


#include <iostream>
using namespace std;

#ifndef _WIN32
#define sprintf_s(buffer, buffer_size, stringbuffer, ...) (sprintf(buffer, stringbuffer, __VA_ARGS__))
#endif

namespace mtf {
	namespace utils {
		//------------------------------------------------------------------------------
		// Graphing functions
		//------------------------------------------------------------------------------
		const cv::Scalar BLACK = CV_RGB(0, 0, 0);
		const cv::Scalar WHITE = CV_RGB(255, 255, 255);
		const cv::Scalar GREY = CV_RGB(150, 150, 150);

		int countGraph = 0;	// Used by 'getGraphColor()'
		cv::Scalar customGraphColor;
		int usingCustomGraphColor = 0;

		// Get a new color to draw graphs. Will use the latest custom color, or change between blue, green, red, dark-blue, dark-green and dark-red until a new image is created.
		cv::Scalar getGraphColor(void) {
			if(usingCustomGraphColor) {
				usingCustomGraphColor = 0;
				return customGraphColor;
			}
			countGraph++;
			switch(countGraph) {
			case 1:
				return CV_RGB(60, 60, 255);	// light-blue
			case 2:
				return CV_RGB(60, 255, 60);	// light-green
			case 3:
				return CV_RGB(255, 60, 40);	// light-red
			case 4:
				return CV_RGB(0, 210, 210);	// blue-green
			case 5:
				return CV_RGB(180, 210, 0);	// red-green
			case 6:
				return CV_RGB(210, 0, 180);	// red-blue
			case 7:
				return CV_RGB(0, 0, 185);		// dark-blue
			case 8:
				return CV_RGB(0, 185, 0);		// dark-green
			case 9:
				return CV_RGB(185, 0, 0);		// dark-red
			default:
				countGraph = 0;	// start rotating through colors again.
				return CV_RGB(200, 200, 200);	// grey
			}
		}
		// Call 'setGraphColor()' to reset the colors that will be used for graphs.
		void setGraphColor(int index) {
			countGraph = index;
			usingCustomGraphColor = 0;	// dont use a custom color.
		}
		// Specify the exact color that the next graph should be drawn as.
		void setCustomGraphColor(int R, int B, int G) {
			customGraphColor = CV_RGB(R, G, B);
			usingCustomGraphColor = 1;	// show that it will be used.
		}

		// Draw the graph of an array of floats into imageDst or a new image, between minV & maxV if given.
		// Remember to free the newly created image if imageDst is not given.
		cv::Mat drawFloatGraph(const float *arraySrc, int nArrayLength,
			cv::Mat imageDst, float minV, float maxV, int width, int height,
			char *graphLabel, bool showScale) {
			int w = width;
			int h = height;
			int b = 10;		// border around graph within the image
			if(w <= 20)
				w = nArrayLength + b * 2;	// width of the image
			if(h <= 20)
				h = 220;

			int s = h - b * 2;// size of graph height
			float xscale = 1.0;
			if(nArrayLength > 1)
				xscale = (w - b * 2) / static_cast<float>(nArrayLength - 1);	// horizontal scale
			cv::Mat imageGraph;	// output image

			// Get the desired image to draw into.
			if(imageDst.empty()) {
				// Create an RGB image for graphing the data
				imageGraph.create(cv::Size(w, h), CV_8UC3);
				// Clear the image
				imageGraph = WHITE;
			} else {
				// Draw onto the given image.
				imageGraph = imageDst;
			}
			if(imageGraph.empty()) {
				cerr << "ERROR in drawFloatGraph(): Couldn't create image of " << w << " x " << h << endl;
				exit(1);
			}
			cv::Scalar colorGraph = getGraphColor();	// use a different color each time.

			// If the user didnt supply min & mav values, find them from the data, so we can draw it at full scale.
			if(fabs(minV) < 0.0000001f && fabs(maxV) < 0.0000001f) {
				for(int i = 0; i < nArrayLength; i++) {
					float v = static_cast<float>(arraySrc[i]);
					if(v < minV)
						minV = v;
					if(v > maxV)
						maxV = v;
				}
			}
			float diffV = maxV - minV;
			if(diffV == 0)
				diffV = 0.00000001f;	// Stop a divide-by-zero error
			float fscale = static_cast<float>(s) / diffV;

			// Draw the horizontal & vertical axis
			int y0 = cvRound(minV * fscale);
			line(imageGraph, cv::Point(b, h - (b - y0)), cv::Point(w - b, h - (b - y0)), BLACK);
			line(imageGraph, cv::Point(b, h - (b)), cv::Point(b, h - (b + s)), BLACK);

			// Write the scale of the y axis
			if(showScale) {
				cv::Scalar clr = GREY;
				std::string text = cv_format("%e", maxV);
				cv::putText(imageGraph, text, cv::Point(1, b + 4), cv::FONT_HERSHEY_PLAIN, 1, clr);
				// Write the scale of the x axis
				text = cv_format("%d", (nArrayLength - 1));
				cv::putText(imageGraph, text, cv::Point(w - b + 4 - 10 * text.size(), (h / 2) + 10), 
					cv::FONT_HERSHEY_PLAIN, 1, clr);
			}

			// Draw the values
			CvPoint ptPrev = cv::Point(b, h - (b - y0));	// Start the lines at the 1st point.
			for(int i = 0; i < nArrayLength; i++) {
				int y = cvRound((arraySrc[i] - minV) * fscale);	// Get the values at a bigger scale
				int x = cvRound(i * xscale);
				CvPoint ptNew = cv::Point(b + x, h - (b + y));
				line(imageGraph, ptPrev, ptNew, colorGraph, 1, CV_AA);	// Draw a line from the previous point to the new point
				ptPrev = ptNew;
			}

			// Write the graph label, if desired
			if(graphLabel != NULL && strlen(graphLabel) > 0) {
				//cvInitFont(cv::FONT_HERSHEY_PLAIN, 1,CV_FONT_HERSHEY_PLAIN, 0.5,0.7, 0,1,CV_AA);
				cv::putText(imageGraph, graphLabel, cv::Point(30, 10), cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 0, 0));	// black text
			}

			return imageGraph;
		}

		// Draw the graph of an array of ints into imageDst or a new image, between minV & maxV if given.
		// Remember to free the newly created image if imageDst is not given.
		cv::Mat drawIntGraph(const int *arraySrc, int nArrayLength, cv::Mat imageDst,
			int minV, int maxV, int width, int height, char *graphLabel, bool showScale) {
			int w = width;
			int h = height;
			int b = 10;		// border around graph within the image
			if(w <= 20)
				w = nArrayLength + b * 2;	// width of the image
			if(h <= 20)
				h = 220;

			int s = h - b * 2;// size of graph height
			float xscale = 1.0;
			if(nArrayLength > 1)
				xscale = (w - b * 2) / static_cast<float>(nArrayLength - 1);	// horizontal scale
			cv::Mat imageGraph;	// output image

			// Get the desired image to draw into.
			if(imageDst.empty()) {
				// Create an RGB image for graphing the data
				imageGraph.create(cv::Size(w, h), CV_8UC3);

				// Clear the image
				imageGraph = WHITE;
			} else {
				// Draw onto the given image.
				imageGraph = imageDst;
			}
			if(imageGraph.empty()) {
				cerr << "ERROR in drawIntGraph(): Couldn't create image of " << w << " x " << h << endl;
				exit(1);
			}
			cv::Scalar colorGraph = getGraphColor();	// use a different color each time.

			// If the user didnt supply min & mav values, find them from the data, so we can draw it at full scale.
			if(minV == 0 && maxV == 0) {
				for(int i = 0; i < nArrayLength; i++) {
					int v = arraySrc[i];
					if(v < minV)
						minV = v;
					if(v > maxV)
						maxV = v;
				}
			}
			int diffV = maxV - minV;
			if(diffV == 0)
				diffV = 1;	// Stop a divide-by-zero error
			float fscale = static_cast<float>(s)/ static_cast<float>(diffV);

			// Draw the horizontal & vertical axis
			int y0 = cvRound(minV * fscale);
			line(imageGraph, cv::Point(b, h - (b - y0)), cv::Point(w - b, h - (b - y0)), BLACK);
			line(imageGraph, cv::Point(b, h - (b)), cv::Point(b, h - (b + s)), BLACK);

			// Write the scale of the y axis
			if(showScale) {
				//cvInitFont(cv::FONT_HERSHEY_PLAIN, 1,CV_FONT_HERSHEY_PLAIN,0.5,0.6, 0,1, CV_AA);	// For OpenCV 2.0
				cv::Scalar clr = GREY;
				std::string text = cv_format("%d", maxV);
				cv::putText(imageGraph, text, cv::Point(1, b + 4), cv::FONT_HERSHEY_PLAIN, 1, clr);
				// Write the scale of the x axis
				text = cv_format("%d", nArrayLength - 1);
				cv::putText(imageGraph, text, cv::Point(w - b + 4 - 10 * text.size(), (h / 2) + 10), cv::FONT_HERSHEY_PLAIN, 1, clr);
			}

			// Draw the values
			CvPoint ptPrev = cv::Point(b, h - (b - y0));	// Start the lines at the 1st point.
			for(int i = 0; i < nArrayLength; i++) {
				int y = cvRound((arraySrc[i] - minV) * fscale);	// Get the values at a bigger scale
				int x = cvRound(i * xscale);
				CvPoint ptNew = cv::Point(b + x, h - (b + y));
				line(imageGraph, ptPrev, ptNew, colorGraph, 1, CV_AA);	// Draw a line from the previous point to the new point
				ptPrev = ptNew;
			}

			// Write the graph label, if desired
			if(graphLabel != NULL && strlen(graphLabel) > 0) {
				//cvInitFont(cv::FONT_HERSHEY_PLAIN, 1,CV_FONT_HERSHEY_PLAIN, 0.5,0.7, 0,1,CV_AA);
				cv::putText(imageGraph, graphLabel, cv::Point(30, 10), cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 0, 0));	// black text
			}
			return imageGraph;
		}

		// Draw the graph of an array of uchars into imageDst or a new image, between minV & maxV if given..
		// Remember to free the newly created image if imageDst is not given.
		cv::Mat drawUCharGraph(const uchar *arraySrc, int nArrayLength, 
			cv::Mat imageDst, int minV, int maxV, int width, int height, 
			char *graphLabel, bool showScale) {
			int w = width;
			int h = height;
			int b = 10;		// border around graph within the image
			if(w <= 20)
				w = nArrayLength + b * 2;	// width of the image
			if(h <= 20)
				h = 220;

			int s = h - b * 2;// size of graph height
			float xscale = 1.0;
			if(nArrayLength > 1)
				xscale = (w - b * 2) / static_cast<float>(nArrayLength - 1);	// horizontal scale
			cv::Mat imageGraph;	// output image

			// Get the desired image to draw into.
			if(imageDst.empty()) {
				// Create an RGB image for graphing the data
				imageGraph.create(cv::Size(w, h), CV_8UC3);

				// Clear the image
				imageGraph = WHITE;
			} else {
				// Draw onto the given image.
				imageGraph = imageDst;
			}
			if(imageGraph.empty()) {
				cerr << "ERROR in drawUCharGraph(): Couldn't create image of " << w << " x " << h << endl;
				exit(1);
			}
			cv::Scalar colorGraph = getGraphColor();	// use a different color each time.

			// If the user didnt supply min & mav values, find them from the data, so we can draw it at full scale.
			if(minV == 0 && maxV == 0) {
				for(int i = 0; i < nArrayLength; i++) {
					int v = arraySrc[i];
					if(v < minV)
						minV = v;
					if(v > maxV)
						maxV = v;
				}
			}
			int diffV = maxV - minV;
			if(diffV == 0)
				diffV = 1;	// Stop a divide-by-zero error
			float fscale = static_cast<float>(s) / static_cast<float>(diffV);

			// Draw the horizontal & vertical axis
			int y0 = cvRound(minV * fscale);
			line(imageGraph, cv::Point(b, h - (b - y0)), cv::Point(w - b, h - (b - y0)), BLACK);
			line(imageGraph, cv::Point(b, h - (b)), cv::Point(b, h - (b + s)), BLACK);

			// Write the scale of the y axis
			if(showScale) {
				cv::Scalar clr = GREY;
				std::string text = cv_format("%d", maxV);
				cv::putText(imageGraph, text, cv::Point(1, b + 4), cv::FONT_HERSHEY_PLAIN, 1, clr);
				// Write the scale of the x axis
				text = cv_format("%d", nArrayLength - 1);
				cv::putText(imageGraph, text, cv::Point(w - b + 4 - 5 * text.size(), (h / 2) + 10), cv::FONT_HERSHEY_PLAIN, 1, clr);
			}

			// Draw the values
			CvPoint ptPrev = cv::Point(b, h - (b - y0));	// Start the lines at the 1st point.
			for(int i = 0; i < nArrayLength; i++) {
				int y = cvRound((arraySrc[i] - minV) * fscale);	// Get the values at a bigger scale
				int x = cvRound(i * xscale);
				CvPoint ptNew = cv::Point(b + x, h - (b + y));
				line(imageGraph, ptPrev, ptNew, colorGraph, 1, CV_AA);	// Draw a line from the previous point to the new point
				ptPrev = ptNew;
			}

			// Write the graph label, if desired
			if(graphLabel != NULL && strlen(graphLabel) > 0) {
				//cvInitFont(cv::FONT_HERSHEY_PLAIN, 1,CV_FONT_HERSHEY_PLAIN, 0.5,0.7, 0,1,CV_AA);
				cv::putText(imageGraph, graphLabel, cv::Point(30, 10), cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 0, 0));	// black text
			}

			return imageGraph;
		}

		// Display a graph of the given float array.
		// If background is provided, it will be drawn into, for combining multiple graphs using drawFloatGraph().
		// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
		void showFloatGraph(const char *name, const float *arraySrc, 
			int nArrayLength, int delay_ms, cv::Mat background) {
			// Draw the graph
			cv::Mat imageGraph = drawFloatGraph(arraySrc, nArrayLength, background);

			// Display the graph into a window
			cv::namedWindow(name);
			imshow(name, imageGraph);

			cv::waitKey(10);		// Note that cv::waitKey() is required for the OpenCV window to show!
			cv::waitKey(delay_ms);	// Wait longer to make sure the user has seen the graph
		}

		// Display a graph of the given int array.
		// If background is provided, it will be drawn into, for combining multiple graphs using drawIntGraph().
		// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
		void showIntGraph(const char *name, const int *arraySrc, int nArrayLength, int delay_ms, cv::Mat background) {
			// Draw the graph
			cv::Mat imageGraph = drawIntGraph(arraySrc, nArrayLength, background);

			// Display the graph into a window
			cv::namedWindow(name);
			imshow(name, imageGraph);

			cv::waitKey(10);		// Note that cv::waitKey() is required for the OpenCV window to show!
			cv::waitKey(delay_ms);	// Wait longer to make sure the user has seen the graph
		}

		// Display a graph of the given unsigned char array.
		// If background is provided, it will be drawn into, for combining multiple graphs using drawUCharGraph().
		// Set delay_ms to 0 if you want to wait forever until a keypress, or set it to 1 if you want it to delay just 1 millisecond.
		void showUCharGraph(const char *name, const uchar *arraySrc, 
			int nArrayLength, int delay_ms, cv::Mat background) {
			// Draw the graph
			cv::Mat imageGraph = drawUCharGraph(arraySrc, nArrayLength, background);

			// Display the graph into a window
			cv::namedWindow(name);
			imshow(name, imageGraph);
			cv::waitKey(10);		// Note that cv::waitKey() is required for the OpenCV window to show!
			cv::waitKey(delay_ms);	// Wait longer to make sure the user has seen the graph
		}

		// Simple helper function to easily view an image, with an optional pause.
		void showImage(const cv::Mat img, int delay_ms, char *name) {
			if(!name)
				name = "Image";
			cv::namedWindow(name, CV_WINDOW_AUTOSIZE);
			imshow(name, img);
			cv::waitKey(delay_ms);
		}
	}
}
