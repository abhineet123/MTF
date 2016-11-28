/*
Copyright 2014 Alberto Crivellaro, Ecole Polytechnique Federale de Lausanne (EPFL), Switzerland.
alberto.crivellaro@epfl.ch

terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
 */

#ifndef TYPEDEFS_HPP_
#define TYPEDEFS_HPP_
#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#ifndef uint
typedef unsigned int uint;
#endif

// TODO make optimization generic wrt type of warp
#define N_PARAM 8
#define MIN_SCALING (0.0) // like thi sYanick can play around ^^
#define MAX_SCALING  (1.0)
#define PICTURE_WIDTH (2.0) // like thi sYanick can play around ^^
#define PICTURE_HEIGHT (2.0)

using namespace std;


typedef enum {intensity, gradientModule, descriptorFields} OptimizationType;

template<typename T = int >
class StructOfArray2d
{
public:
	StructOfArray2d(int size = 0)
{
		x = vector<T>(size);
		y = vector<T>(size);
};
	int size() const
	{
		return x.size();
	}

	void reserve(const int s)
	{
		  x.reserve(s);
		  y.reserve(s);
	}

	void resize(const int s)
	{
		x.resize(s);
		y.resize(s);
	}

	void push_back(cv::Point_<T> p)
	{
		x.push_back(p.x);
		y.push_back(p.y);
	}

	void push_back(T xx, T yy)
	{
		x.push_back(xx);
		y.push_back(yy);
	}

	void clear()
		{
			x.clear();
			y.clear();
		}

	void info() const
	{
		cout<<"size: "<<x.size()<<endl;
		for(int i=0;i<x.size();i++)
			cout<<"("<<x[i]<<","<<y[i]<<")"<<endl;
	}

	const cv::Point_<T> operator[](int n) const
	{
		return cv::Point_<T>(x[n],y[n]) ;
	}

	StructOfArray2d& operator=(const StructOfArray2d &anotherStruct)
	{
			this->x = anotherStruct.x;
			this->y = anotherStruct.y;
			return *this;
	  }

	vector<T> x;
	vector<T> y;
};

typedef StructOfArray2d<int> StructOfArray2di;
typedef StructOfArray2d<float> StructOfArray2df;

template<typename T = int >
class StructOfArray3d
{
	StructOfArray3d<T>(int size)
			{
		x = vector<T>(size);
		y = vector<T>(size);
		z = vector<T>(size);
			};
public:
	vector<T> x;
	vector<T> y;
	vector<T> z;
};


struct AlignmentResults
{
	vector<float> residualNorm;
	vector<vector<float> > poseIntermediateGuess;
	int exitFlag;
	int nIter;
	AlignmentResults(): exitFlag(1e6), nIter(0){};
};


struct OptimizationParameters
{
	float resTol, pTol;
	int maxIter;
	int maxIterSingleLevel;
	vector<float> pyramidSmoothingVariance;
	float presmoothingVariance;
	int nControlPointsOnEdge;
	float borderThicknessHorizontal;
	float borderThicknessVertical;
	bool bAdaptativeChoiceOfPoints;
	bool bNormalizeDescriptors;

	friend ostream &operator<<(ostream &out, OptimizationParameters optParam)     //output
	{
		out<<" ***** optimization parameters ************" << endl;
		out<<"res tol : "<<optParam.resTol<<"\n";
		out<<"param tol : "<<optParam.pTol<<"\n";
		out<<"max iter : "<<optParam.maxIter<<"\n";
		out<<"max iter single level: "<<optParam.maxIterSingleLevel<<"\n";
		out<<"presmoothing variance : "<<optParam.presmoothingVariance<<"\n";
		out<<"pyramid smoothing var : [";
		for (uint i(0); i < optParam.pyramidSmoothingVariance.size(); ++i)
			out<<optParam.pyramidSmoothingVariance[i]<<",  ";
		out<<"]\n";
		out<<"n control points on edge : "<<optParam.nControlPointsOnEdge<<"\n";
		out<<"border thickness horizontal: "<<optParam.borderThicknessHorizontal<<"\n";
		out<<"border thickness vertical: "<<optParam.borderThicknessVertical<<"\n";
		out<<"adaptative choice of points : "<<optParam.bAdaptativeChoiceOfPoints<<"\n";
		out<<"normalization of intensity : "<<optParam.bNormalizeDescriptors<<"\n";

		out<<" *****************" << endl;
		return out;
	}

};

#endif /* TYPEDEFS_HPP_ */
