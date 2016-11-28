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


#ifndef HOMOGRAPHY_H_
#define HOMOGRAPHY_H_

#include "Typedefs.hpp"
#include <vector>

using namespace std;


namespace Homography
{

void ComputeWarpJacobian(const int inputPixelx, const int inputPixely, const vector<float>& p, Eigen::Matrix<float, 2, N_PARAM>& warpJacobian);
void ComputeWarpedHomoPoints(const cv::Point3f & inputPixel, cv::Point3f & outputPixel, const vector<float> & p);
cv::Matx33f GetMatrix(const vector<float> & p);
cv::Matx33f GetUnscaledMatrix(const vector<float> &p);
vector<float> GetScaledParam(const vector<float> &unscaled_p);

void InverseWarpParameters(const vector<float>& directP, vector<float>& invP);
void InverseWarpParameters(const Eigen::Matrix<float, N_PARAM, 1>& directP, vector<float>& invP);

void CompositionalParametersUpdateWithCheck(vector<float> &p, const vector<float>& deltap);
vector<float> ParametersUpdateCompositional(const vector<float> &p,const vector<float>& deltap);

void DisplayParameters(const vector<float> &parameters);
bool CheckHomography(const vector<float> &parameters);

inline void ComputeWarpedPixels(const int inputPixelx, const int inputPixely, int & outputPixelx, int & outputPixely, const vector<float> & p)
{
	float scalex = (MAX_SCALING - MIN_SCALING)/(PICTURE_WIDTH-1);
	float scaley = (MAX_SCALING - MIN_SCALING)/(PICTURE_HEIGHT-1);
	cv::Matx33f scalingMatrix(scalex, 0, MIN_SCALING, 0, scaley, MIN_SCALING, 0, 0, 1);
	cv::Matx33f homographyMatrix = Homography::GetMatrix(p);
	cv::Matx33f unNormalizedHomography  = scalingMatrix.inv() * homographyMatrix * scalingMatrix;
	vector<float> pUnscaled(8);
	pUnscaled[0] = unNormalizedHomography(0,0)-1;
	pUnscaled[1] = unNormalizedHomography(1,0);
	pUnscaled[2] = unNormalizedHomography(0,1);
	pUnscaled[3] = unNormalizedHomography(1,1)-1;
	pUnscaled[4] = unNormalizedHomography(0,2);
	pUnscaled[5] = unNormalizedHomography(1,2);
	pUnscaled[6] = unNormalizedHomography(2,0);
	pUnscaled[7] = unNormalizedHomography(2,1);


	float denom = 1./(1 + pUnscaled[6]*inputPixelx + pUnscaled[7]*inputPixely);
	outputPixelx = (((1+pUnscaled[0])*inputPixelx + pUnscaled[2]*inputPixely + pUnscaled[4]) * denom + 0.5f);
	outputPixely = ((pUnscaled[1]*inputPixelx + (1+pUnscaled[3])*inputPixely + pUnscaled[5]) * denom + 0.5f);
}

}

#endif /* HOMOGRAPHY_H_ */
