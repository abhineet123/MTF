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

#include "mtf/ThirdParty/DFT/Homography.hpp"

void Homography::ComputeWarpedHomoPoints(const cv::Point3f & inputPixel, cv::Point3f & outputPixel, const vector<float> & p)
{
	outputPixel.x =((1+p[0])*inputPixel.x + p[2]*inputPixel.y + p[4]*inputPixel.z);
	outputPixel.y = (p[1]*inputPixel.x + (1+p[3])*inputPixel.y + p[5]*inputPixel.z);
	outputPixel.z = (p[6]*inputPixel.x + p[7]*inputPixel.y + inputPixel.z);
	outputPixel.x /= outputPixel.z;
	outputPixel.y /= outputPixel.z;
	outputPixel.z /= outputPixel.z;
}

cv::Matx33f Homography::GetMatrix(const vector<float>& p)
{
	return cv::Matx33f(1 + p[0], p[2], p[4], p[1], 1 + p[3], p[5], p[6], p[7], 1);
}

cv::Matx33f Homography::GetUnscaledMatrix(const vector<float> &p)
{
	float scalex = (MAX_SCALING - MIN_SCALING)/(PICTURE_WIDTH-1);
	float scaley = (MAX_SCALING - MIN_SCALING)/(PICTURE_HEIGHT-1);
	cv::Matx33f scalingMatrix(scalex, 0, MIN_SCALING, 0, scaley, MIN_SCALING, 0, 0, 1);
	cv::Matx33f homographyMatrix = Homography::GetMatrix(p);
	return scalingMatrix.inv() * homographyMatrix * scalingMatrix;
}

vector<float> Homography::GetScaledParam(const vector<float> &unscaled_p)
{
	float scalex = (MAX_SCALING - MIN_SCALING)/(PICTURE_WIDTH-1);
	float scaley = (MAX_SCALING - MIN_SCALING)/(PICTURE_HEIGHT-1);
	cv::Matx33f scalingMatrix(scalex, 0, MIN_SCALING, 0, scaley, MIN_SCALING, 0, 0, 1);
	cv::Matx33f unscaledHomographyMatrix = Homography::GetMatrix(unscaled_p);
	cv::Matx33f scaledHomographyMatrix = scalingMatrix * unscaledHomographyMatrix * scalingMatrix.inv();

	vector<float> scaled_p(8);
    scaled_p[0] = scaledHomographyMatrix(0,0) - 1;
    scaled_p[2] = scaledHomographyMatrix(0,1);
    scaled_p[4] = scaledHomographyMatrix(0,2);
    scaled_p[1] = scaledHomographyMatrix(1,0);
    scaled_p[3] = scaledHomographyMatrix(1,1) - 1;
    scaled_p[5] = scaledHomographyMatrix(1,2);
    scaled_p[6] = scaledHomographyMatrix(2,0);
    scaled_p[7] = scaledHomographyMatrix(2,1);

	return scaled_p;
}

void Homography::ComputeWarpJacobian(const int inputPixelx, const int inputPixely, 
	const vector<float>& p, Eigen::Matrix<float, 2, N_PARAM> &warpJacobian)
{

	float scalex = (MAX_SCALING - MIN_SCALING)/(PICTURE_WIDTH-1);
	float scaley = (MAX_SCALING - MIN_SCALING)/(PICTURE_HEIGHT-1);

	float xScaled = MIN_SCALING + scalex * inputPixelx;
	float yScaled = MIN_SCALING + scaley * inputPixely;

	float denom = 1./(1 + p[6]*xScaled + p[7]*yScaled);
	// first row
	warpJacobian(0,0) = xScaled*denom;
	warpJacobian(0,1) = 0.;
	warpJacobian(0,2) = yScaled*denom;
	warpJacobian(0,3) = 0.;
	warpJacobian(0,4) = 1*denom;
	warpJacobian(0,5) = 0.;
	warpJacobian(0,6) =-xScaled*((1+p[0])*xScaled + p[2]*yScaled + p[4])*(denom*denom);
	warpJacobian(0,7) = -yScaled*((1+p[0])*xScaled + p[2]*yScaled + p[4])*(denom*denom);
	// second row
	warpJacobian(1,0) = 0.;
	warpJacobian(1,1) = xScaled*denom;
	warpJacobian(1,2) = 0.;
	warpJacobian(1,3) = yScaled*denom;
	warpJacobian(1,4) = 0.;
	warpJacobian(1,5) = 1*denom;
	warpJacobian(1,6) =-xScaled*(p[1]*xScaled +(1+ p[3])*yScaled + p[5])*(denom*denom);
	warpJacobian(1,7) = -yScaled*(p[1]*xScaled +(1+ p[3])*yScaled + p[5])*(denom*denom);

	////
	Eigen::Matrix<float, 2, 2> inverseScaleJacobian;
	inverseScaleJacobian << 1/scalex, 0, 0, 1/scaley;
	warpJacobian = inverseScaleJacobian * warpJacobian;
}

void Homography::InverseWarpParameters(const vector<float>& directP, vector<float>& invP)
{
	float det = (1+directP[0])*((1+directP[3] - directP[5]*directP[7]))
			- directP[2]*(directP[1] - directP[5]*directP[6]) +
			directP[4]*(directP[1]*directP[7]- directP[6]*(1+directP[3]));

	float denom = det * ((1+directP[0]) * (1+directP[3]) - directP[1]*directP[2]);

	invP[0] = (1 + directP[3] - directP[5]*directP[7]) / denom - 1;
	invP[1] =(-directP[1] + directP[5]*directP[6]) /denom;
	invP[2] = (-directP[2] + directP[4]*directP[7])/denom;
	invP[3] = (1 + directP[0] - directP[4]*directP[6])/denom - 1;
	invP[4] = (-directP[4] -directP[3]*directP[4] + directP[2]*directP[5])/denom;
	invP[5] = (-directP[5] -directP[0]*directP[5] + directP[1]*directP[4])/denom;
	invP[6] = (-directP[6] -directP[3]*directP[6] + directP[1]*directP[7])/denom;
	invP[7] = (-directP[7] -directP[0]*directP[7] + directP[2]*directP[6])/denom;

}

void Homography::InverseWarpParameters(const Eigen::Matrix<float, N_PARAM, 1> &directP, vector<float>& invP)
{
	//  TODO shoot me in the face if I leave this as it is!!!
	float det = (1+directP(0,0))*((1+directP(3,0) - directP(5,0)*directP(7,0)))
			- directP(2,0)*(directP(1,0) - directP(5,0)*directP(6,0)) +
			directP(4,0)*(directP(1,0)*directP(7,0)- directP(6,0)*(1+directP(3,0)));

	float denom = det * ((1+directP(0,0)) * (1+directP(3,0)) - directP(1,0)*directP(2,0));

	invP[0] = (1 + directP(3,0) - directP(5,0)*directP(7,0)) / denom - 1;
	invP[1] =(-directP(1,0) + directP(5,0)*directP(6,0)) /denom;
	invP[2] = (-directP(2,0) + directP(4,0)*directP(7,0))/denom;
	invP[3] = (1 + directP(0,0) - directP(4,0)*directP(6,0))/denom - 1;
	invP[4] = (-directP(4,0) -directP(3,0)*directP(4,0) + directP(2,0)*directP(5,0))/denom;
	invP[5] = (-directP(5,0) -directP(0,0)*directP(5,0) + directP(1,0)*directP(4,0))/denom;
	invP[6] = (-directP(6,0) -directP(3,0)*directP(6,0) + directP(1,0)*directP(7,0))/denom;
	invP[7] = (-directP(7,0) -directP(0,0)*directP(7,0) + directP(2,0)*directP(6,0))/denom;

}

vector<float> Homography::ParametersUpdateCompositional(const vector<float> &p,const vector<float>& deltap)
{
	vector<float> newP(8);
	float denom = 1 + deltap[6]*p[4]  + deltap[7] * p[5];

	newP[0] = (deltap[0] + p[0] + deltap[0]*p[0] + deltap[2]*p[1] + deltap[4]*p[6] - deltap[6]*p[4] - deltap[7]*p[5])/denom;
	newP[1] = (deltap[1] + p[1] + deltap[1]*p[0] + deltap[3]*p[1] + deltap[5]*p[6])/denom;
	newP[2] = (deltap[2] + p[2] + deltap[0]*p[2] + deltap[2]*p[3] + deltap[4]*p[7])/denom;
	newP[3] = (deltap[3] + p[3] + deltap[1]*p[2] + deltap[3]*p[3] + deltap[5]*p[7] - deltap[6]*p[4] - deltap[7]*p[5])/denom;
	newP[4] = (deltap[4] + p[4] + deltap[0]*p[4] + deltap[2]*p[5])/denom;
	newP[5] = (deltap[5] + p[5] + deltap[1]*p[4] + deltap[3]*p[5])/denom;
	newP[6] = (deltap[6] + p[6] + deltap[6]*p[0] + deltap[7]*p[1])/denom;
	newP[7] = (deltap[7] + p[7] + deltap[6]*p[2] + deltap[7]*p[3])/denom;
	return newP;
}

void Homography::CompositionalParametersUpdateWithCheck(vector<float> &p,const vector<float>& deltap)
{
  vector<float> newP = Homography::ParametersUpdateCompositional(p, deltap);
  // Check if the result is valid and update
  // TODO re-enable check with less restrictive parameters
  // if (CheckHomography(newP))
    p = newP;
  // else
  //   cout << "ATTENTION! NOT UPDATING PARAMETERS"<<endl;
}

// Check if the Homography is a good one (from BRIEF demo, which is from MVG book)
bool Homography::CheckHomography(const vector<float>& p)
{
//	return true;
//	return Matx33f(1+p[0], p[2], p[4], p[1], 1+p[3], p[5], p[6],p[7], 1);
// 	The homography matrix looks like this
//	[1+p[0], p[2]  , p[4]]
//	[p[1]  , 1+p[3], p[5]]
//	[p[6]  , p[7]  , 1   ]

//	// just to not keep adding the same thing ??
//	float p0_1 = (1.0+p[0]);
//	float p3_1 = (1.0+p[3]);

  const float det = (1.0+p[0]) * (1.0+p[3]) - p[1] * p[2];
  if (det < 0){
    //		cout << "failed condition 1" << endl;
    return false;
  }

  const float N1 = sqrt((1.0+p[0]) * (1.0+p[0]) + p[1] * p[1]);
  if (N1 > 4 || N1 < 0.1){
    //		cout << "failed condition 2" << endl;
    return false;
  }

  const float N2 = sqrt(p[2] * p[2] + (1.0+p[3]) * (1.0+p[3]));
  if (N2 > 4 || N2 < 0.1){
    //		cout << "failed condition 3" << endl;
    return false;
  }

  const float N3 = sqrt(p[6] * p[6] + p[7] * p[7]);
  if (N3 > 0.002){
    //		cout << "failed condition 4 - N3 = " << N3 << "p6=" << p[6]<< "p7=" << p[7]<< endl;
    return false;
  }

  //	cout << "succeeded!" << endl;
  return true;
}

void Homography::DisplayParameters(const vector<float>& parameters)
{
	cout<<" homography: ";
	for(uint i(0); i< parameters.size();++i)
		cout<< parameters[i]<< "  ";
	cout<<endl;
}





