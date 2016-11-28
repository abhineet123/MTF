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

#ifndef TESTS_HPP_
#define TESTS_HPP_
#include <iostream>
#include <list>
#include <vector>

#include <algorithm>    // std::max
#include <assert.h>
#include <ctime>
#include <cmath>
#include<stdio.h>
#include <chrono>

#include "HomographyEstimation.hpp"


void RunAllTests();


void ShotSetImageTest();
void ReadDataFromXMLTest();
void ComputeHomographyTest();
void CompositionHomographyTest();
void HomographyJacobianTest2();
void InverseHomographyTest();
void WarpedIntensitiesTest();
void SDImagesTest();
void LucaskanadeSSDTest();
void LucaskanadeDescriptorFieldsTest();

void LucaskanadeVideoSSDSpeedTest();
void LucaskanadeVideoDesciptorFieldsSpeedTest();
void LucaskanadeVideoGradientMagnitudeSpeedTest();

#endif /* TESTS_HPP_ */
