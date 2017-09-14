/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
 // Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#ifdef _WIN32
#pragma warning(disable:4244)
#endif

#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>

#include "mtf/ThirdParty/MIL/cv_onlineboosting.h"

#define sign(x) (((x)>=0)? 1.0f : -1.0f)

/****************************************************************************************
 COPYRIGHT NOTICE
 ----------------

 The code has been derived from: http://www.vision.ee.ethz.ch/boostingTrackers/
 By Helmut Grabner, Van Gool Luc, Severin Stalder, Michael Grabner, Christian Leistner
 The agreement to put it in OpenCV under the BSD license is at the end of hat file

 ****************************************************************************************/

namespace cv
{
  namespace boosting
  {
    cv::Rect
    RectMultiply(const cv::Rect & rect, float f)
    {
      cv::Rect r_tmp;
      r_tmp.y = (int) (rect.y - ((float) rect.height * f - rect.height) / 2);
      if (r_tmp.y < 0)
        r_tmp.y = 0;
      r_tmp.x = (int) (rect.x - ((float) rect.width * f - rect.width) / 2);
      if (r_tmp.x < 0)
        r_tmp.x = 0;
      r_tmp.height = (int) (rect.height * f);
      r_tmp.width = (int) (rect.width * f);

      return r_tmp;
    }

    bool
    RectIsDetection(const cv::Rect & rect, const cv::Rect & eval, unsigned char *labeledImg, int imgWidth)
    {
      bool isDet = false;
      unsigned char labelEval;
      unsigned char labelDet;

      labelEval = labeledImg[(eval.y) * imgWidth + eval.x];
      labelDet = labeledImg[(rect.y) * imgWidth + rect.x];

      if ((labelEval == labelDet) && (labelDet != 0))
      {
        isDet = true;
      }
      else
      {
        isDet = false;
      }

      return isDet;
    }

    ImageRepresentation::ImageRepresentation(const cv::Mat & image, Size imageSize)
        :
          m_useVariance(false)
    {
      // call the default initialization
      this->defaultInit(image, imageSize);
      return;
    }

    ImageRepresentation::ImageRepresentation(const cv::Mat & image, Size imageSize, Rect imageROI)
        :
          m_useVariance(false)
    {
      this->m_imageSize = imageSize;

      m_ROI = imageROI;
      m_offset = cv::Point2i(m_ROI.x, m_ROI.y);

      if (!image.empty())
        this->createIntegralsOfROI(image);
    }

    void
    ImageRepresentation::defaultInit(const cv::Mat & image, Size imageSize)
    {
      this->m_imageSize = imageSize;

      m_useVariance = false;

      m_ROI.height = imageSize.height;
      m_ROI.width = imageSize.width;
      m_ROI.y = 0;
      m_ROI.x = 0;
      m_offset = cv::Point2i(m_ROI.x, m_ROI.y);

      if (!image.empty())
        this->createIntegralsOfROI(image);

      return;
    }

    void
    ImageRepresentation::setNewROI(Rect ROI)
    {
      this->m_ROI = ROI;
      m_offset = cv::Point2i(ROI.x, ROI.y);
      return;
    }

    void
    ImageRepresentation::setNewImageSize(Rect ROI)
    {
      this->m_imageSize = cv::Size(ROI.width, ROI.height);
    }

    void
    ImageRepresentation::setNewImageAndROI(const cv::Mat & image, Rect ROI)
    {
      this->setNewROI(ROI);
      this->createIntegralsOfROI(image);
    }

    unsigned int
    ImageRepresentation::getValue(cv::Point2i imagePosition)
    {
      cv::Point2i position = imagePosition - m_offset;
      return intImage(position.y, position.x);
    }

    long
    ImageRepresentation::getSqSum(Rect imageROI)
    {
      // left upper Origin
      int OriginX = imageROI.x - m_offset.x;
      int OriginY = imageROI.y - m_offset.y;

      // Check and fix width and height
      int Width = imageROI.width;
      int Height = imageROI.height;

      if (OriginX + Width >= m_ROI.width)
        Width = m_ROI.width - OriginX;
      if (OriginY + Height >= m_ROI.height)
        Height = m_ROI.height - OriginY;

      long int value = intSqImage(OriginY + Height, OriginX + Width) + intSqImage(OriginY, OriginX)
          - intSqImage(OriginY, OriginX + Width) - intSqImage(OriginY + Height, OriginX);

      assert(value >= 0);

      return (long) value;

    }

    float
    ImageRepresentation::getVariance(Rect imageROI)
    {
      double area = imageROI.height * imageROI.width;
      double mean = (double) getSum(imageROI) / area;
      double sqSum = (double) getSqSum(imageROI);

      double variance = sqSum / area - (mean * mean);

      if (variance >= 0.)
        return (float) sqrt(variance);
      else
        return 1.0f;
    }

    int
    ImageRepresentation::getSum(Rect imageROI)
    {
      // left upper Origin
      int OriginX = imageROI.x - m_offset.x;
      int OriginY = imageROI.y - m_offset.y;

      // Check and fix width and height
      int Width = imageROI.width;
      int Height = imageROI.height;

      if (OriginX + Width >= m_ROI.width)
        Width = m_ROI.width - OriginX;
      if (OriginY + Height >= m_ROI.height)
        Height = m_ROI.height - OriginY;

      int value = intImage(OriginY + Height, OriginX + Width) + intImage(OriginY, OriginX)
          - intImage(OriginY, OriginX + Width) - intImage(OriginY + Height, OriginX);

      return value;
    }

    float
    ImageRepresentation::getMean(Rect imageROI)
    {
      // left upper Origin
      int OriginX = imageROI.x - m_offset.x;
      int OriginY = imageROI.y - m_offset.y;

      // Check and fix width and height
      int Width = imageROI.width;
      int Height = imageROI.height;

      if (OriginX + Width >= m_ROI.width)
        Width = m_ROI.width - OriginX;
      if (OriginY + Height >= m_ROI.height)
        Height = m_ROI.height - OriginY;

      return getSum(imageROI) / static_cast<float>(Width * Height);
    }

    void
    ImageRepresentation::createIntegralsOfROI(const cv::Mat & image)
    {
      cv::integral(image(m_ROI), intImage, intSqImage, CV_32S);
    }

    Patches::Patches(void)
        :
          numPatchesX(0),
          numPatchesY(0)
    {
      this->num = 1;
      ROI.height = 0;
      ROI.width = 0;
      ROI.y = 0;
      ROI.x = 0;
    }

    Patches::Patches(int num)
        :
          numPatchesX(0),
          numPatchesY(0)
    {
      this->num = num;
      patches.resize(num);
      ROI.height = 0;
      ROI.width = 0;
      ROI.y = 0;
      ROI.x = 0;
    }

    Rect
    Patches::getRect(int index)
    {
      if (index >= num)
        return Rect(-1, -1, -1, -1);
      if (index < 0)
        return Rect(-1, -1, -1, -1);

      return patches[index];
    }

    int
    Patches::checkOverlap(Rect rect)
    {
      //loop over all patches and return the first found overap
      for (int curPatch = 0; curPatch < num; curPatch++)
      {
        Rect curRect = getRect(curPatch);
        int overlap = (curRect & rect).area();
        if (overlap > 0)
          return overlap;
      }
      return 0;
    }

    bool
    Patches::isDetection(Rect eval, unsigned char *labeledImg, int imgWidth)
    {
      bool isDet = false;
      Rect curRect;

      for (int curPatch = 0; curPatch < num; curPatch++)
      {
        curRect = getRect(curPatch);
        isDet = RectIsDetection(curRect, eval, labeledImg, imgWidth);

        if (isDet)
        {
          break;
        }
      }

      return isDet;
    }

    Rect
    Patches::getSpecialRect(const char* what)
    {
      Rect r;
      r.height = -1;
      r.width = -1;
      r.y = -1;
      r.x = -1;
      return r;
    }

    Rect
    Patches::getSpecialRect(const char* what, Size patchSize)
    {
      Rect r;
      r.height = -1;
      r.width = -1;
      r.y = -1;
      r.x = -1;
      return r;
    }

    Rect
    Patches::getROI()
    {
      return ROI;
    }

    void
    Patches::setCheckedROI(Rect imageROI, Rect validROI)
    {
      int dCol, dRow;
      dCol = imageROI.x - validROI.x;
      dRow = imageROI.y - validROI.y;
      ROI.y = (dRow < 0) ? validROI.y : imageROI.y;
      ROI.x = (dCol < 0) ? validROI.x : imageROI.x;
      dCol = imageROI.x + imageROI.width - (validROI.x + validROI.width);
      dRow = imageROI.y + imageROI.height - (validROI.y + validROI.height);
      ROI.height = (dRow > 0) ? validROI.height + validROI.y - ROI.y : imageROI.height + imageROI.y - ROI.y;
      ROI.width = (dCol > 0) ? validROI.width + validROI.x - ROI.x : imageROI.width + imageROI.x - ROI.x;
    }

    //-----------------------------------------------------------------------------
    PatchesRegularScan::PatchesRegularScan(Rect imageROI, Size patchSize, float relOverlap)
    {
      calculatePatches(imageROI, imageROI, patchSize, relOverlap);
    }

    PatchesRegularScan::PatchesRegularScan(Rect imageROI, Rect validROI, Size patchSize, float relOverlap)
    {
      calculatePatches(imageROI, validROI, patchSize, relOverlap);
    }

    void
    PatchesRegularScan::calculatePatches(Rect imageROI, Rect validROI, Size patchSize, float relOverlap)
    {
      if ((validROI == imageROI))
        ROI = imageROI;
      else
        setCheckedROI(imageROI, validROI);

      int stepCol = (int) floor((1.0f - relOverlap) * (float) patchSize.width + 0.5f);
      int stepRow = (int) floor((1.0f - relOverlap) * (float) patchSize.height + 0.5f);
      if (stepCol <= 0)
        stepCol = 1;
      if (stepRow <= 0)
        stepRow = 1;

      m_patchGrid.height = ((int) ((float) (ROI.height - patchSize.height) / stepRow) + 1);
      m_patchGrid.width = ((int) ((float) (ROI.width - patchSize.width) / stepCol) + 1);

      num = m_patchGrid.width * m_patchGrid.height;
      patches.resize(num);
      int curPatch = 0;

      m_rectUpperLeft = m_rectUpperRight = m_rectLowerLeft = m_rectLowerRight = cv::Rect(0, 0, patchSize.width,
                                                                                         patchSize.height);
      m_rectUpperLeft.y = ROI.y;
      m_rectUpperLeft.x = ROI.x;
      m_rectUpperRight.y = ROI.y;
      m_rectUpperRight.x = ROI.x + ROI.width - patchSize.width;
      m_rectLowerLeft.y = ROI.y + ROI.height - patchSize.height;
      m_rectLowerLeft.x = ROI.x;
      m_rectLowerRight.y = ROI.y + ROI.height - patchSize.height;
      m_rectLowerRight.x = ROI.x + ROI.width - patchSize.width;

      numPatchesX = 0;
      numPatchesY = 0;
      for (int curRow = 0; curRow < ROI.height - patchSize.height + 1; curRow += stepRow)
      {
        numPatchesY++;

        for (int curCol = 0; curCol < ROI.width - patchSize.width + 1; curCol += stepCol)
        {
          if (curRow == 0)
            numPatchesX++;

          patches[curPatch].width = patchSize.width;
          patches[curPatch].height = patchSize.height;
          patches[curPatch].y = curRow + ROI.y;
          patches[curPatch].x = curCol + ROI.x;
          curPatch++;
        }
      }

      assert(curPatch==num);
    }

    PatchesRegularScan::~PatchesRegularScan(void)
    {
    }

    Rect
    PatchesRegularScan::getSpecialRect(const char* what, Size patchSize)
    {
      Rect r;
      r.height = -1;
      r.width = -1;
      r.y = -1;
      r.x = -1;
      return r;
    }

    Rect
    PatchesRegularScan::getSpecialRect(const char* what)
    {
      if (strcmp(what, "UpperLeft") == 0)
        return m_rectUpperLeft;
      if (strcmp(what, "UpperRight") == 0)
        return m_rectUpperRight;
      if (strcmp(what, "LowerLeft") == 0)
        return m_rectLowerLeft;
      if (strcmp(what, "LowerRight") == 0)
        return m_rectLowerRight;
      if (strcmp(what, "Random") == 0)
      {
        int index = (rand() % (num));
        return patches[index];
      }

      // assert (false);
      return Rect(-1, -1, -1, -1); // fixed
    }

    //-----------------------------------------------------------------------------
    PatchesRegularScaleScan::PatchesRegularScaleScan(Rect imageROI, Size patchSize, float relOverlap, float scaleStart,
                                                     float scaleEnd, float scaleFactor)
    {
      calculatePatches(imageROI, imageROI, patchSize, relOverlap, scaleStart, scaleEnd, scaleFactor);
    }

    PatchesRegularScaleScan::PatchesRegularScaleScan(Rect imageROI, Rect validROI, Size patchSize, float relOverlap,
                                                     float scaleStart, float scaleEnd, float scaleFactor)
    {
      calculatePatches(imageROI, validROI, patchSize, relOverlap, scaleStart, scaleEnd, scaleFactor);
    }

    PatchesRegularScaleScan::~PatchesRegularScaleScan(void)
    {

    }

    void
    PatchesRegularScaleScan::calculatePatches(Rect imageROI, Rect validROI, cv::Size patchSize, float relOverlap,
                                              float scaleStart, float scaleEnd, float scaleFactor)
    {

      if ((validROI == imageROI))
        ROI = imageROI;
      else
        setCheckedROI(imageROI, validROI);

      int numScales = (int) (log(scaleEnd / scaleStart) / log(scaleFactor));
      if (numScales < 0)
        numScales = 0;
      float curScaleFactor = 1;
      Size curPatchSize;
      int stepCol, stepRow;

      num = 0;
      for (int curScale = 0; curScale <= numScales; curScale++)
      {
        curPatchSize = cv::Size(patchSize.width * (scaleStart * curScaleFactor),
                                patchSize.height * (scaleStart * curScaleFactor));
        if (curPatchSize.height > ROI.height || curPatchSize.width > ROI.width)
        {
          numScales = curScale - 1;
          break;
        }
        curScaleFactor *= scaleFactor;

        stepCol = (int) floor((1.0f - relOverlap) * (float) curPatchSize.width + 0.5f);
        stepRow = (int) floor((1.0f - relOverlap) * (float) curPatchSize.height + 0.5f);
        if (stepCol <= 0)
          stepCol = 1;
        if (stepRow <= 0)
          stepRow = 1;

        num += ((int) ((float) (ROI.width - curPatchSize.width) / stepCol) + 1)
            * ((int) ((float) (ROI.height - curPatchSize.height) / stepRow) + 1);
      }
      patches.resize(num);

      int curPatch = 0;
      curScaleFactor = 1;
      for (int curScale = 0; curScale <= numScales; curScale++)
      {
        curPatchSize = cv::Size(patchSize.width * (scaleStart * curScaleFactor),
                                patchSize.height * (scaleStart * curScaleFactor));
        curScaleFactor *= scaleFactor;

        stepCol = (int) floor((1.0f - relOverlap) * (float) curPatchSize.width + 0.5f);
        stepRow = (int) floor((1.0f - relOverlap) * (float) curPatchSize.height + 0.5f);
        if (stepCol <= 0)
          stepCol = 1;
        if (stepRow <= 0)
          stepRow = 1;

        for (int curRow = 0; curRow < ROI.height - curPatchSize.height + 1; curRow += stepRow)
        {
          for (int curCol = 0; curCol < ROI.width - curPatchSize.width + 1; curCol += stepCol)
          {
            patches[curPatch].width = curPatchSize.width;
            patches[curPatch].height = curPatchSize.height;
            patches[curPatch].y = curRow + ROI.y;
            patches[curPatch].x = curCol + ROI.x;

            curPatch++;
          }
        }
      }
      assert(curPatch==num);

    }

    Rect
    PatchesRegularScaleScan::getSpecialRect(const char* what)
    {

      if (strcmp(what, "Random") == 0)
      {
        int index = (rand() % (num));
        return patches[index];
      }

      Rect r;
      r.height = -1;
      r.width = -1;
      r.y = -1;
      r.x = -1;
      return r;
    }
    Rect
    PatchesRegularScaleScan::getSpecialRect(const char* what, Size patchSize)
    {
      if (strcmp(what, "UpperLeft") == 0)
      {
        return cv::Rect(ROI.x, ROI.y, patchSize.width, patchSize.height);
      }
      if (strcmp(what, "UpperRight") == 0)
      {
        return cv::Rect(ROI.x + ROI.width - patchSize.width, ROI.y, patchSize.width, patchSize.height);
      }
      if (strcmp(what, "LowerLeft") == 0)
      {
        return cv::Rect(ROI.x, ROI.y + ROI.height - patchSize.height, patchSize.width, patchSize.height);
      }
      if (strcmp(what, "LowerRight") == 0)
      {
        return cv::Rect(ROI.x + ROI.width - patchSize.width, ROI.y + ROI.height - patchSize.height, patchSize.width,
                        patchSize.height);
      }
      if (strcmp(what, "Random") == 0)
      {
        int index = (rand() % (num));
        return patches[index];
      }

      return Rect(-1, -1, -1, -1);
    }

    EstimatedGaussDistribution::EstimatedGaussDistribution()
    {
      m_mean = 0;
      m_sigma = 1;
      this->m_P_mean = 1000;
      this->m_R_mean = 0.01f;
      this->m_P_sigma = 1000;
      this->m_R_sigma = 0.01f;
    }

    EstimatedGaussDistribution::EstimatedGaussDistribution(float P_mean, float R_mean, float P_sigma, float R_sigma)
    {
      m_mean = 0;
      m_sigma = 1;
      this->m_P_mean = P_mean;
      this->m_R_mean = R_mean;
      this->m_P_sigma = P_sigma;
      this->m_R_sigma = R_sigma;
    }

    EstimatedGaussDistribution::~EstimatedGaussDistribution()
    {
    }

    void
    EstimatedGaussDistribution::update(float value)
    {
      //update distribution (mean and sigma) using a kalman filter for each

      float K;
      float minFactor = 0.001f;

      //mean

      K = m_P_mean / (m_P_mean + m_R_mean);
      if (K < minFactor)
        K = minFactor;

      m_mean = K * value + (1.0f - K) * m_mean;
      m_P_mean = m_P_mean * m_R_mean / (m_P_mean + m_R_mean);

      K = m_P_sigma / (m_P_sigma + m_R_sigma);
      if (K < minFactor)
        K = minFactor;

      float tmp_sigma = K * (m_mean - value) * (m_mean - value) + (1.0f - K) * m_sigma * m_sigma;
      m_P_sigma = m_P_sigma * m_R_mean / (m_P_sigma + m_R_sigma);

      m_sigma = static_cast<float>(sqrt(tmp_sigma));
      if (m_sigma <= 1.0f)
        m_sigma = 1.0f;

    }

    void
    EstimatedGaussDistribution::setValues(float mean, float sigma)
    {
      this->m_mean = mean;
      this->m_sigma = sigma;
    }

#define SQROOTHALF 0.7071
#define INITSIGMA( numAreas ) ( static_cast<float>( sqrt( 256.0f*256.0f / 12.0f * (numAreas) ) ) );

    FeatureHaar::FeatureHaar(Size patchSize)
    {
      try
      {
        generateRandomFeature(patchSize);
      } catch (...)
      {
        throw;
      }
    }

    void
    FeatureHaar::generateRandomFeature(Size patchSize)
    {
      cv::Point2i position;
      Size baseDim;
      Size sizeFactor;
      int area;

      Size minSize = Size(3, 3);
      int minArea = 9;

      bool valid = false;
      while (!valid)
      {
        //chosse position and scale
        position.y = rand() % (patchSize.height);
        position.x = rand() % (patchSize.width);

        baseDim.width = (int) ((1 - sqrt(1 - (float) rand() / RAND_MAX)) * patchSize.width);
        baseDim.height = (int) ((1 - sqrt(1 - (float) rand() / RAND_MAX)) * patchSize.height);

        //select types
        //float probType[11] = {0.0909f, 0.0909f, 0.0909f, 0.0909f, 0.0909f, 0.0909f, 0.0909f, 0.0909f, 0.0909f, 0.0909f, 0.0950f};
        float probType[11] =
        { 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        float prob = (float) rand() / RAND_MAX;

        if (prob < probType[0])
        {
          //check if feature is valid
          sizeFactor.height = 2;
          sizeFactor.width = 1;
          if (position.y + baseDim.height * sizeFactor.height >= patchSize.height || position.x
              + baseDim.width * sizeFactor.width
                                                                                     >= patchSize.width)
            continue;
          area = baseDim.height * sizeFactor.height * baseDim.width * sizeFactor.width;
          if (area < minArea)
            continue;

          m_type = 1;
          m_numAreas = 2;
          m_weights.resize(m_numAreas);
          m_weights[0] = 1;
          m_weights[1] = -1;
          m_areas.resize(m_numAreas);
          m_areas[0].x = position.x;
          m_areas[0].y = position.y;
          m_areas[0].height = baseDim.height;
          m_areas[0].width = baseDim.width;
          m_areas[1].x = position.x;
          m_areas[1].y = position.y + baseDim.height;
          m_areas[1].height = baseDim.height;
          m_areas[1].width = baseDim.width;
          m_initMean = 0;
          m_initSigma = INITSIGMA( m_numAreas );

          valid = true;

        }
        else if (prob < probType[0] + probType[1])
        {
          //check if feature is valid
          sizeFactor.height = 1;
          sizeFactor.width = 2;
          if (position.y + baseDim.height * sizeFactor.height >= patchSize.height || position.x
              + baseDim.width * sizeFactor.width
                                                                                     >= patchSize.width)
            continue;
          area = baseDim.height * sizeFactor.height * baseDim.width * sizeFactor.width;
          if (area < minArea)
            continue;

          m_type = 2;
          m_numAreas = 2;
          m_weights.resize(m_numAreas);
          m_weights[0] = 1;
          m_weights[1] = -1;
          m_areas.resize(m_numAreas);
          m_areas[0].x = position.x;
          m_areas[0].y = position.y;
          m_areas[0].height = baseDim.height;
          m_areas[0].width = baseDim.width;
          m_areas[1].x = position.x + baseDim.width;
          m_areas[1].y = position.y;
          m_areas[1].height = baseDim.height;
          m_areas[1].width = baseDim.width;
          m_initMean = 0;
          m_initSigma = INITSIGMA( m_numAreas );
          valid = true;

        }
        else if (prob < probType[0] + probType[1] + probType[2])
        {
          //check if feature is valid
          sizeFactor.height = 4;
          sizeFactor.width = 1;
          if (position.y + baseDim.height * sizeFactor.height >= patchSize.height || position.x
              + baseDim.width * sizeFactor.width
                                                                                     >= patchSize.width)
            continue;
          area = baseDim.height * sizeFactor.height * baseDim.width * sizeFactor.width;
          if (area < minArea)
            continue;

          m_type = 3;
          m_numAreas = 3;
          m_weights.resize(m_numAreas);
          m_weights[0] = 1;
          m_weights[1] = -2;
          m_weights[2] = 1;
          m_areas.resize(m_numAreas);
          m_areas[0].x = position.x;
          m_areas[0].y = position.y;
          m_areas[0].height = baseDim.height;
          m_areas[0].width = baseDim.width;
          m_areas[1].x = position.x;
          m_areas[1].y = position.y + baseDim.height;
          m_areas[1].height = 2 * baseDim.height;
          m_areas[1].width = baseDim.width;
          m_areas[2].y = position.y + 3 * baseDim.height;
          m_areas[2].x = position.x;
          m_areas[2].height = baseDim.height;
          m_areas[2].width = baseDim.width;
          m_initMean = 0;
          m_initSigma = INITSIGMA( m_numAreas );
          valid = true;
        }
        else if (prob < probType[0] + probType[1] + probType[2] + probType[3])
        {
          //check if feature is valid
          sizeFactor.height = 1;
          sizeFactor.width = 4;
          if (position.y + baseDim.height * sizeFactor.height >= patchSize.height || position.x
              + baseDim.width * sizeFactor.width
                                                                                     >= patchSize.width)
            continue;
          area = baseDim.height * sizeFactor.height * baseDim.width * sizeFactor.width;
          if (area < minArea)
            continue;

          m_type = 3;
          m_numAreas = 3;
          m_weights.resize(m_numAreas);
          m_weights[0] = 1;
          m_weights[1] = -2;
          m_weights[2] = 1;
          m_areas.resize(m_numAreas);
          m_areas[0].x = position.x;
          m_areas[0].y = position.y;
          m_areas[0].height = baseDim.height;
          m_areas[0].width = baseDim.width;
          m_areas[1].x = position.x + baseDim.width;
          m_areas[1].y = position.y;
          m_areas[1].height = baseDim.height;
          m_areas[1].width = 2 * baseDim.width;
          m_areas[2].y = position.y;
          m_areas[2].x = position.x + 3 * baseDim.width;
          m_areas[2].height = baseDim.height;
          m_areas[2].width = baseDim.width;
          m_initMean = 0;
          m_initSigma = INITSIGMA( m_numAreas );
          valid = true;
        }
        else if (prob < probType[0] + probType[1] + probType[2] + probType[3] + probType[4])
        {
          //check if feature is valid
          sizeFactor.height = 2;
          sizeFactor.width = 2;
          if (position.y + baseDim.height * sizeFactor.height >= patchSize.height || position.x
              + baseDim.width * sizeFactor.width
                                                                                     >= patchSize.width)
            continue;
          area = baseDim.height * sizeFactor.height * baseDim.width * sizeFactor.width;
          if (area < minArea)
            continue;

          m_type = 5;
          m_numAreas = 4;
          m_weights.resize(m_numAreas);
          m_weights[0] = 1;
          m_weights[1] = -1;
          m_weights[2] = -1;
          m_weights[3] = 1;
          m_areas.resize(m_numAreas);
          m_areas[0].x = position.x;
          m_areas[0].y = position.y;
          m_areas[0].height = baseDim.height;
          m_areas[0].width = baseDim.width;
          m_areas[1].x = position.x + baseDim.width;
          m_areas[1].y = position.y;
          m_areas[1].height = baseDim.height;
          m_areas[1].width = baseDim.width;
          m_areas[2].y = position.y + baseDim.height;
          m_areas[2].x = position.x;
          m_areas[2].height = baseDim.height;
          m_areas[2].width = baseDim.width;
          m_areas[3].y = position.y + baseDim.height;
          m_areas[3].x = position.x + baseDim.width;
          m_areas[3].height = baseDim.height;
          m_areas[3].width = baseDim.width;
          m_initMean = 0;
          m_initSigma = INITSIGMA( m_numAreas );
          valid = true;
        }
        else if (prob < probType[0] + probType[1] + probType[2] + probType[3] + probType[4] + probType[5])
        {
          //check if feature is valid
          sizeFactor.height = 3;
          sizeFactor.width = 3;
          if (position.y + baseDim.height * sizeFactor.height >= patchSize.height || position.x
              + baseDim.width * sizeFactor.width
                                                                                     >= patchSize.width)
            continue;
          area = baseDim.height * sizeFactor.height * baseDim.width * sizeFactor.width;
          if (area < minArea)
            continue;

          m_type = 6;
          m_numAreas = 2;
          m_weights.resize(m_numAreas);
          m_weights[0] = 1;
          m_weights[1] = -9;
          m_areas.resize(m_numAreas);
          m_areas[0].x = position.x;
          m_areas[0].y = position.y;
          m_areas[0].height = 3 * baseDim.height;
          m_areas[0].width = 3 * baseDim.width;
          m_areas[1].x = position.x + baseDim.width;
          m_areas[1].y = position.y + baseDim.height;
          m_areas[1].height = baseDim.height;
          m_areas[1].width = baseDim.width;
          m_initMean = -8 * 128;
          m_initSigma = INITSIGMA( m_numAreas );
          valid = true;
        }
        else if (prob < probType[0] + probType[1] + probType[2] + probType[3] + probType[4] + probType[5] + probType[6])
        {
          //check if feature is valid
          sizeFactor.height = 3;
          sizeFactor.width = 1;
          if (position.y + baseDim.height * sizeFactor.height >= patchSize.height || position.x
              + baseDim.width * sizeFactor.width
                                                                                     >= patchSize.width)
            continue;
          area = baseDim.height * sizeFactor.height * baseDim.width * sizeFactor.width;
          if (area < minArea)
            continue;

          m_type = 7;
          m_numAreas = 3;
          m_weights.resize(m_numAreas);
          m_weights[0] = 1;
          m_weights[1] = -2;
          m_weights[2] = 1;
          m_areas.resize(m_numAreas);
          m_areas[0].x = position.x;
          m_areas[0].y = position.y;
          m_areas[0].height = baseDim.height;
          m_areas[0].width = baseDim.width;
          m_areas[1].x = position.x;
          m_areas[1].y = position.y + baseDim.height;
          m_areas[1].height = baseDim.height;
          m_areas[1].width = baseDim.width;
          m_areas[2].y = position.y + baseDim.height * 2;
          m_areas[2].x = position.x;
          m_areas[2].height = baseDim.height;
          m_areas[2].width = baseDim.width;
          m_initMean = 0;
          m_initSigma = INITSIGMA( m_numAreas );
          valid = true;
        }
        else if (prob
            < probType[0] + probType[1] + probType[2] + probType[3] + probType[4] + probType[5] + probType[6]
              + probType[7])
        {
          //check if feature is valid
          sizeFactor.height = 1;
          sizeFactor.width = 3;
          if (position.y + baseDim.height * sizeFactor.height >= patchSize.height || position.x
              + baseDim.width * sizeFactor.width
                                                                                     >= patchSize.width)
            continue;

          area = baseDim.height * sizeFactor.height * baseDim.width * sizeFactor.width;

          if (area < minArea)
            continue;

          m_type = 8;
          m_numAreas = 3;
          m_weights.resize(m_numAreas);
          m_weights[0] = 1;
          m_weights[1] = -2;
          m_weights[2] = 1;
          m_areas.resize(m_numAreas);
          m_areas[0].x = position.x;
          m_areas[0].y = position.y;
          m_areas[0].height = baseDim.height;
          m_areas[0].width = baseDim.width;
          m_areas[1].x = position.x + baseDim.width;
          m_areas[1].y = position.y;
          m_areas[1].height = baseDim.height;
          m_areas[1].width = baseDim.width;
          m_areas[2].y = position.y;
          m_areas[2].x = position.x + 2 * baseDim.width;
          m_areas[2].height = baseDim.height;
          m_areas[2].width = baseDim.width;
          m_initMean = 0;
          m_initSigma = INITSIGMA( m_numAreas );
          valid = true;
        }
        else if (prob
            < probType[0] + probType[1] + probType[2] + probType[3] + probType[4] + probType[5] + probType[6]
              + probType[7] + probType[8])
        {
          //check if feature is valid
          sizeFactor.height = 3;
          sizeFactor.width = 3;
          if (position.y + baseDim.height * sizeFactor.height >= patchSize.height || position.x
              + baseDim.width * sizeFactor.width
                                                                                     >= patchSize.width)
            continue;
          area = baseDim.height * sizeFactor.height * baseDim.width * sizeFactor.width;
          if (area < minArea)
            continue;

          m_type = 9;
          m_numAreas = 2;
          m_weights.resize(m_numAreas);
          m_weights[0] = 1;
          m_weights[1] = -2;
          m_areas.resize(m_numAreas);
          m_areas[0].x = position.x;
          m_areas[0].y = position.y;
          m_areas[0].height = 3 * baseDim.height;
          m_areas[0].width = 3 * baseDim.width;
          m_areas[1].x = position.x + baseDim.width;
          m_areas[1].y = position.y + baseDim.height;
          m_areas[1].height = baseDim.height;
          m_areas[1].width = baseDim.width;
          m_initMean = 0;
          m_initSigma = INITSIGMA( m_numAreas );
          valid = true;
        }
        else if (prob
            < probType[0] + probType[1] + probType[2] + probType[3] + probType[4] + probType[5] + probType[6]
              + probType[7] + probType[8] + probType[9])
        {
          //check if feature is valid
          sizeFactor.height = 3;
          sizeFactor.width = 1;
          if (position.y + baseDim.height * sizeFactor.height >= patchSize.height || position.x
              + baseDim.width * sizeFactor.width
                                                                                     >= patchSize.width)
            continue;
          area = baseDim.height * sizeFactor.height * baseDim.width * sizeFactor.width;
          if (area < minArea)
            continue;

          m_type = 10;
          m_numAreas = 3;
          m_weights.resize(m_numAreas);
          m_weights[0] = 1;
          m_weights[1] = -1;
          m_weights[2] = 1;
          m_areas.resize(m_numAreas);
          m_areas[0].x = position.x;
          m_areas[0].y = position.y;
          m_areas[0].height = baseDim.height;
          m_areas[0].width = baseDim.width;
          m_areas[1].x = position.x;
          m_areas[1].y = position.y + baseDim.height;
          m_areas[1].height = baseDim.height;
          m_areas[1].width = baseDim.width;
          m_areas[2].y = position.y + baseDim.height * 2;
          m_areas[2].x = position.x;
          m_areas[2].height = baseDim.height;
          m_areas[2].width = baseDim.width;
          m_initMean = 128;
          m_initSigma = INITSIGMA( m_numAreas );
          valid = true;
        }
        else if (prob
            < probType[0] + probType[1] + probType[2] + probType[3] + probType[4] + probType[5] + probType[6]
              + probType[7] + probType[8] + probType[9] + probType[10])
        {
          //check if feature is valid
          sizeFactor.height = 1;
          sizeFactor.width = 3;
          if (position.y + baseDim.height * sizeFactor.height >= patchSize.height || position.x
              + baseDim.width * sizeFactor.width
                                                                                     >= patchSize.width)
            continue;
          area = baseDim.height * sizeFactor.height * baseDim.width * sizeFactor.width;
          if (area < minArea)
            continue;

          m_type = 11;
          m_numAreas = 3;
          m_weights.resize(m_numAreas);
          m_weights[0] = 1;
          m_weights[1] = -1;
          m_weights[2] = 1;
          m_areas.resize(m_numAreas);
          m_areas[0].x = position.x;
          m_areas[0].y = position.y;
          m_areas[0].height = baseDim.height;
          m_areas[0].width = baseDim.width;
          m_areas[1].x = position.x + baseDim.width;
          m_areas[1].y = position.y;
          m_areas[1].height = baseDim.height;
          m_areas[1].width = baseDim.width;
          m_areas[2].y = position.y;
          m_areas[2].x = position.x + 2 * baseDim.width;
          m_areas[2].height = baseDim.height;
          m_areas[2].width = baseDim.width;
          m_initMean = 128;
          m_initSigma = INITSIGMA( m_numAreas );
          valid = true;
        }
        else
          assert(false);
      }

      m_initSize = patchSize;
      m_curSize = m_initSize;
      m_scaleFactorWidth = m_scaleFactorHeight = 1.0f;
      m_scaleAreas.resize(m_numAreas);
      m_scaleWeights.resize(m_numAreas);
      for (int curArea = 0; curArea < m_numAreas; curArea++)
      {
        m_scaleAreas[curArea] = m_areas[curArea];
        m_scaleWeights[curArea] = (float) m_weights[curArea]
            / (float) (m_areas[curArea].width * m_areas[curArea].height);
      }
    }

    bool
    FeatureHaar::eval(ImageRepresentation* image, Rect ROI, float* result)
    {
      *result = 0.0f;
      cv::Point2i offset;
      offset = cv::Point2i(ROI.x, ROI.y);

      // define the minimum size
      Size minSize = Size(3, 3);

      // printf("in eval %d = %d\n",curSize.width,ROI.width );

      if (m_curSize.width != ROI.width || m_curSize.height != ROI.height)
      {
        m_curSize = cv::Size(ROI.width, ROI.height);
        if (!(m_initSize == m_curSize))
        {
          m_scaleFactorHeight = (float) m_curSize.height / m_initSize.height;
          m_scaleFactorWidth = (float) m_curSize.width / m_initSize.width;

          for (int curArea = 0; curArea < m_numAreas; curArea++)
          {
            m_scaleAreas[curArea].height = (int) floor((float) m_areas[curArea].height * m_scaleFactorHeight + 0.5f);
            m_scaleAreas[curArea].width = (int) floor((float) m_areas[curArea].width * m_scaleFactorWidth + 0.5f);

            if (m_scaleAreas[curArea].height < minSize.height || m_scaleAreas[curArea].width < minSize.width)
            {
              m_scaleFactorWidth = 0.0f;
              return false;
            }

            m_scaleAreas[curArea].x = (int) floor((float) m_areas[curArea].x * m_scaleFactorWidth + 0.5f);
            m_scaleAreas[curArea].y = (int) floor((float) m_areas[curArea].y * m_scaleFactorHeight + 0.5f);
            m_scaleWeights[curArea] = (float) m_weights[curArea]
                / (float) ((m_scaleAreas[curArea].width) * (m_scaleAreas[curArea].height));
          }
        }
        else
        {
          m_scaleFactorWidth = m_scaleFactorHeight = 1.0f;
          for (int curArea = 0; curArea < m_numAreas; curArea++)
          {
            m_scaleAreas[curArea] = m_areas[curArea];
            m_scaleWeights[curArea] = (float) m_weights[curArea]
                / (float) ((m_areas[curArea].width) * (m_areas[curArea].height));
          }
        }
      }

      if (m_scaleFactorWidth == 0.0f)
        return false;

      for (int curArea = 0; curArea < m_numAreas; curArea++)
      {
        *result += (float) image->getSum(
            Rect(m_scaleAreas[curArea].x + offset.x, m_scaleAreas[curArea].y + offset.y, m_scaleAreas[curArea].width,
                 m_scaleAreas[curArea].height))
                   * m_scaleWeights[curArea];
      }

      if (image->getUseVariance())
      {
        float variance = (float) image->getVariance(ROI);
        *result /= variance;
      }

      m_response = *result;

      return true;
    }

    void
    FeatureHaar::getInitialDistribution(EstimatedGaussDistribution* distribution)
    {
      distribution->setValues(m_initMean, m_initSigma);
    }

    ClassifierThreshold::ClassifierThreshold()
    {
      m_posSamples = new EstimatedGaussDistribution();
      m_negSamples = new EstimatedGaussDistribution();
      m_threshold = 0.0f;
      m_parity = 0;
    }

    ClassifierThreshold::~ClassifierThreshold()
    {
      if (m_posSamples != NULL)
        delete m_posSamples;
      if (m_negSamples != NULL)
        delete m_negSamples;
    }

    void*
    ClassifierThreshold::getDistribution(int target)
    {
      if (target == 1)
        return m_posSamples;
      else
        return m_negSamples;
    }

    void
    ClassifierThreshold::update(float value, int target)
    {
      //update distribution
      if (target == 1)
        m_posSamples->update(value);
      else
        m_negSamples->update(value);

      //adapt threshold and parity
      m_threshold = (m_posSamples->getMean() + m_negSamples->getMean()) / 2.0f;
      m_parity = (m_posSamples->getMean() > m_negSamples->getMean()) ? 1 : -1;
    }

    int
    ClassifierThreshold::eval(float value)
    {
      return (((m_parity * (value - m_threshold)) > 0) ? 1 : -1);
    }

    WeakClassifier::WeakClassifier()
    {
    }

    WeakClassifier::~WeakClassifier()
    {
    }

    bool
    WeakClassifier::update(ImageRepresentation* image, Rect ROI, int target)
    {
      return true;
    }

    int
    WeakClassifier::eval(ImageRepresentation* image, Rect ROI)
    {
      return 0;
    }

    int
    WeakClassifier::getType()
    {
      return 0;
    }

    float
    WeakClassifier::getValue(ImageRepresentation* image, Rect ROI)
    {
      return 0;
    }

    WeakClassifierHaarFeature::WeakClassifierHaarFeature(Size patchSize)
    {
      m_feature = new FeatureHaar(patchSize);
      generateRandomClassifier();
      m_feature->getInitialDistribution((EstimatedGaussDistribution*) m_classifier->getDistribution(-1));
      m_feature->getInitialDistribution((EstimatedGaussDistribution*) m_classifier->getDistribution(1));
    }

    void
    WeakClassifierHaarFeature::resetPosDist()
    {
      m_feature->getInitialDistribution((EstimatedGaussDistribution*) m_classifier->getDistribution(1));
      m_feature->getInitialDistribution((EstimatedGaussDistribution*) m_classifier->getDistribution(-1));
    }

    WeakClassifierHaarFeature::~WeakClassifierHaarFeature()
    {
      delete m_classifier;
      delete m_feature;

    }

    void
    WeakClassifierHaarFeature::generateRandomClassifier()
    {
      m_classifier = new ClassifierThreshold();
    }

    bool
    WeakClassifierHaarFeature::update(ImageRepresentation *image, Rect ROI, int target)
    {
      float value;

      bool valid = m_feature->eval(image, ROI, &value);
      if (!valid)
        return true;

      m_classifier->update(value, target);
      return (m_classifier->eval(value) != target);
    }

    int
    WeakClassifierHaarFeature::eval(ImageRepresentation *image, Rect ROI)
    {
      float value;
      bool valid = m_feature->eval(image, ROI, &value);
      if (!valid)
        return 0;

      return m_classifier->eval(value);
    }

    float
    WeakClassifierHaarFeature::getValue(ImageRepresentation *image, Rect ROI)
    {
      float value;
      bool valid = m_feature->eval(image, ROI, &value);
      if (!valid)
        return 0;

      return value;
    }

    EstimatedGaussDistribution*
    WeakClassifierHaarFeature::getPosDistribution()
    {
      return static_cast<EstimatedGaussDistribution*>(m_classifier->getDistribution(1));
    }

    EstimatedGaussDistribution*
    WeakClassifierHaarFeature::getNegDistribution()
    {
      return static_cast<EstimatedGaussDistribution*>(m_classifier->getDistribution(-1));
    }

    BaseClassifier::BaseClassifier(int numWeakClassifier, int iterationInit, Size patchSize)
    {
      this->m_numWeakClassifier = numWeakClassifier;
      this->m_iterationInit = iterationInit;

      weakClassifier = new WeakClassifier*[numWeakClassifier + iterationInit];
      m_idxOfNewWeakClassifier = numWeakClassifier;

      generateRandomClassifier(patchSize);

      m_referenceWeakClassifier = false;
      m_selectedClassifier = 0;

      m_wCorrect.assign(numWeakClassifier + iterationInit, 0);

      m_wWrong.assign(numWeakClassifier + iterationInit, 0);

      for (int curWeakClassifier = 0; curWeakClassifier < numWeakClassifier + iterationInit; curWeakClassifier++)
        m_wWrong[curWeakClassifier] = m_wCorrect[curWeakClassifier] = 1;
    }

    BaseClassifier::BaseClassifier(int numWeakClassifier, int iterationInit, WeakClassifier** weakClassifier)
    {
      this->m_numWeakClassifier = numWeakClassifier;
      this->m_iterationInit = iterationInit;
      this->weakClassifier = weakClassifier;
      m_referenceWeakClassifier = true;
      m_selectedClassifier = 0;
      m_idxOfNewWeakClassifier = numWeakClassifier;

      m_wCorrect.assign(numWeakClassifier + iterationInit, 0);
      m_wWrong.assign(numWeakClassifier + iterationInit, 0);

      for (int curWeakClassifier = 0; curWeakClassifier < numWeakClassifier + iterationInit; curWeakClassifier++)
        m_wWrong[curWeakClassifier] = m_wCorrect[curWeakClassifier] = 1;
    }

    BaseClassifier::~BaseClassifier()
    {
      if (!m_referenceWeakClassifier)
      {
        for (int curWeakClassifier = 0; curWeakClassifier < m_numWeakClassifier + m_iterationInit; curWeakClassifier++)
          delete weakClassifier[curWeakClassifier];

        delete[] weakClassifier;
      }
      m_wCorrect.clear();
      m_wWrong.clear();
    }

    void
    BaseClassifier::generateRandomClassifier(Size patchSize)
    {
      for (int curWeakClassifier = 0; curWeakClassifier < m_numWeakClassifier + m_iterationInit; curWeakClassifier++)
      {
        weakClassifier[curWeakClassifier] = new WeakClassifierHaarFeature(patchSize);
      }
    }

    int
    BaseClassifier::eval(ImageRepresentation *image, Rect ROI)
    {
      return weakClassifier[m_selectedClassifier]->eval(image, ROI);
    }

    float
    BaseClassifier::getValue(ImageRepresentation *image, Rect ROI, int weakClassifierIdx)
    {
      if (weakClassifierIdx < 0 || weakClassifierIdx >= m_numWeakClassifier)
        return weakClassifier[m_selectedClassifier]->getValue(image, ROI);
      return weakClassifier[weakClassifierIdx]->getValue(image, ROI);
    }

    void
    BaseClassifier::trainClassifier(ImageRepresentation* image, Rect ROI, int target, float importance, bool* errorMask)
    {
      //get poisson value
      double A = 1;
      int K = 0;
      int K_max = 10;
      while (1)
      {
        double U_k = (double) rand() / RAND_MAX;
        A *= U_k;
        if (K > K_max || A < exp(-importance))
          break;
        K++;
      }

      for (int curK = 0; curK <= K; curK++)
        for (int curWeakClassifier = 0; curWeakClassifier < m_numWeakClassifier + m_iterationInit; curWeakClassifier++)
          errorMask[curWeakClassifier] = weakClassifier[curWeakClassifier]->update(image, ROI, target);

    }

    void
    BaseClassifier::getErrorMask(ImageRepresentation* image, Rect ROI, int target, bool* errorMask)
    {
      for (int curWeakClassifier = 0; curWeakClassifier < m_numWeakClassifier + m_iterationInit; curWeakClassifier++)
        errorMask[curWeakClassifier] = (weakClassifier[curWeakClassifier]->eval(image, ROI) != target);
    }

    float
    BaseClassifier::getError(int curWeakClassifier)
    {
      if (curWeakClassifier == -1)
        curWeakClassifier = m_selectedClassifier;
      return m_wWrong[curWeakClassifier] / (m_wWrong[curWeakClassifier] + m_wCorrect[curWeakClassifier]);
    }

    int
    BaseClassifier::selectBestClassifier(bool* errorMask, float importance, std::vector<float> & errors)
    {
      float minError = FLT_MAX;
      int tmp_selectedClassifier = m_selectedClassifier;

      for (int curWeakClassifier = 0; curWeakClassifier < m_numWeakClassifier + m_iterationInit; curWeakClassifier++)
      {
        if (errorMask[curWeakClassifier])
        {
          m_wWrong[curWeakClassifier] += importance;
        }
        else
        {
          m_wCorrect[curWeakClassifier] += importance;
        }

        if (errors[curWeakClassifier] == FLT_MAX)
          continue;

        errors[curWeakClassifier] = m_wWrong[curWeakClassifier]
            / (m_wWrong[curWeakClassifier] + m_wCorrect[curWeakClassifier]);

        /*if(errors[curWeakClassifier] < 0.001 || !(errors[curWeakClassifier]>0.0))
         {
         errors[curWeakClassifier] = 0.001;
         }

         if(errors[curWeakClassifier] >= 1.0)
         errors[curWeakClassifier] = 0.999;

         assert (errors[curWeakClassifier] > 0.0);
         assert (errors[curWeakClassifier] < 1.0);*/

        if (curWeakClassifier < m_numWeakClassifier)
        {
          if (errors[curWeakClassifier] < minError)
          {
            minError = errors[curWeakClassifier];
            tmp_selectedClassifier = curWeakClassifier;
          }
        }
      }

      m_selectedClassifier = tmp_selectedClassifier;
      return m_selectedClassifier;
    }

    void
    BaseClassifier::getErrors(float* errors)
    {
      for (int curWeakClassifier = 0; curWeakClassifier < m_numWeakClassifier + m_iterationInit; curWeakClassifier++)
      {
        if (errors[curWeakClassifier] == FLT_MAX)
          continue;

        errors[curWeakClassifier] = m_wWrong[curWeakClassifier]
            / (m_wWrong[curWeakClassifier] + m_wCorrect[curWeakClassifier]);

        assert(errors[curWeakClassifier] > 0);
      }
    }

    int
    BaseClassifier::replaceWeakestClassifier(const std::vector<float> & errors, Size patchSize)
    {
      float maxError = 0.0f;
      int index = -1;

      //search the classifier with the largest error
      for (int curWeakClassifier = m_numWeakClassifier - 1; curWeakClassifier >= 0; curWeakClassifier--)
      {
        if (errors[curWeakClassifier] > maxError)
        {
          maxError = errors[curWeakClassifier];
          index = curWeakClassifier;
        }
      }

      assert(index > -1);
      assert(index != m_selectedClassifier);

      //replace
      m_idxOfNewWeakClassifier++;
      if (m_idxOfNewWeakClassifier == m_numWeakClassifier + m_iterationInit)
        m_idxOfNewWeakClassifier = m_numWeakClassifier;

      if (maxError > errors[m_idxOfNewWeakClassifier])
      {
        delete weakClassifier[index];
        weakClassifier[index] = weakClassifier[m_idxOfNewWeakClassifier];
        m_wWrong[index] = m_wWrong[m_idxOfNewWeakClassifier];
        m_wWrong[m_idxOfNewWeakClassifier] = 1;
        m_wCorrect[index] = m_wCorrect[m_idxOfNewWeakClassifier];
        m_wCorrect[m_idxOfNewWeakClassifier] = 1;

        weakClassifier[m_idxOfNewWeakClassifier] = new WeakClassifierHaarFeature(patchSize);

        return index;
      }
      else
        return -1;

    }

    void
    BaseClassifier::replaceClassifierStatistic(int sourceIndex, int targetIndex)
    {
      assert(targetIndex >=0);
      assert(targetIndex != m_selectedClassifier);
      assert(targetIndex < m_numWeakClassifier);

      //replace
      m_wWrong[targetIndex] = m_wWrong[sourceIndex];
      m_wWrong[sourceIndex] = 1.0f;
      m_wCorrect[targetIndex] = m_wCorrect[sourceIndex];
      m_wCorrect[sourceIndex] = 1.0f;
    }

    StrongClassifier::StrongClassifier(int numBaseClassifier, int numWeakClassifier, Size patchSize,
                                       bool useFeatureExchange, int iterationInit)
        :
          baseClassifier(0)
    {
      this->numBaseClassifier = numBaseClassifier;
      this->numAllWeakClassifier = numWeakClassifier + iterationInit;

      alpha.assign(numBaseClassifier, 0);

      this->patchSize = patchSize;
      this->useFeatureExchange = useFeatureExchange;
    }

    StrongClassifier::~StrongClassifier()
    {
      for (int curBaseClassifier = 0; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
        delete baseClassifier[curBaseClassifier];
      delete[] baseClassifier;
      alpha.clear();
    }

    float
    StrongClassifier::getFeatureValue(ImageRepresentation *image, Rect ROI, int baseClassifierIdx)
    {
      return baseClassifier[baseClassifierIdx]->getValue(image, ROI);
    }

    float
    StrongClassifier::eval(ImageRepresentation *image, Rect ROI)
    {
      float value = 0.0f;
      int curBaseClassifier = 0;

      for (curBaseClassifier = 0; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
        value += baseClassifier[curBaseClassifier]->eval(image, ROI) * alpha[curBaseClassifier];

      return value;
    }

    bool
    StrongClassifier::update(ImageRepresentation *image, Rect ROI, int target, float importance)
    {
      assert(true);
      return false;
    }

    bool
    StrongClassifier::updateSemi(ImageRepresentation *image, Rect ROI, float priorConfidence)
    {
      assert(true);
      return false;
    }

    float
    StrongClassifier::getSumAlpha(int toBaseClassifier)
    {
      float sumAlpha = 0;
      if (toBaseClassifier == -1)
        toBaseClassifier = numBaseClassifier;

      for (int curBaseClassifier = 0; curBaseClassifier < toBaseClassifier; curBaseClassifier++)
        sumAlpha += alpha[curBaseClassifier];

      return sumAlpha;
    }

    float
    StrongClassifier::getImportance(ImageRepresentation *image, Rect ROI, int target, int numBaseClassifier)
    {
      if (numBaseClassifier == -1)
        numBaseClassifier = this->numBaseClassifier;

      float importance = 1.0f;

      for (int curBaseClassifier = 0; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
      {
        bool error = (baseClassifier[curBaseClassifier]->eval(image, ROI) != target);

        if (error)
          importance /= (2 * baseClassifier[curBaseClassifier]->getError());
        else
          importance /= (2 * (1 - baseClassifier[curBaseClassifier]->getError()));
      }

      return importance / numBaseClassifier;
    }

    void
    StrongClassifier::resetWeightDistribution()
    {
      for (int curBaseClassifier = 0; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
      {
        for (int curWeakClassifier = 0; curWeakClassifier < baseClassifier[curBaseClassifier]->getNumWeakClassifier();
            curWeakClassifier++)
        {
          baseClassifier[curBaseClassifier]->setWCorrect(curWeakClassifier, 1.0);
          baseClassifier[curBaseClassifier]->setWWrong(curWeakClassifier, 1.0);
        }
      }
    }

    StrongClassifierDirectSelection::StrongClassifierDirectSelection(int numBaseClassifier, int numWeakClassifier,
                                                                     Size patchSize, bool useFeatureExchange,
                                                                     int iterationInit)
        :
          StrongClassifier(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange, iterationInit)
    {
      this->useFeatureExchange = useFeatureExchange;
      baseClassifier = new BaseClassifier*[numBaseClassifier];
      baseClassifier[0] = new BaseClassifier(numWeakClassifier, iterationInit, patchSize);

      for (int curBaseClassifier = 1; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
        baseClassifier[curBaseClassifier] = new BaseClassifier(numWeakClassifier, iterationInit,
                                                               baseClassifier[0]->getReferenceWeakClassifier());

      m_errorMask = new bool[numAllWeakClassifier];
      m_errors.resize(numAllWeakClassifier);
      m_sumErrors.resize(numAllWeakClassifier);
    }

    StrongClassifierDirectSelection::~StrongClassifierDirectSelection()
    {
      delete[] m_errorMask;
    }

    bool
    StrongClassifierDirectSelection::update(ImageRepresentation *image, Rect ROI, int target, float importance)
    {
      memset(m_errorMask, 0, numAllWeakClassifier * sizeof(bool));
      m_errors.assign(numAllWeakClassifier, 0);
      m_sumErrors.assign(numAllWeakClassifier, 0);

      baseClassifier[0]->trainClassifier(image, ROI, target, importance, m_errorMask);
      for (int curBaseClassifier = 0; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
      {
        int selectedClassifier = baseClassifier[curBaseClassifier]->selectBestClassifier(m_errorMask, importance,
                                                                                         m_errors);

        if (m_errors[selectedClassifier] >= 0.5)
          alpha[curBaseClassifier] = 0;
        else
          alpha[curBaseClassifier] = logf((1.0f - m_errors[selectedClassifier]) / m_errors[selectedClassifier]);

        if (m_errorMask[selectedClassifier])
          importance *= (float) sqrt((1.0f - m_errors[selectedClassifier]) / m_errors[selectedClassifier]);
        else
          importance *= (float) sqrt(m_errors[selectedClassifier] / (1.0f - m_errors[selectedClassifier]));

        //weight limitation
        //if (importance > 100) importance = 100;

        //sum up errors
        for (int curWeakClassifier = 0; curWeakClassifier < numAllWeakClassifier; curWeakClassifier++)
        {
          if (m_errors[curWeakClassifier] != FLT_MAX && m_sumErrors[curWeakClassifier] >= 0)
            m_sumErrors[curWeakClassifier] += m_errors[curWeakClassifier];
        }

        //mark feature as used
        m_sumErrors[selectedClassifier] = -1;
        m_errors[selectedClassifier] = FLT_MAX;
      }

      if (useFeatureExchange)
      {
        int replacedClassifier = baseClassifier[0]->replaceWeakestClassifier(m_sumErrors, patchSize);
        if (replacedClassifier > 0)
          for (int curBaseClassifier = 1; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
            baseClassifier[curBaseClassifier]->replaceClassifierStatistic(
                baseClassifier[0]->getIdxOfNewWeakClassifier(), replacedClassifier);
      }

      return true;
    }

    StrongClassifierStandard::StrongClassifierStandard(int numBaseClassifier, int numWeakClassifier, Size patchSize,
                                                       bool useFeatureExchange, int iterationInit)
        :
          StrongClassifier(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange, iterationInit)
    {
      // init Base Classifier
      baseClassifier = new BaseClassifier*[numBaseClassifier];

      for (int curBaseClassifier = 0; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
      {
        baseClassifier[curBaseClassifier] = new BaseClassifier(numWeakClassifier, iterationInit, patchSize);
      }

      m_errorMask = new bool[numAllWeakClassifier];
      m_errors.resize(numAllWeakClassifier);
    }

    StrongClassifierStandard::~StrongClassifierStandard()
    {
      delete[] m_errorMask;
    }

    bool
    StrongClassifierStandard::update(ImageRepresentation *image, Rect ROI, int target, float importance)
    {
      int curBaseClassifier;
      for (curBaseClassifier = 0; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
      {
        memset(m_errorMask, 0x00, numAllWeakClassifier * sizeof(bool));
        m_errors.assign(numAllWeakClassifier, 0);

        int selectedClassifier;
        baseClassifier[curBaseClassifier]->trainClassifier(image, ROI, target, importance, m_errorMask);
        selectedClassifier = baseClassifier[curBaseClassifier]->selectBestClassifier(m_errorMask, importance, m_errors);

        if (m_errors[selectedClassifier] >= 0.5)
          alpha[curBaseClassifier] = 0;
        else
          alpha[curBaseClassifier] = logf((1.0f - m_errors[selectedClassifier]) / m_errors[selectedClassifier]);

        if (m_errorMask[selectedClassifier])
          importance *= (float) sqrt((1.0f - m_errors[selectedClassifier]) / m_errors[selectedClassifier]);
        else
          importance *= (float) sqrt(m_errors[selectedClassifier] / (1.0f - m_errors[selectedClassifier]));

        //weight limitation
        //if (importance > 100) importance = 100;

        if (useFeatureExchange)
          baseClassifier[curBaseClassifier]->replaceWeakestClassifier(m_errors, patchSize);

      }

      return true;
    }

    StrongClassifierStandardSemi::StrongClassifierStandardSemi(int numBaseClassifier, int numWeakClassifier,
                                                               Size patchSize, bool useFeatureExchange,
                                                               int iterationInit)
        :
          StrongClassifier(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange, iterationInit)
    {
      // init Base Classifier
      baseClassifier = new BaseClassifier*[numBaseClassifier];

      m_pseudoTarget.resize(numBaseClassifier);
      m_pseudoLambda.resize(numBaseClassifier);

      for (int curBaseClassifier = 0; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
      {
        baseClassifier[curBaseClassifier] = new BaseClassifier(numWeakClassifier, iterationInit, patchSize);
      }

      m_errorMask = new bool[numAllWeakClassifier];
      m_errors.resize(numAllWeakClassifier);
    }

    StrongClassifierStandardSemi::~StrongClassifierStandardSemi()
    {
      delete[] m_errorMask;
    }

    bool
    StrongClassifierStandardSemi::updateSemi(ImageRepresentation *image, Rect ROI, float priorConfidence)
    {

      float value = 0.0f, kvalue = 0.0f;

      bool decided = false;
      bool uncertain = false;
      bool used = false;
      float sumAlpha = 0;

      float scaleFactor = 2.0f;

      int curBaseClassifier;
      for (curBaseClassifier = 0; curBaseClassifier < numBaseClassifier; curBaseClassifier++)
      {
        memset(m_errorMask, 0x00, numAllWeakClassifier * sizeof(bool));
        m_errors.assign(numAllWeakClassifier, 0);

        int selectedClassifier;
        {
          //scale
          if (sumAlpha > 0)
            kvalue = value / this->getSumAlpha();
          else
            kvalue = 0;

          float combinedDecision = tanh(scaleFactor * priorConfidence) - tanh(scaleFactor * kvalue);
          int myTarget = static_cast<int>(sign(combinedDecision));

          m_pseudoTarget[curBaseClassifier] = myTarget;
          float myImportance = ::fabs(combinedDecision);
          m_pseudoLambda[curBaseClassifier] = myImportance;

          baseClassifier[curBaseClassifier]->trainClassifier(image, ROI, myTarget, myImportance, m_errorMask);
          selectedClassifier = baseClassifier[curBaseClassifier]->selectBestClassifier(m_errorMask, myImportance,
                                                                                       m_errors);
        }

        float curValue = baseClassifier[curBaseClassifier]->eval(image, ROI) * alpha[curBaseClassifier];
        value += curValue;
        sumAlpha += alpha[curBaseClassifier];

        if (m_errors[selectedClassifier] >= 0.5)
          alpha[curBaseClassifier] = 0;
        else
          alpha[curBaseClassifier] = logf((1.0f - m_errors[selectedClassifier]) / m_errors[selectedClassifier]);

        if (useFeatureExchange)
          baseClassifier[curBaseClassifier]->replaceWeakestClassifier(m_errors, patchSize);

      }

      return used;
    }

    Detector::Detector(StrongClassifier* classifier)
        :
          m_sizeDetections(0)
    {
      this->m_classifier = classifier;

      m_sizeConfidences = 0;
      m_maxConfidence = -FLT_MAX;
      m_numDetections = 0;
      m_idxBestDetection = -1;
    }

    Detector::~Detector()
    {
    }

    void
    Detector::prepareConfidencesMemory(int numPatches)
    {
      if (numPatches <= m_sizeConfidences)
        return;

      m_sizeConfidences = numPatches;
      m_confidences.resize(numPatches);
    }

    void
    Detector::prepareDetectionsMemory(int numDetections)
    {
      if (numDetections <= m_sizeDetections)
        return;

      m_sizeDetections = numDetections;
      m_idxDetections.resize(numDetections);
    }

    void
    Detector::classify(ImageRepresentation* image, Patches* patches, float minMargin)
    {
      int numPatches = patches->getNum();

      prepareConfidencesMemory(numPatches);

      m_numDetections = 0;
      m_idxBestDetection = -1;
      m_maxConfidence = -FLT_MAX;
      int numBaseClassifiers = m_classifier->getNumBaseClassifier();

      for (int curPatch = 0; curPatch < numPatches; curPatch++)
      {
        m_confidences[curPatch] = m_classifier->eval(image, patches->getRect(curPatch));

        if (m_confidences[curPatch] > m_maxConfidence)
        {
          m_maxConfidence = m_confidences[curPatch];
          m_idxBestDetection = curPatch;
        }
        if (m_confidences[curPatch] > minMargin)
          m_numDetections++;
      }

      prepareDetectionsMemory(m_numDetections);
      int curDetection = -1;
      for (int curPatch = 0; curPatch < numPatches; curPatch++)
      {
        if (m_confidences[curPatch] > minMargin)
          m_idxDetections[++curDetection] = curPatch;
      }
    }

    void
    Detector::classifySmooth(ImageRepresentation* image, Patches* patches, float minMargin)
    {
      int numPatches = patches->getNum();

      prepareConfidencesMemory(numPatches);

      m_numDetections = 0;
      m_idxBestDetection = -1;
      m_maxConfidence = -FLT_MAX;
      int numBaseClassifiers = m_classifier->getNumBaseClassifier();

      PatchesRegularScan *regPatches = (PatchesRegularScan*) patches;
      Size patchGrid = regPatches->getPatchGrid();

      if ((patchGrid.width != m_confMatrix.cols) || (patchGrid.height != m_confMatrix.rows))
      {
        m_confMatrix.create(patchGrid.height, patchGrid.width);
        m_confMatrixSmooth.create(patchGrid.height, patchGrid.width);
        m_confImageDisplay.create(patchGrid.height, patchGrid.width);
      }

      int curPatch = 0;
      // Eval and filter
      for (int row = 0; row < patchGrid.height; row++)
      {
        for (int col = 0; col < patchGrid.width; col++)
        {
          //int returnedInLayer;
          m_confidences[curPatch] = m_classifier->eval(image, patches->getRect(curPatch));

          // fill matrix
          m_confMatrix(row, col) = m_confidences[curPatch];
          curPatch++;
        }
      }

      // Filter
      //cv::GaussianBlur(m_confMatrix,m_confMatrixSmooth,cv::Size(3,3),0.8);
      cv::GaussianBlur(m_confMatrix, m_confMatrixSmooth, cv::Size(3, 3), 0);

      // Make display friendly
      double min_val, max_val;
      cv::minMaxLoc(m_confMatrixSmooth, &min_val, &max_val);
      for (int y = 0; y < m_confImageDisplay.rows; y++)
      {
        unsigned char* pConfImg = m_confImageDisplay[y];
        const float* pConfData = m_confMatrixSmooth[y];
        for (int x = 0; x < m_confImageDisplay.cols; x++, pConfImg++, pConfData++)
        {
          *pConfImg = static_cast<unsigned char>(255.0 * (*pConfData - min_val) / (max_val - min_val));
        }
      }

      // Get best detection
      curPatch = 0;
      for (int row = 0; row < patchGrid.height; row++)
      {
        for (int col = 0; col < patchGrid.width; col++)
        {
          // fill matrix
          m_confidences[curPatch] = m_confMatrixSmooth(row, col);

          if (m_confidences[curPatch] > m_maxConfidence)
          {
            m_maxConfidence = m_confidences[curPatch];
            m_idxBestDetection = curPatch;
          }
          if (m_confidences[curPatch] > minMargin)
          {
            m_numDetections++;
          }
          curPatch++;
        }
      }

      prepareDetectionsMemory(m_numDetections);
      int curDetection = -1;
      for (int curPatch = 0; curPatch < numPatches; curPatch++)
      {
        if (m_confidences[curPatch] > minMargin)
          m_idxDetections[++curDetection] = curPatch;
      }
    }

    int
    Detector::getNumDetections()
    {
      return m_numDetections;
    }

    float
    Detector::getConfidence(int patchIdx)
    {
      return m_confidences[patchIdx];
    }

    float
    Detector::getConfidenceOfDetection(int detectionIdx)
    {
      return m_confidences[getPatchIdxOfDetection(detectionIdx)];
    }

    int
    Detector::getPatchIdxOfBestDetection()
    {
      return m_idxBestDetection;
    }

    int
    Detector::getPatchIdxOfDetection(int detectionIdx)
    {
      return m_idxDetections[detectionIdx];
    }

    BoostingTracker::BoostingTracker(ImageRepresentation* image, Rect initPatch, Rect validROI, int numBaseClassifier)
    {
      int numWeakClassifier = numBaseClassifier * 10;
      bool useFeatureExchange = true;
      int iterationInit = 50;
      cv::Size patchSize(initPatch.width, initPatch.height);

      this->validROI = validROI;

      classifier = new StrongClassifierDirectSelection(numBaseClassifier, numWeakClassifier, patchSize,
                                                       useFeatureExchange, iterationInit);

      detector = new Detector(classifier);

      trackedPatch = initPatch;
      Rect trackingROI = getTrackingROI(2.0f);
      cv::Size trackedPatchSize(trackedPatch.width, trackedPatch.height);
      Patches* trackingPatches = new PatchesRegularScan(trackingROI, validROI, trackedPatchSize, 0.99f);

      iterationInit = 50;
      for (int curInitStep = 0; curInitStep < iterationInit; curInitStep++)
      {
        std::cout << "\rinit tracker... " << int(((float) curInitStep) / (iterationInit - 1) * 100) << " %%";

        classifier->update(image, trackingPatches->getSpecialRect("UpperLeft"), -1);
        classifier->update(image, trackedPatch, 1);
        classifier->update(image, trackingPatches->getSpecialRect("UpperRight"), -1);
        classifier->update(image, trackedPatch, 1);
        classifier->update(image, trackingPatches->getSpecialRect("LowerLeft"), -1);
        classifier->update(image, trackedPatch, 1);
        classifier->update(image, trackingPatches->getSpecialRect("LowerRight"), -1);
        classifier->update(image, trackedPatch, 1);
      }

      confidence = -1;
      delete trackingPatches;

    }

    BoostingTracker::~BoostingTracker(void)
    {
      delete detector;
      delete classifier;
    }

    bool
    BoostingTracker::track(ImageRepresentation* image, Patches* patches)
    {
      //detector->classify (image, patches);
      detector->classifySmooth(image, patches);

      //move to best detection
      if (detector->getNumDetections() <= 0)
      {
        confidence = 0;
        return false;
      }

      trackedPatch = patches->getRect(detector->getPatchIdxOfBestDetection());
      confidence = detector->getConfidenceOfBestDetection();

      classifier->update(image, patches->getSpecialRect("UpperLeft"), -1);
      classifier->update(image, trackedPatch, 1);
      classifier->update(image, patches->getSpecialRect("UpperRight"), -1);
      classifier->update(image, trackedPatch, 1);
      classifier->update(image, patches->getSpecialRect("UpperLeft"), -1);
      classifier->update(image, trackedPatch, 1);
      classifier->update(image, patches->getSpecialRect("LowerRight"), -1);
      classifier->update(image, trackedPatch, 1);

      return true;
    }

    Rect
    BoostingTracker::getTrackingROI(float searchFactor)
    {
      Rect searchRegion;

      searchRegion = RectMultiply(trackedPatch, searchFactor);
      //check
      if (searchRegion.y + searchRegion.height > validROI.height)
        searchRegion.height = validROI.height - searchRegion.y;
      if (searchRegion.x + searchRegion.width > validROI.width)
        searchRegion.width = validROI.width - searchRegion.x;

      return searchRegion;
    }

    float
    BoostingTracker::getConfidence()
    {
      return confidence / classifier->getSumAlpha();
    }

    Rect
    BoostingTracker::getTrackedPatch()
    {
      return trackedPatch;
    }

    cv::Point2i
    BoostingTracker::getCenter()
    {
      cv::Point2i center;
      center.y = trackedPatch.y + trackedPatch.height / 2;
      center.x = trackedPatch.x + trackedPatch.width / 2;
      return center;
    }

    SemiBoostingTracker::SemiBoostingTracker(ImageRepresentation* image, Rect initPatch, Rect validROI,
                                             int numBaseClassifier)
    {
      int numWeakClassifier = 100;
      bool useFeatureExchange = true;
      int iterationInit = 50;
      Size patchSize(initPatch.width, initPatch.height);

      this->validROI = validROI;

      //	classifierOff = new StrongClassifierDirectSelection(numBaseClassifier, numBaseClassifier*10, patchSize, useFeatureExchange, iterationInit);
      classifierOff = new StrongClassifierStandardSemi(numBaseClassifier, numWeakClassifier, patchSize,
                                                       useFeatureExchange, iterationInit);
      classifier = new StrongClassifierStandardSemi(numBaseClassifier, numWeakClassifier, patchSize, useFeatureExchange,
                                                    iterationInit);

      detector = new Detector(classifier);

      trackedPatch = initPatch;
      Rect trackingROI = getTrackingROI(2.0f);
      Size trackedPatchSize(trackedPatch.width, trackedPatch.height);
      Patches* trackingPatches = new PatchesRegularScan(trackingROI, validROI, trackedPatchSize, 0.99f);

      iterationInit = 50;
      for (int curInitStep = 0; curInitStep < iterationInit; curInitStep++)
      {
        std::cout << "\rinit tracker... " << int(((float) curInitStep) / (iterationInit - 1) * 100) << " %%";
        classifier->updateSemi(image, trackingPatches->getSpecialRect("UpperLeft"), -1);
        classifier->updateSemi(image, trackedPatch, 1);
        classifier->updateSemi(image, trackingPatches->getSpecialRect("UpperRight"), -1);
        classifier->updateSemi(image, trackedPatch, 1);
        classifier->updateSemi(image, trackingPatches->getSpecialRect("LowerLeft"), -1);
        classifier->updateSemi(image, trackedPatch, 1);
        classifier->updateSemi(image, trackingPatches->getSpecialRect("LowerRight"), -1);
        classifier->updateSemi(image, trackedPatch, 1);
      }
      std::cout << " done." << std::endl;

      //one (first) shot learning
      iterationInit = 50;
      for (int curInitStep = 0; curInitStep < iterationInit; curInitStep++)
      {
        std::cout << "\rinit detector... " << int(((float) curInitStep) / (iterationInit - 1) * 100) << " %%";

        classifierOff->updateSemi(image, trackedPatch, 1);
        classifierOff->updateSemi(image, trackingPatches->getSpecialRect("UpperLeft"), -1);
        classifierOff->updateSemi(image, trackedPatch, 1);
        classifierOff->updateSemi(image, trackingPatches->getSpecialRect("UpperRight"), -1);
        classifierOff->updateSemi(image, trackedPatch, 1);
        classifierOff->updateSemi(image, trackingPatches->getSpecialRect("LowerLeft"), -1);
        classifierOff->updateSemi(image, trackedPatch, 1);
        classifierOff->updateSemi(image, trackingPatches->getSpecialRect("LowerRight"), -1);
      }

      delete trackingPatches;

      confidence = -1;
      priorConfidence = -1;

    }

    bool
    SemiBoostingTracker::track(ImageRepresentation* image, Patches* patches)
    {
      //detector->classify(image, patches);
      detector->classifySmooth(image, patches);

      //move to best detection
      if (detector->getNumDetections() <= 0)
      {
        confidence = 0;
        priorConfidence = 0;
        return false;
      }

      trackedPatch = patches->getRect(detector->getPatchIdxOfBestDetection());
      confidence = detector->getConfidenceOfBestDetection();

      float off;

      //updates
      /*int numUpdates = 10;
       Rect tmp;
       for (int curUpdate = 0; curUpdate < numUpdates; curUpdate++)
       {
       tmp = patches->getSpecialRect ("Random");
       off = classifierOff->eval(image, tmp)/classifierOff->getSumAlpha();
       classifier->updateSemi (image, tmp, off);

       priorConfidence = classifierOff->eval(image, trackedPatch)/classifierOff->getSumAlpha();
       classifier->updateSemi (image, trackedPatch, priorConfidence);

       }*/

      Rect tmp = patches->getSpecialRect("UpperLeft");
      off = classifierOff->eval(image, tmp) / classifierOff->getSumAlpha();
      classifier->updateSemi(image, tmp, off);

      priorConfidence = classifierOff->eval(image, trackedPatch) / classifierOff->getSumAlpha();
      classifier->updateSemi(image, trackedPatch, priorConfidence);

      tmp = patches->getSpecialRect("LowerLeft");
      off = classifierOff->eval(image, tmp) / classifierOff->getSumAlpha();
      classifier->updateSemi(image, tmp, off);

      priorConfidence = classifierOff->eval(image, trackedPatch) / classifierOff->getSumAlpha();
      classifier->updateSemi(image, trackedPatch, priorConfidence);

      tmp = patches->getSpecialRect("UpperRight");
      off = classifierOff->eval(image, tmp) / classifierOff->getSumAlpha();
      classifier->updateSemi(image, tmp, off);

      priorConfidence = classifierOff->eval(image, trackedPatch) / classifierOff->getSumAlpha();
      classifier->updateSemi(image, trackedPatch, priorConfidence);

      tmp = patches->getSpecialRect("LowerRight");
      off = classifierOff->eval(image, tmp) / classifierOff->getSumAlpha();
      classifier->updateSemi(image, tmp, off);

      priorConfidence = classifierOff->eval(image, trackedPatch) / classifierOff->getSumAlpha();
      classifier->updateSemi(image, trackedPatch, priorConfidence);

      return true;
    }

    Rect
    SemiBoostingTracker::getTrackingROI(float searchFactor)
    {
      Rect searchRegion;

      searchRegion = RectMultiply(trackedPatch, searchFactor);
      //check
      if (searchRegion.y + searchRegion.height > validROI.height)
        searchRegion.height = validROI.height - searchRegion.y;
      if (searchRegion.x + searchRegion.width > validROI.width)
        searchRegion.width = validROI.width - searchRegion.x;

      return searchRegion;
    }

    float
    SemiBoostingTracker::getConfidence()
    {
      return confidence / classifier->getSumAlpha();
    }

    float
    SemiBoostingTracker::getPriorConfidence()
    {
      return priorConfidence;
    }

    Rect
    SemiBoostingTracker::getTrackedPatch()
    {
      return trackedPatch;
    }

    cv::Point2i
    SemiBoostingTracker::getCenter()
    {
      cv::Point2i center;
      center.y = trackedPatch.y + trackedPatch.height / 2;
      center.x = trackedPatch.x + trackedPatch.width / 2;
      return center;
    }

  }
}

/* Original email that allows OpenCV to publish that code under BSD


 Delivered-To: vrabaud@willowgarage.com
 Received: by 10.68.68.114 with SMTP id v18csp70225pbt;
 Wed, 18 Jul 2012 05:20:48 -0700 (PDT)
 Received: by 10.180.102.136 with SMTP id fo8mr6253489wib.19.1342614047314;
 Wed, 18 Jul 2012 05:20:47 -0700 (PDT)
 Return-Path: <grabner@vision.ee.ethz.ch>
 Received: from smtp.ee.ethz.ch (smtp.ee.ethz.ch. [129.132.2.219])
 by mx.google.com with ESMTPS id u4si28131263wed.25.2012.07.18.05.20.46
 (version=TLSv1/SSLv3 cipher=OTHER);
 Wed, 18 Jul 2012 05:20:47 -0700 (PDT)
 Received-SPF: pass (google.com: domain of grabner@vision.ee.ethz.ch designates 129.132.2.219 as permitted sender) client-ip=129.132.2.219;
 Authentication-Results: mx.google.com; spf=pass (google.com: domain of grabner@vision.ee.ethz.ch designates 129.132.2.219 as permitted sender) smtp.mail=grabner@vision.ee.ethz.ch
 Received: from localhost (localhost [127.0.0.1])
 by smtp.ee.ethz.ch (Postfix) with ESMTP id BD95ED930C;
 Wed, 18 Jul 2012 14:20:45 +0200 (MEST)
 X-Virus-Scanned: by amavisd-new on smtp.ee.ethz.ch
 Received: from smtp.ee.ethz.ch ([127.0.0.1])
 by localhost (.ee.ethz.ch [127.0.0.1]) (amavisd-new, port 10024)
 with LMTP id eRmMmgCtT1I8; Wed, 18 Jul 2012 14:20:45 +0200 (MEST)
 Received: from [129.132.158.18] (biwiwin08.ee.ethz.ch [129.132.158.18])
 by smtp.ee.ethz.ch (Postfix) with ESMTP id 8CC0ED9307;
 Wed, 18 Jul 2012 14:20:45 +0200 (MEST)
 Message-ID: <5006AA1D.1010906@vision.ee.ethz.ch>
 Date: Wed, 18 Jul 2012 14:20:45 +0200
 From: Helmut Grabner <grabner@vision.ee.ethz.ch>
 User-Agent: Mozilla/5.0 (Windows; U; Windows NT 6.1; en; rv:1.9.2.15) Gecko/20110303 Thunderbird/3.1.9
 MIME-Version: 1.0
 To: Vincent Rabaud <vrabaud@willowgarage.com>
 CC: Van Gool Luc <vangool@vision.ee.ethz.ch>,
 Horst Bischof <bischof@icg.tugraz.at>,
 Severin Stalder <sstalder@vision.ee.ethz.ch>,
 Michael Grabner <Michael.Grabner@microsoft.com>,
 Christian Leistner <leistner@vision.ee.ethz.ch>,
 Helmut Grabner <grabner@vision.ee.ethz.ch>
 Subject: Re: online boosting tracking in OpenCV
 References: <CAK+j2t8Ch3hGO_xvA8XMVSWtydGBCVgq63=pzNyAaKvh4TaZfg@mail.gmail.com>
 In-Reply-To: <CAK+j2t8Ch3hGO_xvA8XMVSWtydGBCVgq63=pzNyAaKvh4TaZfg@mail.gmail.com>
 Content-Type: text/plain; charset=ISO-8859-1; format=flowed
 Content-Transfer-Encoding: 7bit

 hi vincent,

 > We definitely respect any of your decision (vision is already very
 > grateful for your research :) ) but if you are willing to change your
 > code to BSD, we could then integrate it seemlessly into OpenCV. All I
 > would need from you is just an answer to this email saying: I am willing
 > to have OpenCV use my code as BSD.

 I am willing to have OpenCV use my code (on-line boosting trackers) as BSD.

 > And if you do accept so, please also make sure that you discuss it with
 > the co-authors of the code and that they are cced in that mail.

 [x] done.

 cheers
 helmut

 --
 -----------------------------------------------------------
 Dr. Helmut GRABNER               grabner@vision.ee.ethz.ch
 Computer Vision Lab        www.vision.ee.ethz.ch/~hegrabne
 ETH-Zurich, ETZ J78                 phone: +41 44 632 4990
 Sternwartstr. 7                     fax:   +41 44 632 1199
 CH-8092 Zurich, Switzerland         cell:  +41 78 625 3693
 -----------------------------------------------------------

 */
