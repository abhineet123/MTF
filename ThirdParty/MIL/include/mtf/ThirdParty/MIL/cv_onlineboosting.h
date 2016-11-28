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

#ifndef __OPENCV_ONLINE_BOOSTING_H__
#define __OPENCV_ONLINE_BOOSTING_H__

#include <opencv2/core/core.hpp>

namespace cv
{
  namespace boosting
  {
    class ImageRepresentation
    {
    public:

      ImageRepresentation(const cv::Mat & image, Size imagSize);
      ImageRepresentation(const cv::Mat & image, Size imagSize, Rect imageROI);
      void
      defaultInit(const cv::Mat & image, Size imageSize);

      int
      getSum(Rect imageROI);
      float
      getMean(Rect imagROI);
      unsigned int
      getValue(cv::Point2i position);
      void
      setNewROI(Rect ROI);
      void
      setNewImageSize(Rect ROI);
      void
      setNewImageAndROI(const cv::Mat & image, Rect ROI);
      float
      getVariance(Rect imageROI);
      long
      getSqSum(Rect imageROI);
      bool
      getUseVariance()
      {
        return m_useVariance;
      }
      ;
      void
      setUseVariance(bool useVariance)
      {
        this->m_useVariance = useVariance;
      }
      ;

    private:

      bool m_useVariance;
      void
      createIntegralsOfROI(const cv::Mat & image);

      cv::Size m_imageSize;
      cv::Mat_<int> intImage;
      cv::Mat_<double> intSqImage;
      Rect m_ROI;
      cv::Point2i m_offset;
    };

    class Patches
    {
    public:

      Patches();
      Patches(int num);
      virtual
      ~Patches(void)
      {
      }
      ;

      virtual Rect
      getRect(int index);
      virtual Rect
      getSpecialRect(const char* what);
      virtual Rect
      getSpecialRect(const char* what, Size patchSize);

      virtual Rect
      getROI();
      virtual int
      getNum(void)
      {
        return num;
      }
      ;

      int
      checkOverlap(Rect rect);

      virtual bool
      isDetection(Rect eval, unsigned char *labeledImg, int imgWidth);
      virtual int
      getNumPatchesX()
      {
        return numPatchesX;
      }
      ;
      virtual int
      getNumPatchesY()
      {
        return numPatchesY;
      }
      ;

    protected:

      void
      setCheckedROI(Rect imageROI, Rect validROI);

      std::vector<Rect> patches;
      int num;
      Rect ROI;
      int numPatchesX;
      int numPatchesY;
    };

    class PatchesRegularScan: public Patches
    {
    public:

      PatchesRegularScan(Rect imageROI, Size patchSize, float relOverlap);
      PatchesRegularScan(Rect imageROI, Rect validROI, Size patchSize, float relOverlap);
      virtual
      ~PatchesRegularScan(void);

      Rect
      getSpecialRect(const char* what);
      Rect
      getSpecialRect(const char* what, Size patchSize);
      cv::Size
      getPatchGrid()
      {
        return m_patchGrid;
      }
      ;

    private:

      void
      calculatePatches(Rect imageROI, Rect validROI, Size patchSize, float relOverlap);

      Rect m_rectUpperLeft;
      Rect m_rectUpperRight;
      Rect m_rectLowerLeft;
      Rect m_rectLowerRight;
      cv::Size m_patchGrid;

    };

    class PatchesRegularScaleScan: public Patches
    {
    public:

      PatchesRegularScaleScan(Rect imageROI, Size patchSize, float relOverlap, float scaleStart, float scaleEnd,
                              float scaleFactor);
      PatchesRegularScaleScan(Rect imageROI, Rect validROI, Size patchSize, float relOverlap, float scaleStart,
                              float scaleEnd, float scaleFactor);
      virtual
      ~PatchesRegularScaleScan();

      Rect
      getSpecialRect(const char* what);
      Rect
      getSpecialRect(const char* what, Size patchSize);

    private:

      void
      calculatePatches(Rect imageROI, Rect validROI, Size patchSize, float relOverlap, float scaleStart, float scaleEnd,
                       float scaleFactor);

    };

    class PatchesFunctionScaleScan: public Patches
    {
    public:

      typedef float
      (*GetScale)(int, int);

      PatchesFunctionScaleScan(Rect imageROI, Size patchSize, float relOverlap, GetScale getScale);
      PatchesFunctionScaleScan(Rect imageROI, Rect validROI, Size PatchSize, float relOverlap, GetScale getScale);
      PatchesFunctionScaleScan(Rect imageROI, Size patchSize, float relOverlap, float coefY, float coef1,
                               float minScaleFactor = 1.0f);
      PatchesFunctionScaleScan(Rect imageROI, Rect validROI, Size patchSize, float relOverlap, float coefY, float coef1,
                               float minScaleFactor = 1.0f);
      virtual
      ~PatchesFunctionScaleScan();

      Rect
      getSpecialRect(const char* what);
      Rect
      getSpecialRect(const char* what, Size patchSize);

    private:

      void
      calculatePatches(Rect imageROI, Rect validROI, Size patchSize, float relOverlap, GetScale getScale);
      void
      calculatePatches(Rect imageROI, Rect validROI, Size patchSize, float relOverlap, float coefY, float coef1,
                       float minScaleFactor);

      Rect rectUpperLeft;
      Rect rectUpperRight;
      Rect rectLowerLeft;
      Rect rectLowerRight;
    };

    class PatchesManualSet: public Patches
    {
    public:

      PatchesManualSet(int numPatches, Rect* patches);
      PatchesManualSet(int numPatches, Rect* patches, Rect ROI);
      virtual
      ~PatchesManualSet(void);

      Rect
      getSpecialRect(const char* what)
      {
        return Rect(-1, -1, -1, -1);
      }
      ;
      Rect
      getSpecialRect(const char* what, cv::Size patchSize)
      {
        return Rect(-1, -1, -1, -1);
      }
      ;
    };

    class EstimatedGaussDistribution
    {
    public:

      EstimatedGaussDistribution();
      EstimatedGaussDistribution(float P_mean, float R_mean, float P_sigma, float R_sigma);
      virtual
      ~EstimatedGaussDistribution();

      void
      update(float value); //, float timeConstant = -1.0);

      float
      getMean()
      {
        return m_mean;
      }
      ;
      float
      getSigma()
      {
        return m_sigma;
      }
      ;
      void
      setValues(float mean, float sigma);

    private:

      float m_mean;
      float m_sigma;
      float m_P_mean;
      float m_P_sigma;
      float m_R_mean;
      float m_R_sigma;
    };

    class FeatureHaar
    {

    public:

      FeatureHaar(Size patchSize);

      void
      getInitialDistribution(EstimatedGaussDistribution *distribution);

      bool
      eval(ImageRepresentation* image, Rect ROI, float* result);

      float
      getResponse()
      {
        return m_response;
      }
      ;

      int
      getNumAreas()
      {
        return m_numAreas;
      }
      ;
      const std::vector<int> &
      getWeights() const
      {
        return m_weights;
      }
      ;
      const std::vector<Rect> &
      getAreas() const
      {
        return m_areas;
      }
      ;

    private:

      int m_type;
      int m_numAreas;
      std::vector<int> m_weights;
      float m_initMean;
      float m_initSigma;

      void
      generateRandomFeature(Size imageSize);
      std::vector<Rect> m_areas; // areas within the patch over which to compute the feature
      cv::Size m_initSize; // size of the patch used during training
      cv::Size m_curSize; // size of the patches currently under investigation
      float m_scaleFactorHeight; // scaling factor in vertical direction
      float m_scaleFactorWidth; // scaling factor in horizontal direction
      std::vector<Rect> m_scaleAreas; // areas after scaling
      std::vector<float> m_scaleWeights; // weights after scaling
      float m_response;

    };

    class ClassifierThreshold
    {
    public:

      ClassifierThreshold();
      virtual
      ~ClassifierThreshold();

      void
      update(float value, int target);
      int
      eval(float value);

      void*
      getDistribution(int target);

    private:

      EstimatedGaussDistribution* m_posSamples;
      EstimatedGaussDistribution* m_negSamples;

      float m_threshold;
      int m_parity;
    };

    class WeakClassifier
    {

    public:

      WeakClassifier();
      virtual
      ~WeakClassifier();

      virtual bool
      update(ImageRepresentation* image, Rect ROI, int target);

      virtual int
      eval(ImageRepresentation* image, Rect ROI);

      virtual float
      getValue(ImageRepresentation* image, Rect ROI);

      virtual int
      getType();

    };

    class WeakClassifierHaarFeature: public WeakClassifier
    {

    public:

      WeakClassifierHaarFeature(Size patchSize);
      virtual
      ~WeakClassifierHaarFeature();

      bool
      update(ImageRepresentation* image, Rect ROI, int target);

      int
      eval(ImageRepresentation* image, Rect ROI);

      float
      getValue(ImageRepresentation* image, Rect ROI);

      int
      getType()
      {
        return 1;
      }
      ;

      EstimatedGaussDistribution*
      getPosDistribution();
      EstimatedGaussDistribution*
      getNegDistribution();

      void
      resetPosDist();
      void
      initPosDist();

    private:

      FeatureHaar* m_feature;
      ClassifierThreshold* m_classifier;

      void
      generateRandomClassifier();

    };

    class BaseClassifier
    {
    public:

      BaseClassifier(int numWeakClassifier, int iterationInit, Size patchSize);
      BaseClassifier(int numWeakClassifier, int iterationInit, WeakClassifier** weakClassifier);

      virtual
      ~BaseClassifier();

      void
      trainClassifier(ImageRepresentation* image, Rect ROI, int target, float importance, bool* errorMask);

      void
      getErrorMask(ImageRepresentation* image, Rect ROI, int target, bool* errorMask);
      void
      getErrors(float* errors);
      virtual int
      selectBestClassifier(bool* errorMask, float importance, std::vector<float> & errors);

      virtual int
      replaceWeakestClassifier(const std::vector<float> & errors, Size patchSize);
      virtual float
      getError(int curWeakClassifier = -1);

      void
      replaceClassifierStatistic(int sourceIndex, int targetIndex);

      int
      eval(ImageRepresentation* image, Rect ROI);

      float
      getValue(ImageRepresentation *image, Rect ROI, int weakClassifierIdx = -1);

      WeakClassifier**
      getReferenceWeakClassifier()
      {
        return weakClassifier;
      }
      ;
      void
      setReferenceWeakClassifier(WeakClassifier** weakClassifier)
      {
        this->weakClassifier = weakClassifier;
      }
      ;

      int
      getNumWeakClassifier()
      {
        return m_numWeakClassifier;
      }
      ;
      int
      getIterationInit()
      {
        return m_iterationInit;
      }
      ;
      float
      getWCorrect()
      {
        return m_wCorrect[m_selectedClassifier];
      }
      ;
      float
      getWWrong()
      {
        return m_wWrong[m_selectedClassifier];
      }
      ;
      void
      setWCorrect(int idx, float value)
      {
        if (idx < m_numWeakClassifier)
          m_wCorrect[idx] = value;
      }
      ;
      void
      setWWrong(int idx, float value)
      {
        if (idx < m_numWeakClassifier)
          m_wWrong[idx] = value;
      }
      ;

      int
      getTypeOfSelectedClassifier()
      {
        return weakClassifier[m_selectedClassifier]->getType();
      }
      ;
      int
      getIdxOfSelectedClassifier()
      {
        return m_selectedClassifier;
      }
      ;
      int
      getIdxOfNewWeakClassifier()
      {
        return m_idxOfNewWeakClassifier;
      }
      ;

    protected:

      WeakClassifier** weakClassifier;
      bool m_referenceWeakClassifier;
      int m_numWeakClassifier;
      int m_selectedClassifier;
      int m_idxOfNewWeakClassifier;
      std::vector<float> m_wCorrect;
      std::vector<float> m_wWrong;
      int m_iterationInit;
      void
      generateRandomClassifier(Size patchSize);

    };

    class StrongClassifier
    {
    public:

      StrongClassifier(int numBaseClassifier, int numWeakClassifier, Size patchSize, bool useFeatureExchange = false,
                       int iterationInit = 0);

      virtual
      ~StrongClassifier();

      virtual float
      eval(ImageRepresentation *image, Rect ROI);

      virtual bool
      update(ImageRepresentation *image, Rect ROI, int target, float importance = 1.0f);
      virtual bool
      updateSemi(ImageRepresentation *image, Rect ROI, float priorConfidence);

      cv::Size
      getPatchSize()
      {
        return patchSize;
      }
      ;
      int
      getNumBaseClassifier()
      {
        return numBaseClassifier;
      }
      ;
      int
      getIdxOfSelectedClassifierOfBaseClassifier(int baseClassifierIdx = 0)
      {
        return baseClassifier[baseClassifierIdx]->getIdxOfSelectedClassifier();
      }
      ;
      virtual float
      getSumAlpha(int toBaseClassifier = -1);
      float
      getAlpha(int idx)
      {
        return alpha[idx];
      }
      ;

      float
      getFeatureValue(ImageRepresentation *image, Rect ROI, int baseClassifierIdx);
      float
      getImportance(ImageRepresentation *image, Rect ROI, int traget, int numBaseClassifiers = -1);

      WeakClassifier**
      getReferenceWeakClassifier()
      {
        return baseClassifier[0]->getReferenceWeakClassifier();
      }
      ;

      void
      resetWeightDistribution();

    protected:

      int numBaseClassifier;
      int numAllWeakClassifier;

      BaseClassifier** baseClassifier;
      std::vector<float> alpha;
      cv::Size patchSize;

      bool useFeatureExchange;

    };

    class StrongClassifierDirectSelection: public StrongClassifier
    {
    public:

      StrongClassifierDirectSelection(int numBaseClassifier, int numWeakClassifier, Size patchSize,
                                      bool useFeatureExchange = false, int iterationInit = 0);

      virtual
      ~StrongClassifierDirectSelection();

      bool
      update(ImageRepresentation *image, Rect ROI, int target, float importance = 1.0);

    private:

      bool * m_errorMask;
      std::vector<float> m_errors;
      std::vector<float> m_sumErrors;
    };

    class StrongClassifierStandard: public StrongClassifier
    {
    public:

      StrongClassifierStandard(int numBaseClassifier, int numWeakClassifier, Size patchSize, bool useFeatureExchange =
          false,
                               int iterationInit = 0);

      virtual
      ~StrongClassifierStandard();

      bool
      update(ImageRepresentation *image, Rect ROI, int target, float importance = 1.0);

    private:

      bool *m_errorMask;
      std::vector<float> m_errors;
    };

    class StrongClassifierStandardSemi: public StrongClassifier
    {
    public:

      StrongClassifierStandardSemi(int numBaseClassifier, int numWeakClassifier, Size patchSize,
                                   bool useFeatureExchange = false, int iterationInit = 0);

      virtual
      ~StrongClassifierStandardSemi();

      bool
      updateSemi(ImageRepresentation *image, Rect ROI, float priorConfidence);
      void
      getPseudoValues(ImageRepresentation *image, Rect ROI, float priorConfidence, float* pseudoLambdaInOut,
                      int* pseudoTargetInOut);

    private:

      bool *m_errorMask;
      std::vector<float> m_errors;
      std::vector<float> m_pseudoLambda;
      std::vector<int> m_pseudoTarget;
    };

    class Detector
    {
    public:

      Detector(StrongClassifier* classifier);
      virtual
      ~Detector(void);

      void
      classify(ImageRepresentation* image, Patches* patches, float minMargin = 0.0f);
      void
      classify(ImageRepresentation* image, Patches* patches, float minMargin, float minVariance);

      void
      classifySmooth(ImageRepresentation* image, Patches* patches, float minMargin = 0);

      int
      getNumDetections();
      float
      getConfidence(int patchIdx);
      float
      getConfidenceOfDetection(int detectionIdx);

      float
      getConfidenceOfBestDetection()
      {
        return m_maxConfidence;
      }
      ;
      int
      getPatchIdxOfBestDetection();

      int
      getPatchIdxOfDetection(int detectionIdx);

      const std::vector<int> &
      getIdxDetections() const
      {
        return m_idxDetections;
      }
      ;
      const std::vector<float> &
      getConfidences() const
      {
        return m_confidences;
      }
      ;

      const cv::Mat &
      getConfImageDisplay() const
      {
        return m_confImageDisplay;
      }

    private:

      void
      prepareConfidencesMemory(int numPatches);
      void
      prepareDetectionsMemory(int numDetections);

      StrongClassifier* m_classifier;
      std::vector<float> m_confidences;
      int m_sizeConfidences;
      int m_numDetections;
      std::vector<int> m_idxDetections;
      int m_sizeDetections;
      int m_idxBestDetection;
      float m_maxConfidence;
      cv::Mat_<float> m_confMatrix;
      cv::Mat_<float> m_confMatrixSmooth;
      cv::Mat_<unsigned char> m_confImageDisplay;
    };

    /** The main Online Boosting tracker class */
    class BoostingTracker
    {
    public:
      BoostingTracker(ImageRepresentation* image, Rect initPatch, Rect validROI, int numBaseClassifier);
      virtual
      ~BoostingTracker();

      bool
      track(ImageRepresentation* image, Patches* patches);

      cv::Rect
      getTrackingROI(float searchFactor);
      float
      getConfidence();
      cv::Rect
      getTrackedPatch();
      cv::Point2i
      getCenter();
      const cv::Mat &
      getConfImageDisplay() const
      {
        return detector->getConfImageDisplay();
      }

    private:
      StrongClassifier* classifier;
      Detector* detector;
      Rect validROI;
      Rect trackedPatch;
      float confidence;
      cv::Point2i dxDy;
    };

    class SemiBoostingTracker
    {
    public:
      SemiBoostingTracker(ImageRepresentation* image, Rect initPatch, Rect validROI, int numBaseClassifier);

      bool
      track(ImageRepresentation* image, Patches* patches);

      Rect
      getTrackingROI(float searchFactor);
      float
      getConfidence();
      float
      getPriorConfidence();
      Rect
      getTrackedPatch();
      cv::Point2i
      getCenter();
      const cv::Mat &
      getConfImageDisplay() const
      {
        return detector->getConfImageDisplay();
      }

    private:
      cv::Ptr<StrongClassifier> classifierOff;
      cv::Ptr<StrongClassifierStandardSemi> classifier;
      cv::Ptr<Detector> detector;
      Rect trackedPatch;
      Rect validROI;
      float confidence;
      float priorConfidence;
    };

  }
}

#endif  // #ifndef __OPENCV_ONLINE_BOOSTING_H__
/* End of file. */
