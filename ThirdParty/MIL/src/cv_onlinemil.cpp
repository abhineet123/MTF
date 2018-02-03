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

#include <iomanip>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "mtf/ThirdParty/MIL/cv_onlinemil.h"

/****************************************************************************************
 COPYRIGHT NOTICE
 ----------------

 The code has been derived from MILTRACK on http://vision.ucsd.edu/~bbabenko
 By Boris Babenko
 The agreement to put it in OpenCV under the BSD license is at the end of hat file

 ****************************************************************************************/

using namespace std;

void
compute_integral(const cv::Mat & img, std::vector<cv::Mat_<float> > & ii_imgs)
{
  cv::Mat ii_img;
  cv::integral(img, ii_img, CV_32F);
  cv::split(ii_img, ii_imgs);
}

namespace mtf
{
  namespace mil
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    /** Static random number generator shared between functions
     */
    cv::RNG RandomGenerator::rng_ = cv::RNG();

    std::string
    int2str(int i, int ndigits)
    {
      std::ostringstream temp;
      temp << setfill('0') << setw(ndigits) << i;
      return temp.str();
    }

    void
    display(const cv::Mat & img, int fignum, float p)
    {
      CV_Assert(!img.empty());
      char name[1024];
      sprintf(name, "Figure %d", fignum);
      cv::namedWindow(name, CV_WINDOW_AUTOSIZE);
      cv::imshow(name, img);
    }

    Sample::Sample(const cv::Mat & img, const std::vector<cv::Mat_<float> > & ii_imgs, int row, int col, int width,
                   int height, float weight)
    {
      _img = img;
      _ii_imgs = ii_imgs;
      _row = row;
      _col = col;
      _width = width;
      _height = height;
      _weight = weight;
    }

    void
    SampleSet::sampleImage(const cv::Mat & img, const std::vector<cv::Mat_<float> > & ii_imgs, int x, int y, int w,
                           int h, float inrad, float outrad, int maxnum)
    {
      int rowsz = img.rows - h - 1;
      int colsz = img.cols - w - 1;
      float inradsq = inrad * inrad;
      float outradsq = outrad * outrad;
      int dist;

      uint minrow = max(0, (int) y - (int) inrad);
      uint maxrow = min((int) rowsz - 1, (int) y + (int) inrad);
      uint mincol = max(0, (int) x - (int) inrad);
      uint maxcol = min((int) colsz - 1, (int) x + (int) inrad);

      //fprintf(stderr,"inrad=%f minrow=%d maxrow=%d mincol=%d maxcol=%d\n",inrad,minrow,maxrow,mincol,maxcol);

      _samples.resize((maxrow - minrow + 1) * (maxcol - mincol + 1));
      int i = 0;

      float prob = ((float) (maxnum)) / _samples.size();

      for (int r = minrow; r <= (int) maxrow; r++)
        for (int c = mincol; c <= (int) maxcol; c++)
        {
          dist = (y - r) * (y - r) + (x - c) * (x - c);
          if (RandomGenerator::randfloat() < prob && dist < inradsq && dist >= outradsq)
          {
            _samples[i]._img = img;
            _samples[i]._ii_imgs = ii_imgs;
            _samples[i]._col = c;
            _samples[i]._row = r;
            _samples[i]._height = h;
            _samples[i]._width = w;
            i++;
          }
        }

      _samples.resize(min(i, maxnum));

    }

    void
    SampleSet::sampleImage(const cv::Mat & img, const std::vector<cv::Mat_<float> > & ii_imgs, uint num, int w, int h)
    {
      int rowsz = img.rows - h - 1;
      int colsz = img.cols - w - 1;

      _samples.resize(num);
      for (int i = 0; i < (int) num; i++)
      {
        _samples[i]._img = img;
        _samples[i]._ii_imgs = ii_imgs;
        _samples[i]._col = RandomGenerator::randint(0, colsz);
        _samples[i]._row = RandomGenerator::randint(0, rowsz);
        _samples[i]._height = h;
        _samples[i]._width = w;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    HaarFtrParams::HaarFtrParams()
    {
      _numCh = -1;
      for (int k = 0; k < 1024; k++)
        _useChannels[k] = -1;
      _minNumRect = 2;
      _maxNumRect = 6;
      _useChannels[0] = 0;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    HaarFtr::HaarFtr()
        :
          _maxSum(0)
    {
      _width = 0;
      _height = 0;
      _channel = 0;
    }

    void
    HaarFtr::generate(FtrParams *op)
    {
      HaarFtrParams *p = (HaarFtrParams*) op;
      _width = p->_width;
      _height = p->_height;
      int numrects = RandomGenerator::randint(p->_minNumRect, p->_maxNumRect);
      _rects.resize(numrects);
      _weights.resize(numrects);
      _rsums.resize(numrects);
      _maxSum = 0.0f;

      for (int k = 0; k < numrects; k++)
      {
        _weights[k] = RandomGenerator::randfloat(-1, 1);
        _rects[k].x = RandomGenerator::randint(0, (uint) (p->_width - 3));
        _rects[k].y = RandomGenerator::randint(0, (uint) (p->_height - 3));
        _rects[k].width = RandomGenerator::randint(1, (p->_width - _rects[k].x - 2));
        _rects[k].height = RandomGenerator::randint(1, (p->_height - _rects[k].y - 2));
        _rsums[k] = std::abs(_weights[k] * (_rects[k].width + 1) * (_rects[k].height + 1) * 255);
        //_rects[k].width = RandomGenerator::randint(1,3);
        //_rects[k].height = RandomGenerator::randint(1,3);
      }

      if (p->_numCh < 0)
      {
        p->_numCh = 0;
        for (int k = 0; k < 1024; k++)
          p->_numCh += p->_useChannels[k] >= 0;
      }

      _channel = p->_useChannels[RandomGenerator::randint(0, p->_numCh - 1)];
    }

    cv::Mat
    HaarFtr::toViz()
    {
      cv::Mat_<cv::Vec3b> v = cv::Mat_<cv::Vec3b>::zeros(_height, _width);

      for (uint k = 0; k < _rects.size(); k++)
      {
        if (_weights[k] < 0)
          cv::rectangle(v, cv::Point2i(_rects[k].x, _rects[k].y),
                        cv::Point2i(_rects[k].x + _rects[k].width, _rects[k].y + _rects[k].height),
                        cv::Scalar((255 * std::max<double>(-1 * _weights[k], 0.5)), 0, 0), 1);
        else
          cv::rectangle(
              v, cv::Point2i(_rects[k].x, _rects[k].y),
              cv::Point2i(_rects[k].x + _rects[k].width, _rects[k].y + _rects[k].height),
              cv::Scalar(0, 255 * std::max<double>(_weights[k], 0.5), 255 * std::max<double>(_weights[k], 0.5)), 1);
      }

      return v;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
    Ftr::compute(SampleSet &samples, const vecFtr &ftrs)
    {
      int numftrs = ftrs.size();
      int numsamples = samples.size();
      if (numsamples == 0)
        return;

      samples.resizeFtrs(numftrs);

#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int ftr = 0; ftr < numftrs; ftr++)
      {
        for (int k = 0; k < numsamples; k++)
        {
          samples.getFtrVal(k, ftr) = ftrs[ftr]->compute(samples[k]);
        }
      }

    }
    void
    Ftr::compute(SampleSet &samples, Ftr *ftr, int ftrind)
    {

      int numsamples = samples.size();

#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int k = 0; k < numsamples; k++)
      {
        samples.getFtrVal(k, ftrind) = ftr->compute(samples[k]);
      }

    }
    vecFtr
    Ftr::generate(FtrParams *params, uint num)
    {
      vecFtr ftrs;

      ftrs.resize(num);
      for (uint k = 0; k < num; k++)
      {
        switch (params->ftrType())
        {
          case 0:
            ftrs[k] = new HaarFtr();
            break;
        }
        ftrs[k]->generate(params);
      }

      // DEBUG
      if (0)
        Ftr::toViz(ftrs, "ftrs");

      return ftrs;
    }

    void
    Ftr::deleteFtrs(vecFtr ftrs)
    {
      for (uint k = 0; k < ftrs.size(); k++)
        delete ftrs[k];
    }
    void
    Ftr::toViz(vecFtr &ftrs, const char *dirname)
    {
      char fname[1024];
      cv::Mat img;
      for (uint k = 0; k < ftrs.size(); k++)
      {
        sprintf(fname, "%s/ftr%05d.png", dirname, k);
        img = ftrs[k]->toViz();
        cv::imwrite(fname, img);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ClfStrong*
    ClfStrong::makeClf(ClfStrongParams *clfparams)
    {
      ClfStrong* clf;

      switch (clfparams->clfType())
      {
        case 0:
          clf = new ClfAdaBoost();
          break;
        case 1:
          clf = new ClfMilBoost();
          break;

        default:
          abortError(__LINE__, __FILE__, "Incorrect clf type!");
      }

      clf->init(clfparams);
      return clf;
    }

    cv::Mat_<float>
    ClfStrong::applyToImage(ClfStrong *clf, const cv::Mat & img, bool logR)
    {
      std::vector<cv::Mat_<float> > ii_imgs;
      compute_integral(img, ii_imgs);
      cv::Mat_<float> resp(img.rows, img.cols);
      int height = clf->_params->_ftrParams->_height;
      int width = clf->_params->_ftrParams->_width;

      //int rowsz = img.rows() - width - 1;
      //int colsz = img.cols() - height - 1;

      SampleSet x;
      x.sampleImage(img, ii_imgs, 0, 0, width, height, 100000); // sample every point
      Ftr::compute(x, clf->_ftrs);
      vectorf rf = clf->classify(x, logR);
      for (int i = 0; i < x.size(); i++)
        resp(x[i]._row, x[i]._col) = rf[i];

      return resp;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ClfWeak::ClfWeak()
    {
      _trained = false;
      _ind = -1;
    }

    ClfWeak::ClfWeak(int id)
    {
      _trained = false;
      _ind = id;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
    ClfOnlineStump::init()
    {
      _mu0 = 0;
      _mu1 = 0;
      _sig0 = 1;
      _sig1 = 1;
      _lRate = 0.85f;
      _trained = false;
    }

    void
    ClfWStump::init()
    {
      _mu0 = 0;
      _mu1 = 0;
      _sig0 = 1;
      _sig1 = 1;
      _lRate = 0.85f;
      _trained = false;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
    ClfAdaBoost::init(ClfStrongParams *params)
    {
      // initialize model
      _params = params;
      _myParams = (ClfAdaBoostParams*) params;
      _numsamples = 0;

      if (_myParams->_numSel > _myParams->_numFeat || _myParams->_numSel < 1)
        _myParams->_numSel = _myParams->_numFeat / 2;

      //_countFP.Resize(_params._numSel,_params._numFeat,1);
      //_countFN.Resize(_params._numSel,_params._numFeat,1);
      //_countTP.Resize(_params._numSel,_params._numFeat,1);
      //_countTN.Resize(_params._numSel,_params._numFeat,1);
      resizeVec(_countFPv, _myParams->_numSel, _myParams->_numFeat, 1.0f);
      resizeVec(_countTPv, _myParams->_numSel, _myParams->_numFeat, 1.0f);
      resizeVec(_countFNv, _myParams->_numSel, _myParams->_numFeat, 1.0f);
      resizeVec(_countTNv, _myParams->_numSel, _myParams->_numFeat, 1.0f);

      _alphas.resize(_myParams->_numSel, 0);
      _ftrs = Ftr::generate(_myParams->_ftrParams, _myParams->_numFeat);
      _selectors.resize(_myParams->_numSel, 0);
      _weakclf.resize(_myParams->_numFeat);
      for (int k = 0; k < _myParams->_numFeat; k++)
        //if (_params._weakLearner == string("kalman"))
        //	_weakclf[k] = new ClfKalmanStump();
        //else
        if (_myParams->_weakLearner == std::string("stump"))
        {
          _weakclf[k] = new ClfOnlineStump(k);
          _weakclf[k]->_ftr = _ftrs[k];
          _weakclf[k]->_lRate = _myParams->_lRate;
          _weakclf[k]->_parent = this;
        }
        else if (_myParams->_weakLearner == std::string("wstump"))
        {
          _weakclf[k] = new ClfWStump(k);
          _weakclf[k]->_ftr = _ftrs[k];
          _weakclf[k]->_lRate = _myParams->_lRate;
          _weakclf[k]->_parent = this;
        }
        else
          abortError(__LINE__, __FILE__, "incorrect weak clf name");
    }
    void
    ClfAdaBoost::update(SampleSet &posx, SampleSet &negx)
    {
      int numpts = posx.size() + negx.size();

      // compute ftrs
      if (!posx.ftrsComputed())
        Ftr::compute(posx, _ftrs);
      if (!negx.ftrsComputed())
        Ftr::compute(negx, _ftrs);

      //vectorf poslam(posx[0].size(),.5f*numpts/posx[0].size()), neglam(negx[0].size(),.5f*numpts/negx[0].size());
      //vectorf poslam(posx[0].size(),1), neglam(negx[0].size(),1);
      vectorf poslam(posx.size(), .5f / posx.size()), neglam(negx.size(), .5f / negx.size());
      vector<vectorb> pospred(nFtrs()), negpred(nFtrs());
      vectorf errs(nFtrs());
      vectori order(nFtrs());

      _sumAlph = 0.0f;
      _selectors.clear();

      // update all weak classifiers and get predicted labels
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int k = 0; k < nFtrs(); k++)
      {
        _weakclf[k]->update(posx, negx);
        pospred[k] = _weakclf[k]->classifySet(posx);
        negpred[k] = _weakclf[k]->classifySet(negx);
      }

      vectori worstinds;

      // loop over selectors
      for (int t = 0; t < _myParams->_numSel; t++)
      {
        // calculate errors for selector t
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int k = 0; k < _myParams->_numFeat; k++)
        {
          for (int j = 0; j < (int) poslam.size(); j++)
          {
            //if( poslam[j] > 1e-5 )
            (pospred[k][j]) ? _countTPv[t][k] += poslam[j] : _countFNv[t][k] += poslam[j];
          }
        }
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int k = 0; k < _myParams->_numFeat; k++)
        {
          for (int j = 0; j < (int) neglam.size(); j++)
          {
            //if( neglam[j] > 1e-5 )
            (!negpred[k][j]) ? _countTNv[t][k] += neglam[j] : _countFPv[t][k] += neglam[j];
          }
        }
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int k = 0; k < _myParams->_numFeat; k++)
        {
          //float fp,fn;
          //fp = _countFPv[t][k] / (_countFPv[t][k] + _countTNv[t][k]);
          //fn = _countFNv[t][k] / (_countFNv[t][k] + _countTPv[t][k]);
          //errs[k] = 0.3f*fp + 0.7f*fn;
          errs[k] = (_countFPv[t][k] + _countFNv[t][k])
              / (_countFPv[t][k] + _countFNv[t][k] + _countTPv[t][k] + _countTNv[t][k]);
        }

        // pick the best weak clf and udpate _selectors and _selectedFtrs
        float minerr;
        uint bestind;

        sort_order_des(errs, order);

        // find best in that isn't already included
        for (uint k = 0; k < order.size(); k++)
          if (count(_selectors.begin(), _selectors.end(), order[k]) == 0)
          {
            _selectors.push_back(order[k]);
            minerr = errs[k];
            bestind = order[k];
            break;
          }

        //cout << "min err=" << minerr << endl;

        // find worst ind
        worstinds.push_back(order[order.size() - 1]);

        // update alpha
        _alphas[t] = std::max<float>(0, std::min<float>(0.5f * log((1 - minerr) / (minerr + 0.00001f)), 10));
        _sumAlph += _alphas[t];

        // update weights
        float corw = 1 / (2 - 2 * minerr);
        float incorw = 1 / (2 * minerr);
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int j = 0; j < (int) poslam.size(); j++)
          poslam[j] *= (pospred[bestind][j] == 1) ? corw : incorw;
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int j = 0; j < (int) neglam.size(); j++)
          neglam[j] *= (negpred[bestind][j] == 0) ? corw : incorw;

      }

      _numsamples += numpts;

      return;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
    ClfMilBoost::init(ClfStrongParams *params)
    {
      // initialize model
      _params = params;
      _myParams = (ClfMilBoostParams*) params;
      _numsamples = 0;

      _ftrs = Ftr::generate(_myParams->_ftrParams, _myParams->_numFeat);
      if (params->_storeFtrHistory)
        Ftr::toViz(_ftrs, "haarftrs");
      _weakclf.resize(_myParams->_numFeat);
      for (int k = 0; k < _myParams->_numFeat; k++)
        if (_myParams->_weakLearner == std::string("stump"))
        {
          _weakclf[k] = new ClfOnlineStump(k);
          _weakclf[k]->_ftr = _ftrs[k];
          _weakclf[k]->_lRate = _myParams->_lRate;
          _weakclf[k]->_parent = this;
        }
        else if (_myParams->_weakLearner == std::string("wstump"))
        {
          _weakclf[k] = new ClfWStump(k);
          _weakclf[k]->_ftr = _ftrs[k];
          _weakclf[k]->_lRate = _myParams->_lRate;
          _weakclf[k]->_parent = this;
        }
        else
          abortError(__LINE__, __FILE__, "incorrect weak clf name");

      if (params->_storeFtrHistory)
        this->_ftrHist.create(_myParams->_numFeat, 2000);

      _counter = 0;
    }
    void
    ClfMilBoost::update(SampleSet &posx, SampleSet &negx)
    {
      int numneg = negx.size();
      int numpos = posx.size();

      // compute ftrs
      if (!posx.ftrsComputed())
        Ftr::compute(posx, _ftrs);
      if (!negx.ftrsComputed())
        Ftr::compute(negx, _ftrs);

      // initialize H
      static vectorf Hpos, Hneg;
      Hpos.clear();
      Hneg.clear();
      Hpos.resize(posx.size(), 0.0f), Hneg.resize(negx.size(), 0.0f);

      _selectors.clear();
      vectorf posw(posx.size()), negw(negx.size());
      vector<vectorf> pospred(_weakclf.size()), negpred(_weakclf.size());

      // train all weak classifiers without weights
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int m = 0; m < _myParams->_numFeat; m++)
      {
        _weakclf[m]->update(posx, negx);
        pospred[m] = _weakclf[m]->classifySetF(posx);
        negpred[m] = _weakclf[m]->classifySetF(negx);
      }

      // pick the best features
      for (int s = 0; s < _myParams->_numSel; s++)
      {

        // compute errors/likl for all weak clfs
        vectorf poslikl(_weakclf.size(), 1.0f), neglikl(_weakclf.size()), likl(_weakclf.size());
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int w = 0; w < (int) _weakclf.size(); w++)
        {
          float lll = 1.0f;
          for (int j = 0; j < numpos; j++)
            lll *= (1 - sigmoid(Hpos[j] + pospred[w][j]));
          poslikl[w] = (float) -log(1 - lll + 1e-5);

          lll = 0.0f;
          for (int j = 0; j < numneg; j++)
            lll += (float) -log(1e-5f + 1 - sigmoid(Hneg[j] + negpred[w][j]));
          neglikl[w] = lll;

          likl[w] = poslikl[w] / numpos + neglikl[w] / numneg;
        }

        // pick best weak clf
        vectori order;
        sort_order_des(likl, order);

        // find best weakclf that isn't already included
        for (uint k = 0; k < order.size(); k++)
          if (count(_selectors.begin(), _selectors.end(), order[k]) == 0)
          {
            _selectors.push_back(order[k]);
            break;
          }

        // update H = H + h_m
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int k = 0; k < posx.size(); k++)
          Hpos[k] += pospred[_selectors[s]][k];
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int k = 0; k < negx.size(); k++)
          Hneg[k] += negpred[_selectors[s]][k];

      }

      if (_myParams->_storeFtrHistory)
        for (uint j = 0; j < _selectors.size(); j++)
          _ftrHist(_selectors[j], _counter) = 1.0f / (j + 1);

      _counter++;

      return;
    }

    bool
    SimpleTracker::init(const cv::Mat & frame, const SimpleTrackerParams p, ClfStrongParams *clfparams)
    {
      static cv::Mat img;

      img = frame;
      std::vector<cv::Mat_<float> > ii_imgs;
      compute_integral(img, ii_imgs);

      _clf = ClfStrong::makeClf(clfparams);
      _curState.resize(4);
      for (int i = 0; i < 4; i++)
        _curState[i] = p._initstate[i];
      SampleSet posx, negx;

      fprintf(stderr, "Initializing Tracker..\n");

      // sample positives and negatives from first frame
      posx.sampleImage(img, ii_imgs, (uint) _curState[0], (uint) _curState[1], (uint) _curState[2], (uint) _curState[3],
                       p._init_postrainrad);
      negx.sampleImage(img, ii_imgs, (uint) _curState[0], (uint) _curState[1], (uint) _curState[2], (uint) _curState[3],
                       2.0f * p._srchwinsz, (1.5f * p._init_postrainrad), p._init_negnumtrain);
      if (posx.size() < 1 || negx.size() < 1)
        return false;

      // train
      _clf->update(posx, negx);
      negx.clear();

      _trparams = p;
      _clfparams = clfparams;
      _cnt = 0;
      return true;
    }

    double
    SimpleTracker::track_frame(const cv::Mat & frame)
    {
      static SampleSet posx, negx, detectx;
      static vectorf prob;
      static vectori order;
      static cv::Mat img;

      double resp;

      img = frame;
      std::vector<cv::Mat_<float> > ii_imgs;
      compute_integral(img, ii_imgs);

      // run current clf on search window
      detectx.sampleImage(img, ii_imgs, (uint) _curState[0], (uint) _curState[1], (uint) _curState[2],
                          (uint) _curState[3], (float) _trparams._srchwinsz);
      prob = _clf->classify(detectx, _trparams._useLogR);

      /////// DEBUG /////// display actual probability map
      if (_trparams._debugv)
      {
        cv::Mat_<float> probimg(frame.rows, frame.cols);
        for (uint k = 0; k < (uint) detectx.size(); k++)
          probimg(detectx[k]._row, detectx[k]._col) = prob[k];

        display(probimg, 2, 2);
        cv::waitKey(1);
      }

      // find best location
      int bestind = max_idx(prob);
      resp = prob[bestind];

      _curState[1] = (float) detectx[bestind]._row;
      _curState[0] = (float) detectx[bestind]._col;

      // train location clf (negx are randomly selected from image, posx is just the current tracker location)

      if (_trparams._negsamplestrat == 0)
        negx.sampleImage(img, ii_imgs, _trparams._negnumtrain, (int) _curState[2], (int) _curState[3]);
      else
        negx.sampleImage(img, ii_imgs, (int) _curState[0], (int) _curState[1], (int) _curState[2], (int) _curState[3],
                         (1.5f * _trparams._srchwinsz), _trparams._posradtrain + 5, _trparams._negnumtrain);

      if (_trparams._posradtrain == 1)
        posx.push_back(img, ii_imgs, (int) _curState[0], (int) _curState[1], (int) _curState[2], (int) _curState[3]);
      else
        posx.sampleImage(img, ii_imgs, (int) _curState[0], (int) _curState[1], (int) _curState[2], (int) _curState[3],
                         _trparams._posradtrain, 0, _trparams._posmaxtrain);

      _clf->update(posx, negx);

      // clean up
      posx.clear();
      negx.clear();
      detectx.clear();

      _cnt++;

      return resp;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    TrackerParams::TrackerParams()
    {
      _boxcolor.resize(3);
      _boxcolor[0] = 204;
      _boxcolor[1] = 25;
      _boxcolor[2] = 204;
      _lineWidth = 2;
      _negnumtrain = 15;
      _posradtrain = 1;
      _posmaxtrain = 100000;
      _init_negnumtrain = 1000;
      _init_postrainrad = 3;
      _initstate.resize(4);
      _debugv = false;
      _useLogR = true;
      _disp = true;
      _initWithFace = true;
      _vidsave = "";
      _trsave = "";
    }

    SimpleTrackerParams::SimpleTrackerParams()
    {
      _srchwinsz = 30;
      _initstate.resize(4);
      _negsamplestrat = 1;
    }

  } // namespace mil
} // namespace cv

/* Original email that allows OpenCV to publish that code under BSD


 Delivered-To: vincent.rabaud@gmail.com
 Received: by 10.194.18.73 with SMTP id u9csp110377wjd;
 Fri, 20 Jul 2012 11:32:17 -0700 (PDT)
 Return-Path: <bbabenko@gmail.com>
 Received-SPF: pass (google.com: domain of bbabenko@gmail.com designates 10.112.36.163 as permitted sender) client-ip=10.112.36.163
 Authentication-Results: mr.google.com; spf=pass (google.com: domain of bbabenko@gmail.com designates 10.112.36.163 as permitted sender) smtp.mail=bbabenko@gmail.com; dkim=pass header.i=bbabenko@gmail.com
 Received: from mr.google.com ([10.112.36.163])
 by 10.112.36.163 with SMTP id r3mr3455011lbj.87.1342809136469 (num_hops = 1);
 Fri, 20 Jul 2012 11:32:16 -0700 (PDT)
 DKIM-Signature: v=1; a=rsa-sha256; c=relaxed/relaxed;
 d=gmail.com; s=20120113;
 h=mime-version:date:message-id:subject:from:to:content-type;
 bh=zAyfVf0vZ7CADoyo71OCowiEefaRanjmlFaj9sIFrtY=;
 b=WoLFJ1/l4SRUE5i2Y9BdhaNtsrOiGyX6dbLfJpQgSKowbnVujbqYeElQJ0Kcph4MaT
 tJ90x+cr2vYrOnIj/gOq2Y1zM8+1NZWARF25PD+MQSD1B7HfhN5pPB//f6qg+0AonMfh
 7hiwn5ulZLFd8RQLoEyohHXy+0/HVqKf8Pmnbl6ilRbRu84tgjZac1zY6H+NXeJTJhla
 QhiIYHpC5v7LOG6L44+V0CqOvK6+DtuxS9MVIFYG65RRwP3AMZmy4UwNN3rBOhQI48H/
 HcuAxou8O4FVt5tKFFjaMJRYTSx5+TOTasDtQRuRLko7RL7F8dRzvCNAFHI/lGN1IGqK
 baag==
 MIME-Version: 1.0
 Received: by 10.112.36.163 with SMTP id r3mr3455011lbj.87.1342809136462; Fri,
 20 Jul 2012 11:32:16 -0700 (PDT)
 Received: by 10.152.1.42 with HTTP; Fri, 20 Jul 2012 11:32:16 -0700 (PDT)
 Date: Fri, 20 Jul 2012 11:32:16 -0700
 Message-ID: <CAD6p0qPmWH02-1w1zYkvLXf=kZCzHQkBY9OhK_FEjhtux2ayMw@mail.gmail.com>
 Subject: miltrack code
 From: Boris Babenko <bbabenko@gmail.com>
 To: Vincent Rabaud <vincent.rabaud@gmail.com>
 Content-Type: multipart/alternative; boundary=485b390f79fe947ab904c5471d05

 --485b390f79fe947ab904c5471d05
 Content-Type: text/plain; charset=ISO-8859-1

 Hi Vincent,

 You (or anyone at WillowGarage) have my permission to change the license of
 my MilTrack code to BSD as you see fit.

 Thanks,
 - Boris

 --485b390f79fe947ab904c5471d05
 Content-Type: text/html; charset=ISO-8859-1

 Hi Vincent,<div><br></div><div>You (or anyone at WillowGarage) have my permission to change the license of my MilTrack code to BSD as you see fit.</div><div><br></div><div>Thanks,</div><div>- Boris</div>

 --485b390f79fe947ab904c5471d05--

 */
