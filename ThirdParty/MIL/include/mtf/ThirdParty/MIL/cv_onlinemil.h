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

#ifndef MTF_ONLINE_MIL_H
#define MTF_ONLINE_MIL_H

#include <fstream>
#include <iostream>
#include <limits>
#include <math.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

namespace cv
{
  namespace mil
  {
    typedef unsigned int uint;

    typedef std::vector<float> vectorf;
    typedef std::vector<int> vectori;
    typedef std::vector<bool> vectorb;

	template<typename ScalarT>int sign(ScalarT s){
		return ((s > 0) ? 1 : ((s < 0) ? -1 : 0));
	}

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    // random generator stuff
    struct RandomGenerator
    {
    public:
      static
      void
      initialize(const int init)
      {
        rng_ = cv::RNG(init);
      }

      static
      int
      randint(const int min, const int max)
      {
        return rng_.uniform(min, max);
      }

      static
      float
      randfloat(const float min = 0, const float max = 1)
      {
        return rng_.uniform(min, max);
      }
      static cv::RNG rng_;
    };

    inline float
    sigmoid(float x)
    {
      return 1.0f / (1.0f + exp(-x));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    // vector functions
    template<class T> class SortableElement
    {
    public:
      T _val;
      int _ind;
      SortableElement()
          :
            _ind(0)
      {
      }
      SortableElement(T val, int ind)
      {
        _val = val;
        _ind = ind;
      }
      bool
      operator<(SortableElement &b)
      {
        return (_val > b._val);
      }
      ;
    };

    template<class T> class SortableElementRev
    {
    public:
      T _val;
      int _ind;
      SortableElementRev()
          :
            _ind(0)
      {
      }
      SortableElementRev(T val, int ind)
      {
        _val = val;
        _ind = ind;
      }
      bool
      operator<(SortableElementRev<T> &b)
      {
        return (_val < b._val);
      }
      ;
    };

    static bool
    CompareSortableElementRev(const SortableElementRev<float>& i, const SortableElementRev<float>& j)
    {
      return i._val < j._val;
    }

    template<class T> void
    sort_order_des(std::vector<T> &v, vectori &order)
    {
      uint n = (uint) v.size();
      std::vector<SortableElementRev<T> > v2;
      v2.resize(n);
      order.clear();
      order.resize(n);
      for (uint i = 0; i < n; i++)
      {
        v2[i]._ind = i;
        v2[i]._val = v[i];
      }
      //std::sort( v2.begin(), v2.end() );
      std::sort(v2.begin(), v2.end(), CompareSortableElementRev);
      for (uint i = 0; i < n; i++)
      {
        order[i] = v2[i]._ind;
        v[i] = v2[i]._val;
      }
    }
    ;

    template<class T> void
    resizeVec(std::vector<std::vector<T> > &v, int sz1, int sz2, T val = 0)
    {
      v.resize(sz1);
      for (int k = 0; k < sz1; k++)
        v[k].resize(sz2, val);
    }
    ;

    ////template<class T> inline uint		min_idx( const vector<T> &v )
    ////{
    ////  return (uint)(min_element(v.begin(),v.end())._Myptr-v.begin()._Myptr);
    ////}
    template<class T> inline uint
    max_idx(const std::vector<T> &v)
    {
      const T* findPtr = &(*std::max_element(v.begin(), v.end()));
      const T* beginPtr = &(*v.begin());
      return (uint) (findPtr - beginPtr);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    // error functions
    inline void
    abortError(const int line, const char *file, const char *msg = NULL)
    {
      if (msg == NULL)
        fprintf(stderr, "%s %d: ERROR\n", file, line);
      else
        fprintf(stderr, "%s %d: ERROR: %s\n", file, line, msg);
      exit(0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////

    class Sample
    {
    public:
      Sample(const cv::Mat & img, const std::vector<cv::Mat_<float> > & ii_imgs, int row, int col, int width = 0,
             int height = 0, float weight = 1.0);
      Sample()
      {
        _row = _col = _height = _width = 0;
        _weight = 1.0f;
      }
      Sample&
      operator=(const Sample &a);

    public:
      cv::Mat _img;
      std::vector<cv::Mat_<float> > _ii_imgs;
      int _row, _col, _width, _height;
      float _weight;

    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    class SampleSet
    {
    public:
      SampleSet()
      {
      }
      ;
      SampleSet(const Sample &s)
      {
        _samples.push_back(s);
      }
      ;

      int
      size() const
      {
        return _samples.size();
      }
      ;
      void
      push_back(const Sample &s)
      {
        _samples.push_back(s);
      }
      ;
      void
      push_back(const cv::Mat & img, const std::vector<cv::Mat_<float> > & ii_img, int x, int y, int width = 0,
                int height = 0, float weight = 1.0f);
      void
      resize(int i)
      {
        _samples.resize(i);
      }
      ;
      void
      resizeFtrs(int i);
      float &
      getFtrVal(int sample, int ftr)
      {
        return _ftrVals[ftr](sample);
      }
      ;
      float
      getFtrVal(int sample, int ftr) const
      {
        return _ftrVals[ftr](sample);
      }
      ;
      Sample &
      operator[](const int sample)
      {
        return _samples[sample];
      }
      ;
      Sample
      operator[](const int sample) const
      {
        return _samples[sample];
      }
      ;
      const cv::Mat_<float> &
      ftrVals(int ftr) const
      {
        return _ftrVals[ftr];
      }
      bool
      ftrsComputed() const
      {
        return !_ftrVals.empty() && !_samples.empty() && !_ftrVals[0].empty();
      }
      ;
      void
      clear()
      {
        _ftrVals.clear();
        _samples.clear();
      }
      ;

      // densely sample the image in a donut shaped region: will take points inside circle of radius inrad,
      // but outside of the circle of radius outrad.  when outrad=0 (default), then just samples points inside a circle
      void
      sampleImage(const cv::Mat & img, const std::vector<cv::Mat_<float> > & ii_imgs, int x, int y, int w, int h,
                  float inrad, float outrad = 0, int maxnum = 1000000);
      void
      sampleImage(const cv::Mat & img, const std::vector<cv::Mat_<float> > & ii_imgs, uint num, int w, int h);

    private:
      std::vector<Sample> _samples;
      std::vector<cv::Mat_<float> > _ftrVals; // [ftr][sample]
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    inline Sample&
    Sample::operator=(const Sample &a)
    {
      _img = a._img;
      _row = a._row;
      _col = a._col;
      _width = a._width;
      _height = a._height;

      return (*this);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void
    SampleSet::resizeFtrs(int nftr)
    {
      _ftrVals.resize(nftr);
      int nsamp = _samples.size();

      if (nsamp > 0)
        for (int k = 0; k < nftr; k++)
          _ftrVals[k].create(1, nsamp);
    }

    inline void
    SampleSet::push_back(const cv::Mat & img, const std::vector<cv::Mat_<float> > & ii_imgs, int x, int y, int width,
                         int height, float weight)
    {
      Sample s(img, ii_imgs, y, x, width, height, weight);
      push_back(s);
    }

    class Ftr;
    typedef std::vector<Ftr*> vecFtr;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    class FtrParams
    {
    public:
      uint _width, _height;

    public:
      virtual int
      ftrType()=0;
      virtual
      ~FtrParams()
      {
      }
    };

    class HaarFtrParams: public FtrParams
    {
    public:
      HaarFtrParams();
      uint _maxNumRect, _minNumRect;
      int _useChannels[1024];
      int _numCh;

    public:
      virtual int
      ftrType()
      {
        return 0;
      }
      ;
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    class Ftr
    {
    public:
      uint _width, _height;
      virtual
      ~Ftr()
      {
      }
      virtual float
      compute(const Sample &sample) const =0;
      virtual void
      generate(FtrParams *params) = 0;
      virtual cv::Mat
      toViz()
      {
        return cv::Mat();
      }
      virtual bool
      update(const SampleSet &posx, const SampleSet &negx, const cv::Mat_<float> &posw, const cv::Mat_<float> &negw)
      {
        return false;
      }
      static void
      compute(SampleSet &samples, const vecFtr &ftrs);
      static void
      compute(SampleSet &samples, Ftr *ftr, int ftrind);
      static vecFtr
      generate(FtrParams *params, uint num);
      static void
      deleteFtrs(vecFtr ftrs);
      static void
      toViz(vecFtr &ftrs, const char *dirname);

      virtual int
      ftrType()=0;
    };

    class HaarFtr: public Ftr
    {
    public:
      uint _channel;
      vectorf _weights;
      std::vector<cv::Rect> _rects;
      vectorf _rsums;
      double _maxSum;

    public:
      //HaarFtr( HaarFtrParams &params );
      HaarFtr();

      HaarFtr&
      operator=(const HaarFtr &a);

      float
      expectedValue() const;

      virtual float
      compute(const Sample &sample) const;
      virtual void
      generate(FtrParams *params);
      virtual cv::Mat
      toViz();
      virtual int
      ftrType()
      {
        return 0;
      }
      ;

    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    inline float
    HaarFtr::compute(const Sample &sample) const
    {
      if (sample._ii_imgs.empty())
        abortError(__LINE__, __FILE__, "Integral image not initialized before called compute()");
      cv::Rect r;
      float sum = 0.0f;

      for (int k = 0; k < (int) _rects.size(); k++)
      {
        r = _rects[k];
        r.x += sample._col;
        r.y += sample._row;
        sum +=
            _weights[k] * (sample._ii_imgs[_channel](r.y + r.height, r.x + r.width)
                + sample._ii_imgs[_channel](r.y, r.x)
                           - sample._ii_imgs[_channel](r.y + r.height, r.x)
                           - sample._ii_imgs[_channel](r.y, r.x + r.width)); ///_rsums[k];
      }

      r.x = sample._col;
      r.y = sample._row;
      r.width = (int) sample._weight;
      r.height = (int) sample._height;

      return (float) (sum);
      //return (float) (100*sum/sample._img->sumRect(r,_channel));
    }

    inline HaarFtr&
    HaarFtr::operator=(const HaarFtr &a)
    {
      _width = a._width;
      _height = a._height;
      _channel = a._channel;
      _weights = a._weights;
      _rects = a._rects;
      _maxSum = a._maxSum;

      return (*this);
    }

    inline float
    HaarFtr::expectedValue() const
    {
      float sum = 0.0f;
      for (int k = 0; k < (int) _rects.size(); k++)
      {
        sum += _weights[k] * _rects[k].height * _rects[k].width * 125;
      }
      return sum;
    }

    class ClfWeak;
    class ClfStrong;
    class ClfAdaBoost;
    class ClfMilBoost;

    class ClfStrongParams
    {
    public:
      ClfStrongParams()
          :
            _ftrParams(0),
            _weakLearner("stump"),
            _lRate(0.85f),
            _storeFtrHistory(false)
      {
      }
      virtual
      ~ClfStrongParams()
      {
      }
      virtual int
      clfType()=0; // [0] Online AdaBoost (Oza/Grabner) [1] Online StochBoost_LR [2] Online StochBoost_MIL
    public:
      FtrParams *_ftrParams;
      std::string _weakLearner; // "stump" or "wstump"; current code only uses "stump"
      float _lRate; // learning rate for weak learners;
      bool _storeFtrHistory;
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    class ClfStrong
    {
    public:
      ClfStrongParams *_params;
      vecFtr _ftrs;
      vecFtr _selectedFtrs;
      cv::Mat_<float> _ftrHist;
      uint _counter;

    public:
      virtual
      ~ClfStrong()
      {
      }
      int
      nFtrs()
      {
        return _ftrs.size();
      }
      ;

      // abstract functions
      virtual void
      init(ClfStrongParams *params)=0;
      virtual void
      update(SampleSet &posx, SampleSet &negx)=0;
      virtual vectorf
      classify(SampleSet &x, bool logR = true)=0;

      static ClfStrong*
      makeClf(ClfStrongParams *clfparams);
      static cv::Mat_<float>
      applyToImage(ClfStrong *clf, const cv::Mat & img, bool logR = true); // returns a probability map (or log odds ratio map if logR=true)

      static void
      eval(vectorf ppos, vectorf pneg, float &err, float &fp, float &fn, float thresh = 0.5f);
      static float
      likl(vectorf ppos, vectorf pneg);
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // IMEPLEMENTATIONS - PARAMS

    class ClfAdaBoostParams: public ClfStrongParams
    {
    public:
      int _numSel, _numFeat;

    public:
      ClfAdaBoostParams()
      {
        _numSel = 50;
        _numFeat = 250;
      }
      ;
      virtual int
      clfType()
      {
        return 0;
      }
      ;
    };

    class ClfMilBoostParams: public ClfStrongParams
    {
    public:
      int _numFeat, _numSel;

    public:
      ClfMilBoostParams()
      {
        _numSel = 50;
        _numFeat = 250;
      }
      ;
      virtual int
      clfType()
      {
        return 1;
      }
      ;
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // IMEPLEMENTATIONS - CLF

    class ClfAdaBoost: public ClfStrong
    {
    private:
      vectorf _alphas;
      vectori _selectors;
      std::vector<ClfWeak*> _weakclf;
      uint _numsamples;
      float _sumAlph;
      std::vector<vectorf> _countFPv, _countFNv, _countTPv, _countTNv; //[selector][feature]
      ClfAdaBoostParams *_myParams;
    public:
      ClfAdaBoost()
          :
            _sumAlph(0),
            _myParams(0),
            _numsamples(0)
      {
      }
      virtual void
      init(ClfStrongParams *params);
      virtual void
      update(SampleSet &posx, SampleSet &negx);
      virtual vectorf
      classify(SampleSet &x, bool logR = true);
    };

    class ClfMilBoost: public ClfStrong
    {
    private:
      vectori _selectors;
      std::vector<ClfWeak*> _weakclf;
      uint _numsamples;
      ClfMilBoostParams *_myParams;

    public:
      ClfMilBoost()
          :
            _numsamples(0),
            _myParams(0)
      {
      }
      virtual void
      init(ClfStrongParams *params);
      virtual void
      update(SampleSet &posx, SampleSet &negx);
      virtual vectorf
      classify(SampleSet &x, bool logR = true);

    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // WEAK CLF

    class ClfWeak
    {
    public:
      ClfWeak();
      ClfWeak(int id);
      virtual
      ~ClfWeak()
      {
      }

      virtual void
      init()=0;
      virtual void
      update(SampleSet &posx, SampleSet &negx, const cv::Mat_<float> & posw = cv::Mat_<float>(),
             const cv::Mat_<float> & negw = cv::Mat_<float>())=0;
      virtual bool
      classify(SampleSet &x, int i)=0;
      virtual float
      classifyF(SampleSet &x, int i)=0;
      virtual void
      copy(const ClfWeak* c)=0;

      virtual vectorb
      classifySet(SampleSet &x);
      virtual vectorf
      classifySetF(SampleSet &x);

      float
      ftrcompute(const Sample &x)
      {
        return _ftr->compute(x);
      }
      ;
      float
      getFtrVal(const SampleSet &x, int i)
      {
        return (x.ftrsComputed()) ? x.getFtrVal(i, _ind) : _ftr->compute(x[i]);
      }
      ;

    protected:
      bool _trained;
      Ftr *_ftr;
      vecFtr *_ftrs;
      int _ind;
      float _lRate;
      ClfStrong *_parent;

      friend class ClfAdaBoost;
      friend class ClfMilBoost;
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    class ClfOnlineStump: public ClfWeak
    {
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      // members
      float _mu0, _mu1, _sig0, _sig1;
      float _q;
      int _s;
      float _log_n1, _log_n0;
      float _e1, _e0;
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      // functions
      ClfOnlineStump()
          :
            ClfWeak()
      {
        init();
      }
      ;
      ClfOnlineStump(int ind)
          :
            ClfWeak(ind)
      {
        init();
      }
      ;
      virtual void
      init();
      virtual void
      update(SampleSet &posx, SampleSet &negx, const cv::Mat_<float> & posw, const cv::Mat_<float> & negw);
      virtual bool
      classify(SampleSet &x, int i);
      virtual float
      classifyF(SampleSet &x, int i);
      virtual void
      copy(const ClfWeak* c);

    };

    class ClfWStump: public ClfWeak
    {
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      // members
      float _mu0, _mu1, _sig0, _sig1;
      float _q;
      int _s;
      float _log_n1, _log_n0;
      float _e1, _e0;
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////
      // functions
      ClfWStump()
          :
            ClfWeak()
      {
        init();
      }
      ;
      ClfWStump(int ind)
          :
            ClfWeak(ind)
      {
        init();
      }
      ;
      virtual void
      init();
      virtual void
      update(SampleSet &posx, SampleSet &negx, const cv::Mat_<float> & posw, const cv::Mat_<float> & negw);
      virtual bool
      classify(SampleSet &x, int i)
      {
        return classifyF(x, i) > 0;
      }
      ;
      virtual float
      classifyF(SampleSet &x, int i);
      virtual void
      copy(const ClfWeak* c);
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline vectorb
    ClfWeak::classifySet(SampleSet &x)
    {
      vectorb res(x.size());

      for (int k = 0; k < (int) res.size(); k++)
      {
        res[k] = classify(x, k);
      }
      return res;
    }
    inline vectorf
    ClfWeak::classifySetF(SampleSet &x)
    {
      vectorf res(x.size());

#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int k = 0; k < (int) res.size(); k++)
      {
        res[k] = classifyF(x, k);
      }
      return res;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void
    ClfOnlineStump::update(SampleSet &posx, SampleSet &negx, const cv::Mat_<float> & posw, const cv::Mat_<float> & negw)
    {
      float posmu = 0.0, negmu = 0.0;
      if (posx.size() > 0)
        posmu = static_cast<float>(cv::mean(posx.ftrVals(_ind))[0]);
      if (negx.size() > 0)
		  negmu = static_cast<float>(cv::mean(negx.ftrVals(_ind))[0]);

      if (_trained)
      {
        if (posx.size() > 0)
        {
          _mu1 = (_lRate * _mu1 + (1 - _lRate) * posmu);
          cv::Mat diff = posx.ftrVals(_ind) - _mu1;
		  _sig1 = _lRate * _sig1 + (1 - _lRate) * static_cast<float>(cv::mean(diff.mul(diff))[0]);
        }
        if (negx.size() > 0)
        {
          _mu0 = (_lRate * _mu0 + (1 - _lRate) * negmu);
          cv::Mat diff = negx.ftrVals(_ind) - _mu0;
		  _sig0 = _lRate * _sig0 + (1 - _lRate) * static_cast<float>(cv::mean(diff.mul(diff))[0]);
        }

        _q = (_mu1 - _mu0) / 2;
        _s = sign(_mu1-_mu0);
        _log_n0 = std::log(float(1.0f / pow(_sig0, 0.5f)));
        _log_n1 = std::log(float(1.0f / pow(_sig1, 0.5f)));
        //_e1 = -1.0f/(2.0f*_sig1+1e-99f);
        //_e0 = -1.0f/(2.0f*_sig0+1e-99f);
        _e1 = -1.0f / (2.0f * _sig1 + std::numeric_limits<float>::min());
        _e0 = -1.0f / (2.0f * _sig0 + std::numeric_limits<float>::min());

      }
      else
      {
        _trained = true;
        if (posx.size() > 0)
        {
          _mu1 = posmu;
          cv::Scalar scal_mean, scal_std_dev;
          cv::meanStdDev(posx.ftrVals(_ind), scal_mean, scal_std_dev);
		  _sig1 = static_cast<float>(scal_std_dev[0] * scal_std_dev[0]) + 1e-9f;
        }

        if (negx.size() > 0)
        {
          _mu0 = negmu;
          cv::Scalar scal_mean, scal_std_dev;
          cv::meanStdDev(negx.ftrVals(_ind), scal_mean, scal_std_dev);
		  _sig0 = static_cast<float>(scal_std_dev[0] * scal_std_dev[0]) + 1e-9f;
        }

        _q = (_mu1 - _mu0) / 2;
        _s = sign(_mu1-_mu0);
        _log_n0 = std::log(float(1.0f / pow(_sig0, 0.5f)));
        _log_n1 = std::log(float(1.0f / pow(_sig1, 0.5f)));
        //_e1 = -1.0f/(2.0f*_sig1+1e-99f);
        //_e0 = -1.0f/(2.0f*_sig0+1e-99f);
        _e1 = -1.0f / (2.0f * _sig1 + std::numeric_limits<float>::min());
        _e0 = -1.0f / (2.0f * _sig0 + std::numeric_limits<float>::min());
      }
    }

    inline bool
    ClfOnlineStump::classify(SampleSet &x, int i)
    {
      float xx = getFtrVal(x, i);
      double log_p0 = (xx - _mu0) * (xx - _mu0) * _e0 + _log_n0;
      double log_p1 = (xx - _mu1) * (xx - _mu1) * _e1 + _log_n1;
      return log_p1 > log_p0;
    }
    inline float
    ClfOnlineStump::classifyF(SampleSet &x, int i)
    {
      float xx = getFtrVal(x, i);
      double log_p0 = (xx - _mu0) * (xx - _mu0) * _e0 + _log_n0;
      double log_p1 = (xx - _mu1) * (xx - _mu1) * _e1 + _log_n1;
      return float(log_p1 - log_p0);
    }
    inline void
    ClfOnlineStump::copy(const ClfWeak* c)
    {
      ClfOnlineStump *cc = (ClfOnlineStump*) c;
      _mu0 = cc->_mu0;
      _mu1 = cc->_mu1;
      _sig0 = cc->_sig0;
      _sig1 = cc->_sig1;
      _lRate = cc->_lRate;
      _e0 = cc->_e0;
      _e1 = cc->_e1;
      _log_n0 = cc->_log_n0;
      _log_n1 = cc->_log_n1;

      return;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void
    ClfWStump::update(SampleSet &posx, SampleSet &negx, const cv::Mat_<float> & posw, const cv::Mat_<float> & negw)
    {
      cv::Mat_<float> poswn, negwn;
      if ((posx.size() != posw.size().area()) || (negx.size() != negw.size().area()))
        abortError(__LINE__, __FILE__, "ClfWStump::update - number of samples and number of weights mismatch");

      float posmu = 0.0, negmu = 0.0;
      if (posx.size() > 0)
      {
        poswn = posw / (cv::sum(posw)[0] + 1e-6);
		posmu = static_cast<float>(cv::mean(posx.ftrVals(_ind).mul(poswn))[0]);
      }
      if (negx.size() > 0)
      {
        negwn = negw / (cv::sum(negw)[0] + 1e-6);
		negmu = static_cast<float>(cv::mean(negx.ftrVals(_ind).mul(negwn))[0]);
      }

      if (_trained)
      {
        if (posx.size() > 0)
        {
          _mu1 = (_lRate * _mu1 + (1 - _lRate) * posmu);
          cv::Scalar scal_mean, scal_std_dev;
          cv::meanStdDev(posx.ftrVals(_ind).mul(poswn), scal_mean, scal_std_dev);
		  _sig1 = static_cast<float>(_lRate * _sig1 + (1 - _lRate) * scal_std_dev[0] * scal_std_dev[0]);
        }
        if (negx.size() > 0)
        {
          _mu0 = (_lRate * _mu0 + (1 - _lRate) * negmu);
          cv::Scalar scal_mean, scal_std_dev;
          cv::meanStdDev(negx.ftrVals(_ind).mul(negwn), scal_mean, scal_std_dev);
		  _sig0 = static_cast<float>(_lRate * _sig0 + (1 - _lRate) * scal_std_dev[0] * scal_std_dev[0]);
        }
      }
      else
      {
        _trained = true;
        _mu1 = posmu;
        _mu0 = negmu;
        cv::Scalar scal_mean, scal_std_dev;
        if (negx.size() > 0)
        {
          cv::meanStdDev(negx.ftrVals(_ind).mul(negwn), scal_mean, scal_std_dev);
		  _sig0 = static_cast<float>(scal_std_dev[0] * scal_std_dev[0]) + 1e-9f;
        }
        if (posx.size() > 0)
        {
          cv::meanStdDev(posx.ftrVals(_ind).mul(poswn), scal_mean, scal_std_dev);
		  _sig1 = static_cast<float>(scal_std_dev[0] * scal_std_dev[0]) + 1e-9f;
        }
      }

      _log_n0 = std::log(float(1.0f / pow(_sig0, 0.5f)));
      _log_n1 = std::log(float(1.0f / pow(_sig1, 0.5f)));
      _e1 = -1.0f / (2.0f * _sig1);
      _e0 = -1.0f / (2.0f * _sig0);
    }

    inline float
    ClfWStump::classifyF(SampleSet &x, int i)
    {
      float xx = getFtrVal(x, i);
      double log_p0 = (xx - _mu0) * (xx - _mu0) * _e0 + _log_n0;
      double log_p1 = (xx - _mu1) * (xx - _mu1) * _e1 + _log_n1;
      return (float) (log_p1 - log_p0);
    }
    inline void
    ClfWStump::copy(const ClfWeak* c)
    {
      ClfWStump *cc = (ClfWStump*) c;
      _mu0 = cc->_mu0;
      _mu1 = cc->_mu1;

      return;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline vectorf
    ClfAdaBoost::classify(SampleSet &x, bool logR)
    {
      int numsamples = x.size();
      vectorf res(numsamples);
      vectorb tr;

      // for each selector, accumate in the res vector
      for (int sel = 0; sel < (int) _selectors.size(); sel++)
      {
        tr = _weakclf[_selectors[sel]]->classifySet(x);
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int j = 0; j < numsamples; j++)
        {
          res[j] += tr[j] ? _alphas[sel] : -_alphas[sel];
        }

      }

      // return probabilities or log odds ratio
      if (!logR)
      {
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int j = 0; j < (int) res.size(); j++)
        {
          res[j] = sigmoid(2 * res[j]);
        }
      }

      return res;
    }

    inline vectorf
    ClfMilBoost::classify(SampleSet &x, bool logR)
    {
      int numsamples = x.size();
      vectorf res(numsamples);
      vectorf tr;

      for (uint w = 0; w < _selectors.size(); w++)
      {
        tr = _weakclf[_selectors[w]]->classifySetF(x);
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int j = 0; j < numsamples; j++)
        {
          res[j] += tr[j];
        }
      }

      // return probabilities or log odds ratio
      if (!logR)
      {
#ifdef _OPENMP
#pragma omp parallel for
#endif
        for (int j = 0; j < (int) res.size(); j++)
        {
          res[j] = sigmoid(res[j]);
        }
      }

      return res;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void
    ClfStrong::eval(vectorf ppos, vectorf pneg, float &err, float &fp, float &fn, float thresh)
    {
      fp = 0;
      fn = 0;
      for (uint k = 0; k < ppos.size(); k++)
        (ppos[k] < thresh) ? fn++ : fn;

      for (uint k = 0; k < pneg.size(); k++)
        (pneg[k] >= thresh) ? fp++ : fp;

      fn /= ppos.size();
      fp /= pneg.size();

      err = 0.5f * fp + 0.5f * fn;
    }
    inline float
    ClfStrong::likl(vectorf ppos, vectorf pneg)
    {
      float likl = 0, posw = 1.0f / ppos.size(), negw = 1.0f / pneg.size();

      for (uint k = 0; k < ppos.size(); k++)
        likl += log(ppos[k] + 1e-5f) * posw;

      for (uint k = 0; k < pneg.size(); k++)
        likl += log(1 - pneg[k] + 1e-5f) * negw;

      return likl;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    class TrackerParams
    {
    public:
      TrackerParams();

      vectori _boxcolor; // for outputting video
      uint _lineWidth; // line width
      uint _negnumtrain, _init_negnumtrain; // # negative samples to use during training, and init
      float _posradtrain, _init_postrainrad; // radius for gathering positive instances
      uint _posmaxtrain; // max # of pos to train with
      bool _debugv; // displays response map during tracking [kinda slow, but help in debugging]
      vectorf _initstate; // [x,y,scale,orientation] - note, scale and orientation currently not used
      bool _useLogR; // use log ratio instead of probabilities (tends to work much better)
      bool _initWithFace; // initialize with the OpenCV tracker rather than _initstate
      bool _disp; // display video with tracker state (colored box)

      std::string _vidsave; // filename - save video with tracking box
      std::string _trsave; // filename - save file containing the coordinates of the box (txt file with [x y width height] per row)

    };

    class SimpleTrackerParams: public TrackerParams
    {
    public:
      SimpleTrackerParams();

      uint _srchwinsz; // size of search window
      uint _negsamplestrat; // [0] all over image [1 - default] close to the search window
    };

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    class SimpleTracker
    {
    public:
      SimpleTracker()
          :
            _cnt(0)
      {
      }
      ~SimpleTracker()
      {
      }
      double
      track_frame(const cv::Mat & frame); // track object in a frame;  requires init() to have been called.
      bool
      init(const cv::Mat & frame, SimpleTrackerParams p, ClfStrongParams *clfparams);
      const cv::Mat_<float> &
      getFtrHist() const
      {
        return _clf->_ftrHist;
      } // only works if _clf->_storeFtrHistory is set to true.. mostly for debugging

      inline void
      getTrackBox(cv::Rect & roi)
      {
        roi.width = cvRound(_curState[2]);
        roi.height = cvRound(_curState[3]);
        roi.x = cvRound(_curState[0]);
        roi.y = cvRound(_curState[1]);
      }

    private:
      cv::Ptr<ClfStrong> _clf;
      vectorf _curState;
      SimpleTrackerParams _trparams;
      cv::Ptr<ClfStrongParams> _clfparams;
      int _cnt;
    };

  } // namespace mil
} // namespace cv

#endif  // #ifndef __OPENCV_ONLINE_MIL_H__
/* End of file. */
