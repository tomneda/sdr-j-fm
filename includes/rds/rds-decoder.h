/*
 *    Copyright (C) 2014
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair Programming
 *
 *	This part of the jsdr is a mixture of code  based on code from
 *	various sources. Two in particular:
 *
 *    FMSTACK Copyright (C) 2010 Michael Feilen
 *
 *    Author(s)       : Michael Feilen (michael.feilen@tum.de)
 *    Initial release : 01.09.2009
 *    Last changed    : 09.03.2010
 *
 *	cuteSDR (c) M Wheatly 2011
 *
 *    This file is part of the SDR-J.
 *    Many of the ideas as implemented in SDR-J are derived from
 *    other work, made available through the GNU general Public License.
 *    All copyrights of the original authors are recognized.
 *
 *    SDR-J is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    SDR-J is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with SDR-J; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef __RDS_DECODER
#define __RDS_DECODER

#include <QObject>
#include "fm-constants.h"
#include "rds-group.h"
#include "rds-blocksynchronizer.h"
#include "rds-groupdecoder.h"
#include "fft.h"
#include "iir-filters.h"
#include "sincos.h"
#include "Xtan2.h"

class AGC
{
public:
  AGC(float rate, float reference, float gain);
  AGC() = delete;
  ~AGC() = default;

  float rate() const { return _rate; }
  float reference() const { return _reference; }
  float gain() const { return _gain; }

  void set_rate(float rate) { _rate = rate; }
  void set_reference(float reference) { _reference = reference; }
  void set_gain(float gain) { _gain = gain; }

  DSPCOMPLEX scale(DSPCOMPLEX input);
  void scaleN(DSPCOMPLEX output[], const DSPCOMPLEX input[], uint32_t n);

protected:
  float _rate;     // adjustment rate
  float _reference;// reference value
  float _gain;     // current gain
};

class TimeSync
{
public:
  TimeSync(const float iSps, const float iAlpha) : mSmplPerSym(iSps), mAlpha(iAlpha) {}
  TimeSync() = delete;
  ~TimeSync() = default;

  inline bool process_sample(const DSPCOMPLEX iZ, DSPCOMPLEX & oZ)
  {
    mSmplBuff[0] = mSmplBuff[1];
    mSmplBuff[1] = mSmplBuff[2];
    mSmplBuff[2] = iZ;

    if (++mSmplCnt >= mSkipNoSmpl)
    {
      // get hard decision values (rail to rail)
      for (int32_t i = 0; i < 3; ++i)
      {
        mSampBuffRail[i] = DSPCOMPLEX((real(mSmplBuff[i]) > 0.0f ? 1.0f : -1.0f), (imag(mSmplBuff[i]) > 0.0f ? 1.0f : -1.0f));
      }

      // Calculate Mueller & Muller metrics
      //    const DSPCOMPLEX x = (out_rail[2] - out_rail[0]) * conj(out[1]);
      //    const DSPCOMPLEX y = (out[2] - out[0]) * conj(out_rail[1]);
      //    const DSPFLOAT mm_val = real(y - x);

      //    const DSPFLOAT x = real((out_rail[2] - out_rail[0]) * conj(out[1]));
      //    const DSPFLOAT y = real((out[2] - out[0]) * conj(out_rail[1]));
      //    const DSPFLOAT mm_val = y - x;

      const DSPFLOAT x = real(mSampBuffRail[2] - mSampBuffRail[0]) * real(mSmplBuff[1])     + imag(mSampBuffRail[2] - mSampBuffRail[0]) * imag(mSmplBuff[1]);
      const DSPFLOAT y = real(mSmplBuff[2]     - mSmplBuff[0])     * real(mSampBuffRail[1]) + imag(mSmplBuff[2]     - mSmplBuff[0])     * imag(mSampBuffRail[1]);
      const DSPFLOAT mm_val = y - x;

      mMu += mSmplPerSym + mAlpha * mm_val;
      mSkipNoSmpl = (int32_t)(/*round*/(mMu)); // round down to nearest int since we are using it as an index
      mMu -= mSkipNoSmpl; // remove the integer part of mu

      oZ = mSmplBuff[2];
      mSmplCnt = 0;
      return true;
    }

    return false;
  }

private:
  DSPCOMPLEX mSmplBuff[3] {}; // current and last 2 samples
  DSPCOMPLEX mSampBuffRail[3] {}; // current and last 2 samples
  DSPFLOAT mMu = 0.00; // 0.01;
  float mSmplPerSym = 16;
  float mAlpha = 0.01;
  int32_t mSkipNoSmpl = 3;  // 2 to fill out buffer
  int32_t mSmplCnt = 0;
};

class Costas
{
public:
  Costas(const float iAlpha, const float iBeta) : mAlpha(iAlpha), mBeta(iBeta) {}
  Costas() = delete;
  ~Costas() = default;

  inline DSPCOMPLEX process_sample(const DSPCOMPLEX z)
  {
    const DSPCOMPLEX r = z * std::exp(DSPCOMPLEX(0, -mPhase));
    const float error = real(r) * imag(r);
    mFreq += (mBeta * error);
    mPhase += mFreq + (mAlpha * error);
    mPhase = PI_Constrain(mPhase);
    return r;
  }

  //float get_cur_freq() const { return mFreq * mSampleRate / (2 * M_PI); }

  private:
    const float mAlpha;
    const float mBeta;
    float mFreq = 0;
    float mPhase = 0;
};

class RadioInterface;

class rdsDecoder : public QObject
{
  Q_OBJECT
public:
  rdsDecoder(RadioInterface *, int32_t, SinCos *);
  ~rdsDecoder(void);

  enum class ERdsMode
  {
    NO_RDS,
    RDS1,
    RDS2,
    RDS3
  };

  void doDecode(const DSPFLOAT, DSPFLOAT * const, const ERdsMode);
  bool doDecode(const DSPCOMPLEX, DSPCOMPLEX * const);
  void reset(void);

private:
  void processBit(bool);
  void doDecode1(const DSPFLOAT, DSPFLOAT * const);
  void doDecode2(const DSPFLOAT, DSPFLOAT * const);

  AGC mAGC;
  TimeSync mTimeSync;
  Costas mCostas;
  compAtan mAtan;
  int32_t mSampleRate;
  int32_t mNumOfFrames;
  SinCos * mpSinCos;
  RadioInterface * mpRadioInterface;
  RDSGroup * mpRdsGroup;
  rdsBlockSynchronizer * mpRdsBlockSync;
  rdsGroupDecoder * mpRdsGroupDecoder;
  DSPFLOAT mOmegaRDS;
  int32_t mSymbolCeiling;
  int32_t mSymbolFloor;
  bool mPrevBit;
  DSPFLOAT mBitIntegrator;
  DSPFLOAT mBitClkPhase;
  DSPFLOAT mPrev_clkState;
  bool mResync;

  std::vector<DSPFLOAT> mRrcImpMancVec;

  DSPFLOAT * mpRdsBuffer;
  DSPCOMPLEX * mpRdsBufferCplx;
  DSPFLOAT * mpRdsKernel;
  int16_t mCurRdsBufferIdx;
  int16_t mRdsfilterSize;

  DSPFLOAT doMatchFiltering(DSPFLOAT);
  DSPCOMPLEX doMatchFiltering(DSPCOMPLEX);

  BandPassIIR * mpSharpFilter;
  DSPFLOAT mRdsLastSyncSlope;
  DSPFLOAT mRdsLastSync;
  DSPFLOAT mRdsLastData;
  bool mPreviousBit;
  DSPFLOAT * mSyncBuffer;
  int16_t mSyncBuffPtrIdx;
  DSPCOMPLEX * mSyncBufferCplx;
  int16_t mSyncBuffPtrIdxCplx = 0;

  void synchronizeOnBitClk(DSPFLOAT *, int16_t);

signals:
  void setCRCErrors(int);
  void setSyncErrors(int);
  void setbitErrorRate(int);
};

#endif
