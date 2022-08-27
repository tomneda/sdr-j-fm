/*
 *    Copyright (C) 2008, 2009, 2010
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair Programming
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

#ifndef __FM_PROCESSOR__
#define __FM_PROCESSOR__

#include "fft-filters.h"
#include "fir-filters.h"
#include "fm-constants.h"
#include "fm-demodulator.h"
#include "rds-decoder.h"
#include "squelchClass.h"
#include "oscillator.h"
#include "pllC.h"
#include "ringbuffer.h"
#include "sincos.h"
#include <QObject>
#include <QThread>
#include <sndfile.h>

class deviceHandler;
class RadioInterface;
//class fm_Demodulator;
//class rdsDecoder;
class audioSink;
class newConverter;

class fmProcessor : public QThread
{
  Q_OBJECT

public:
  enum class FM_Mode
  {
    Stereo,
    StereoPano,
    Mono
  };

  enum class ELfPlot
  {
    OFF,
    IF_FILTERED,
    MULTIPLEX,
    AF_SUM,
    AF_DIFF,
    AF_MONO_FILTERED,
    AF_LEFT_FILTERED,
    AF_RIGHT_FILTERED,
    RDS
  };

public:
  fmProcessor(deviceHandler *,
              RadioInterface *,
              audioSink *,
              int32_t,                // inputRate
              int32_t,                // decimation
              int32_t,                // workingRate
              int32_t,                // audioRate,
              int32_t,                // displaySize
              int32_t,                // spectrumSize
              int32_t,                // averageCount
              int32_t,                // repeatRate
              RingBuffer<double> *,   // HFScope
              RingBuffer<double> *,   // LFScope
              int16_t,                // filterDepth
              int16_t);               // threshold scanning
  ~fmProcessor() override;

  void stop();   // stop the processor
  void setfmMode(FM_Mode);
  void setFMdecoder(int16_t);
  void setSoundMode(uint8_t);
  void setStereoPanorama(int16_t iStereoPan);
  void setSoundBalance(int16_t);
  void setDeemphasis(int16_t);
  void setVolume(const float iVolGainDb);
  void setLFcutoff(int32_t);
  void startDumping(SNDFILE *);
  void stopDumping();
  void setBandwidth(int32_t);
  void setBandfilterDegree(int32_t);
  void setAttenuation(int16_t, int16_t);
  void setfmRdsSelector(rdsDecoder::ERdsMode);
  void resetRds();
  void set_localOscillator(int32_t);
  void setFreezer(bool);
  void set_squelchMode(bool);
  void setInputMode(uint8_t);
  void setLfPlotType(ELfPlot);
  bool ok();
  bool isPilotLocked(float & oLockStrength) const;
  squelch * getSquelchObj() const { return mMySquelch; }
  void setAutoMonoMode(const bool iAutoMonoMode) { mAutoMono = iAutoMonoMode; }
  void triggerDrawNewHfSpectrum() { mFillAverageHfBuffer = true; }
  void triggerDrawNewLfSpectrum() { mFillAverageLfBuffer = true; }

  DSPFLOAT get_pilotStrength();
  DSPFLOAT get_rdsStrength();
  DSPFLOAT get_noiseStrength();
  DSPFLOAT get_dcComponent();

  void startScanning();
  void stopScanning();
  fm_Demodulator::TDecoderListNames & listNameofDecoder();
  const char * nameofDecoder();

  enum Channels
  {
    S_STEREO,
    S_STEREO_SWAPPED,
    S_LEFT,
    S_RIGHT,
    S_LEFTplusRIGHT,
    S_LEFTminusRIGHT
  };

  void set_squelchValue(int16_t);

private:
  void run() override;
  void mapSpectrum(const DSPCOMPLEX * const, double * const);
  void mapHalfSpectrum(const DSPCOMPLEX * const, double * const);
  void processLfSpectrum();
  void fill_average_buffer(const double * const, double * const);
  void add_to_average(const double * const, double * const);
  void extractLevels(const double * const, const int32_t);
  void extractLevelsHalfSpectrum(const double * const, const int32_t);
  void sendSampletoOutput(DSPCOMPLEX);
  void evaluatePeakLevel(const DSPCOMPLEX s);

private:
  deviceHandler * mMyRig;
  RadioInterface * mMyRadioInterface;
  audioSink * mAudioSink;
  squelch *mMySquelch;
  int32_t mInputRate;    // typ. 1536 kSpS
  int32_t mFmRate;       // typ.  256 kSpS = mInputRate / 6
  int32_t mWorkingRate;  // typ.   48 kSpS
  int32_t mAudioRate;    // typ.   48 kSpS
  int32_t mDisplaySize;
  int32_t mAverageCount;
  int32_t mRepeatRate;
  bool mFillAverageHfBuffer{ true };
  bool mFillAverageLfBuffer{ true };
  RingBuffer<double> * mpHfBuffer;
  RingBuffer<double> * mpLfBuffer;
  int16_t mFilterDepth;
  uint8_t mInputMode;
  //int32_t freezer;
  bool mScanning;
  int16_t mThresHold;

  DSPFLOAT getSignal(DSPCOMPLEX *, int32_t);
  DSPFLOAT getNoise(DSPCOMPLEX *, int32_t);

  bool mSquelchOn;
  int32_t mSpectrumSize;
  common_fft * mpSpectrum_fft_hf;
  common_fft * mpSpectrum_fft_lf;
  DSPCOMPLEX * mpSpectrumBuffer_hf;
  DSPCOMPLEX * mpSpectrumBuffer_lf;
  double * mpDisplayBuffer_lf = nullptr;
  //double *mpDisplayBuffer;
  double * mpLocalBuffer;
  DecimatingFIR * mpFmBandfilter;
  Oscillator * mpLocalOscillator;
  newConverter * mpTheConverter;
  int32_t mLoFrequency;
  bool mRunning;
  SinCos * mpMySinCos;
  LowPassFIR * mpFmFilter;
  int32_t mFmBandwidth;
  int32_t mFmFilterDegree;
  bool mNewFilter;
  bool mAutoMono{ true };

  int16_t mOldSquelchValue;
  int16_t mSquelchValue;

  bool mDumping;
  SNDFILE * mpDumpFile;
  int32_t mDecimatingScale;

  int32_t mMyCount;
  int16_t mLgain;
  int16_t mRgain;

  int32_t mPeakLevelCurSampleCnt{ 0 };
  int32_t mPeakLevelSampleMax{ 0x7FFFFFFF };
  DSPFLOAT mAbsPeakLeft{ 0.0f };
  DSPFLOAT mAbsPeakRight{ 0.0f };

  newConverter * mpAudioDecimator;
  DSPCOMPLEX * mpAudioOut;
  rdsDecoder * mpMyRdsDecoder;

  void process_stereo_or_mono(const float, DSPCOMPLEX *, DSPFLOAT *);
  void process_mono_only(const float, DSPCOMPLEX *, DSPFLOAT *);

  fftFilter * mpPilotBandFilter;
  fftFilter * mpRdsBandFilter;
  fftFilter * mpRdsLowPassFilter;
  HilbertFilter * mpRdsHilbertFilter;
  int32_t mRdsSampleCnt;

  DSPFLOAT mPilotDelay;

  DSPCOMPLEX audioGainCorrection(DSPCOMPLEX);

  DSPFLOAT mVolumeFactor{ 0.5f };
  //DSPFLOAT audioGain;
  int32_t mMaxFreqDeviation;
  int32_t mNormFreqDeviation;
  DSPFLOAT mOmegaDemod;
  fftFilter * mpFmAudioFilter;
  bool mFmAudioFilterActive;

  DSPFLOAT mPanorama{ 1.0f };
  int16_t mBalance{ 0 };
  DSPFLOAT mLeftChannel{ 1.0f };    // -(balance - 50.0) / 100.0;;
  DSPFLOAT mRightChannel{ 1.0f };   // (balance + 50.0) / 100.0;;
  FM_Mode mFmModus;
  uint8_t mSelector;
  fm_Demodulator * mpTheDemodulator;

  rdsDecoder::ERdsMode mRdsModus{ rdsDecoder::ERdsMode::NO_RDS };

  float mNoiseLevel;
  float mPilotLevel;
  float mRdsLevel;
  //int8_t viewSelector;
  pllC * mpRds_plldecoder;
  DSPFLOAT mK_FM;

  DSPFLOAT mXkm1;
  DSPFLOAT mYkml;
  DSPFLOAT mAlpha;

  ELfPlot mLfPlotType = ELfPlot::MULTIPLEX;

  class pilotRecovery
  {
  private:
    int32_t Rate_in;
    int32_t mSampleLockStableCnt;
    DSPFLOAT pilot_OscillatorPhase;
    DSPFLOAT pilot_oldValue;
    DSPFLOAT omega;
    DSPFLOAT gain;
    SinCos * mySinCos;
    DSPFLOAT pilot_Lock;
    DSPFLOAT quadRef;
    bool pll_isLocked;
    bool mLockStable;

  public:
    pilotRecovery(int32_t Rate_in, DSPFLOAT omega, DSPFLOAT gain, SinCos * mySinCos)
    {
      this->Rate_in = Rate_in;
      this->omega = omega;
      this->gain = gain;
      this->mySinCos = mySinCos;
      pll_isLocked = false;
      mLockStable = false;
      pilot_Lock = 0;
      pilot_oldValue = 0;
      pilot_OscillatorPhase = 0;
      mSampleLockStableCnt = 0;
    }

    ~pilotRecovery() = default;

    bool isLocked() const { return pll_isLocked; }
    float getLockedStrength() const { return pilot_Lock; }

    DSPFLOAT getPilotPhase(const DSPFLOAT pilot)
    {
      const DSPFLOAT OscillatorValue = mySinCos->getCos(pilot_OscillatorPhase);
      const DSPFLOAT PhaseError = pilot * OscillatorValue;

      pilot_OscillatorPhase += PhaseError * gain;

      const DSPFLOAT currentPhase = PI_Constrain(pilot_OscillatorPhase);

      pilot_OscillatorPhase = PI_Constrain(pilot_OscillatorPhase + omega);

      quadRef = (OscillatorValue - pilot_oldValue) / omega;
      //	         quadRef	= PI_Constrain (quadRef);
      pilot_oldValue = OscillatorValue;
      constexpr float alpha = 1.0f / 3000.0f;
      pilot_Lock = alpha * (-quadRef * pilot) + pilot_Lock * (1.0 - alpha);

      const bool pll_isLocked_temp = (pilot_Lock > 0.07f);

      // Check if the PLL lock is stable for a while.
      // This is important in very noisy receive condition to maintain a stable mono mode.
      if (pll_isLocked_temp)
      {
        if (pll_isLocked || ++mSampleLockStableCnt > (Rate_in >> 1))   // for 500ms the PLL lock has to be stable
        {
          pll_isLocked = true;
        }
      }
      else   // not locked -> reset stable counter
      {
        pll_isLocked = false;
        mSampleLockStableCnt = 0;
      }

      return currentPhase;
    }
  };

  pilotRecovery * mpPilotRecover;

signals:
  void setPLLisLocked(bool);
  void hfBufferLoaded();
  void lfBufferLoaded();
  void showStrength(float, float);
  void scanresult();
  void showPeakLevel(const float, const float);
};

#endif
