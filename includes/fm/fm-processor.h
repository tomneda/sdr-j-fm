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
class rdsDecoder;
class audioSink;
class newConverter;

class fmProcessor : public QThread {
  Q_OBJECT

public:
  enum class FM_Mode { Stereo, StereoPano, Mono };

public:
  fmProcessor(deviceHandler *, RadioInterface *, audioSink *,
              int32_t,              // inputRate
              int32_t,              // decimation
              int32_t,              // workingRate
              int32_t,              // audioRate,
              int32_t,              // displaySize
              int32_t,              // spectrumSize
              int32_t,              // averageCount
              int32_t,              // repeatRate
              RingBuffer<double> *, // HFScope
              RingBuffer<double> *, // LFScope
              int16_t,              // filterDepth
              int16_t);             // threshold scanning
  ~fmProcessor() override;

  void stop(); // stop the processor
  void setfmMode(FM_Mode);
  void setFMdecoder(int16_t);
  void setSoundMode(uint8_t);
  void setStereoPanorama(int16_t iStereoPan);
  void setSoundBalance(int16_t);
  void setDeemphasis(int16_t);
  void setVolume(int16_t);
  void setLFcutoff(int32_t);
  void startDumping(SNDFILE *);
  void stopDumping();
  void setBandwidth(int32_t);
  void setBandfilterDegree(int32_t);
  void setAttenuation(int16_t, int16_t);
  void setfmRdsSelector(int8_t);
  void resetRds();
  void set_localOscillator(int32_t);
  void setFreezer(bool);
  void set_squelchMode(bool);
  void setInputMode(uint8_t);
  bool ok();
  bool isPilotLocked(float & oLockStrength) const;
  void set_auto_mono_mode(const bool iAutoMonoMode) { mAutoMono = iAutoMonoMode; }

  DSPFLOAT get_pilotStrength();
  DSPFLOAT get_rdsStrength();
  DSPFLOAT get_noiseStrength();
  DSPFLOAT get_dcComponent();

  void startScanning();
  void stopScanning();
  fm_Demodulator::TDecoderListNames & listNameofDecoder();
  const char *nameofDecoder();

  enum Channels
  {
    S_STEREO = 0,
    S_LEFT = 1,
    S_RIGHT = 2,
    S_LEFTplusRIGHT = 0103,
    S_LEFTminusRIGHT = 0104
  };

  void set_squelchValue(int16_t);

private:
  void run() override;
  void mapSpectrum(DSPCOMPLEX *, double *);
  void add_to_average(double *, double *);
  void extractLevels(double *, int32_t);
  deviceHandler *myRig;
  RadioInterface *myRadioInterface;
  audioSink *theSink;
  int32_t inputRate;
  int32_t fmRate;
  int32_t workingRate;
  int32_t audioRate;
  int32_t displaySize;
  int32_t averageCount;
  int32_t repeatRate;
  RingBuffer<double> *hfBuffer;
  RingBuffer<double> *lfBuffer;
  int16_t filterDepth;
  uint8_t inputMode;
  int32_t freezer;
  bool scanning;
  int16_t thresHold;
  DSPFLOAT getSignal(DSPCOMPLEX *, int32_t);
  DSPFLOAT getNoise(DSPCOMPLEX *, int32_t);
  bool squelchOn;
  int32_t spectrumSize;
  common_fft *spectrum_fft_hf;
  common_fft *spectrum_fft_lf;
  DSPCOMPLEX *spectrumBuffer_hf;
  DSPCOMPLEX *spectrumBuffer_lf;
  double *displayBuffer;
  double *localBuffer;
  void sendSampletoOutput(DSPCOMPLEX);
  DecimatingFIR *fmBandfilter;
  Oscillator *localOscillator;
  newConverter *theConverter;
  int32_t lo_frequency;
  bool running;
  SinCos *mySinCos;
  LowPassFIR *fmFilter;
  int32_t fmBandwidth;
  int32_t fmFilterDegree;
  bool newFilter;
  bool mAutoMono{true};

  int16_t old_squelchValue;
  int16_t squelchValue;

  bool dumping;
  SNDFILE *dumpFile;
  int32_t decimatingScale;

  int32_t myCount;
  int16_t Lgain;
  int16_t Rgain;

  newConverter *audioDecimator;
  DSPCOMPLEX *audioOut;
  rdsDecoder *myRdsDecoder;

  void stereo(float, DSPCOMPLEX *, DSPFLOAT *);
  void mono(float, DSPCOMPLEX *, DSPFLOAT *);
  fftFilter *pilotBandFilter;
  fftFilter *rdsBandFilter;
  fftFilter *rdsLowPassFilter;
  HilbertFilter *rdsHilbertFilter;

  DSPFLOAT pilotDelay;
  DSPCOMPLEX audioGainCorrection(DSPCOMPLEX);
  DSPFLOAT Volume;
  //DSPFLOAT audioGain;
  int32_t max_freq_deviation;
  int32_t norm_freq_deviation;
  DSPFLOAT omega_demod;
  LowPassFIR *fmAudioFilter;

  DSPFLOAT mPanorama{1.0f};
  int16_t balance;
  DSPFLOAT leftChannel;
  DSPFLOAT rightChannel;
  FM_Mode fmModus;
  uint8_t selector;
  //DSPFLOAT peakLevel;
  //int32_t peakLevelcnt;
  fm_Demodulator *TheDemodulator;

  int8_t rdsModus;

  float noiseLevel;
  float pilotLevel;
  float rdsLevel;
  int8_t viewSelector;
  pllC *rds_plldecoder;
  DSPFLOAT K_FM;

  DSPFLOAT xkm1;
  DSPFLOAT ykm1;
  DSPFLOAT alpha;

  class pilotRecovery
  {
  private:
    int32_t Rate_in;
    DSPFLOAT pilot_OscillatorPhase;
    DSPFLOAT pilot_oldValue;
    DSPFLOAT omega;
    DSPFLOAT gain;
    SinCos *mySinCos;
    DSPFLOAT pilot_Lock;
    bool pll_isLocked;
    DSPFLOAT quadRef;
    DSPFLOAT accumulator;
    int32_t count;

  public:
    pilotRecovery(int32_t Rate_in, DSPFLOAT omega, DSPFLOAT gain, SinCos * mySinCos)
    {
      this->Rate_in         = Rate_in;
      this->omega           = omega;
      this->gain            = gain;
      this->mySinCos        = mySinCos;
      pll_isLocked          = false;
      pilot_Lock            = 0;
      pilot_oldValue        = 0;
      pilot_OscillatorPhase = 0;
    }

    ~pilotRecovery() = default;

    bool isLocked() const { return pll_isLocked; }
    float getLockedStrength() const { return pilot_Lock; }

    DSPFLOAT getPilotPhase(const DSPFLOAT pilot)
    {
      const DSPFLOAT OscillatorValue = mySinCos->getCos(pilot_OscillatorPhase);
      const DSPFLOAT PhaseError      = pilot * OscillatorValue;

      pilot_OscillatorPhase += PhaseError * gain;

      const DSPFLOAT currentPhase = PI_Constrain(pilot_OscillatorPhase);

      pilot_OscillatorPhase = PI_Constrain(pilot_OscillatorPhase + omega);

      quadRef = (OscillatorValue - pilot_oldValue) / omega;
      //	         quadRef	= PI_Constrain (quadRef);
      pilot_oldValue = OscillatorValue;
      constexpr float alpha = 1.0f / 3000.0f;
      pilot_Lock     = alpha * (-quadRef * pilot) + pilot_Lock * (1.0 - alpha);

      pll_isLocked = (pilot_Lock > 0.08f);

      return currentPhase;
    }
  };

  pilotRecovery *pilotRecover;

signals:
  void setPLLisLocked(bool);
  void hfBufferLoaded();
  void lfBufferLoaded();
  void showStrength(float, float);
  void scanresult();
};

#endif
