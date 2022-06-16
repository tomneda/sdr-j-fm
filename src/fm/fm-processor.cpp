/*
 *    Copyright (C) 2010, 2011, 2012
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair Programming
 *
 *    This file is part of the SDR-J-FM program.
 *
 *    SDR-J-FM is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    SDR-J-FM is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with SDR-J-FM; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include "fm-processor.h"
#include "audiosink.h"
#include "device-handler.h"
#include "fm-constants.h"
//#include "fm-demodulator.h"
#include "newconverter.h"
#include "radio.h"
#include "rds-decoder.h"
#include "sincos.h"
#include "squelchClass.h"

#define AUDIO_FREQ_DEV_PROPORTION    0.85f
#define PILOT_FREQUENCY              19000
#define RDS_FREQUENCY                (3 * PILOT_FREQUENCY)
#define OMEGA_DEMOD                  2 * M_PI / fmRate
#define OMEGA_PILOT                  ((DSPFLOAT(PILOT_FREQUENCY)) / fmRate) * (2 * M_PI)
#define OMEGA_RDS                    ((DSPFLOAT)RDS_FREQUENCY / fmRate) * (2 * M_PI)

//
//	Note that no decimation done as yet: the samplestream is still
//	full speed
fmProcessor::fmProcessor(deviceHandler *vi, RadioInterface *RI,
                         audioSink *mySink, int32_t inputRate, int32_t fmRate,
                         int32_t workingRate, int32_t audioRate,
                         int32_t displaySize, int32_t spectrumSize,
                         int32_t averageCount, int32_t repeatRate,
                         RingBuffer<double> *hfBuffer,
                         RingBuffer<double> *lfBuffer, int16_t filterDepth,
                         int16_t thresHold)
{
  mRunning          = false;
  mMyRig            = vi;
  mMyRadioInterface = RI;
  mAudioSink        = mySink;
  mInputRate        = inputRate;
  mFmRate           = fmRate;
  mDecimatingScale  = inputRate / fmRate;
  mWorkingRate      = workingRate;
  mAudioRate        = audioRate;
  mDisplaySize      = displaySize;
  mAverageCount     = averageCount;
  mRepeatRate       = repeatRate;
  mpHfBuffer        = hfBuffer;
  mpLfBuffer        = lfBuffer;
  mFilterDepth      = filterDepth;
  mThresHold        = thresHold;
  //freezer          = 0;
  mSquelchOn        = false;
  mScanning         = false;
  mLgain            = 20;
  mRgain            = 20;

  mPeakLevelSampleMax = workingRate / 5;  // workingRate is typ. 48000Ss -> so eval each 9600 samples for 200ms for peak level meter
  mpMyRdsDecoder = NULL;

  this->mpLocalBuffer = new double[displaySize];
  //	we trust that neither displaySize nor SpectrumSize are 0
  //

  if ((spectrumSize & (spectrumSize - 1)) != 0)
  {
    this->mSpectrumSize = 4 * displaySize;
  }
  else
  {
    this->mSpectrumSize = spectrumSize;
  }

  this->mpSpectrum_fft_hf   = new common_fft(this->mSpectrumSize);
  this->mpSpectrumBuffer_hf = mpSpectrum_fft_hf->getVector();
  this->mpSpectrum_fft_lf   = new common_fft(this->mSpectrumSize);
  this->mpSpectrumBuffer_lf = mpSpectrum_fft_lf->getVector();

  this->mpLocalOscillator = new Oscillator(inputRate);
  this->mpMySinCos        = new SinCos(fmRate);
  this->mLoFrequency      = 0;
  this->mOmegaDemod       = 2 * M_PI / fmRate;
  this->mFmBandwidth     = 0.95 * fmRate;
  this->mFmFilterDegree  = 21;
  this->mpFmFilter        = new LowPassFIR(21, 0.95 * fmRate / 2, fmRate);
  this->mNewFilter       = false;
  /*
   *	default values, will be set through the user interface
   *	to their appropriate values
   */
  this->mFmModus        = FM_Mode::Stereo;
  this->mSelector       = S_STEREO;
  this->mInputMode      = IandQ;
  this->mpAudioDecimator = new newConverter(fmRate, workingRate, workingRate / 200);
  this->mpAudioOut = new DSPCOMPLEX[mpAudioDecimator->getOutputsize()];
  /*
   *	averagePeakLevel and audioGain are set
   *	prior to calling the processFM method
   */
  //this->peakLevel           = -100;
  //this->peakLevelcnt        = 0;
  this->mMaxFreqDeviation  = 0.95 * (0.5 * fmRate);
  this->mNormFreqDeviation = 0.6 * mMaxFreqDeviation;
  //this->audioGain           = 0;
  //
  mNoiseLevel = 0;
  mPilotLevel = 0;
  mRdsLevel   = 0;
  //	Since data is coming with a pretty high rate, we need to filter
  //	and decimate in an efficient way. We have an optimized
  //	decimating filter (optimized or not, it takes quite some
  //	cycles when entering with high rates)
  mpFmBandfilter = new DecimatingFIR(15 * mDecimatingScale, fmRate / 2, inputRate, mDecimatingScale);

  //	to isolate the pilot signal, we need a reasonable
  //	filter. The filtered signal is beautified by a pll
  mpPilotBandFilter = new fftFilter(FFT_SIZE, PILOTFILTER_SIZE);
  mpPilotBandFilter->setBand(PILOT_FREQUENCY - PILOT_WIDTH / 2, PILOT_FREQUENCY + PILOT_WIDTH / 2, fmRate);
  mpPilotRecover = new pilotRecovery(fmRate, OMEGA_PILOT, 25 * mOmegaDemod, mpMySinCos);
  mPilotDelay = (FFT_SIZE - PILOTFILTER_SIZE) * OMEGA_PILOT;

  mpRdsLowPassFilter = new fftFilter(FFT_SIZE, RDSLOWPASS_SIZE);
  mpRdsLowPassFilter->setLowPass(RDS_WIDTH, fmRate);
  //
  //	the constant K_FM is still subject to many questions
  DSPFLOAT F_G     = 0.65 * fmRate / 2;// highest freq in message
  DSPFLOAT Delta_F = 0.95 * fmRate / 2;//
  DSPFLOAT B_FM    = 2 * (Delta_F + F_G);

  mK_FM           = B_FM * M_PI / F_G;
  mpTheDemodulator = new fm_Demodulator(fmRate, mpMySinCos, mK_FM);
  mpFmAudioFilter  = NULL;
  //
  //	In the case of mono we do not assume a pilot
  //	to be available. We borrow the approach from CuteSDR
  mpRdsHilbertFilter =
    new HilbertFilter(HILBERT_SIZE, (DSPFLOAT)RDS_FREQUENCY / fmRate, fmRate);
  mpRdsBandFilter = new fftFilter(FFT_SIZE, RDSBANDFILTER_SIZE);
  mpRdsBandFilter->setSimple(RDS_FREQUENCY - RDS_WIDTH / 2,
                           RDS_FREQUENCY + RDS_WIDTH / 2, fmRate);
  mpRds_plldecoder = new pllC(fmRate, RDS_FREQUENCY, RDS_FREQUENCY - 50,
                            RDS_FREQUENCY + 50, 200, mpMySinCos);

  //	for the deemphasis we use an in-line filter with
  mXkm1     = 0;
  mYkml     = 0;
  mAlpha    = 1.0 / (fmRate / (1000000.0 / 50.0 + 1));
  mDumping  = false;
  mpDumpFile = NULL;

  connect(this, SIGNAL(hfBufferLoaded(void)), mMyRadioInterface, SLOT(hfBufferLoaded(void)));
  connect(this, SIGNAL(lfBufferLoaded(void)), mMyRadioInterface, SLOT(lfBufferLoaded(void)));
  connect(this, &fmProcessor::showPeakLevel, mMyRadioInterface, &RadioInterface::showPeakLevel);
  connect(this, SIGNAL(showStrength(float,float)), mMyRadioInterface, SLOT(showStrength(float,float)));
  connect(this, SIGNAL(scanresult(void)), mMyRadioInterface,SLOT(scanresult(void)));

  mSquelchValue     = 0;
  mOldSquelchValue = 0;

  mpTheConverter = NULL;
  if (audioRate != workingRate)
  {
    mpTheConverter = new newConverter(workingRate, audioRate, workingRate / 20);
  }
  mMyCount = 0;
}

fmProcessor::~fmProcessor()
{
  stop();

  //	delete	TheDemodulator;
  //	delete	rds_plldecoder;
  //	delete	pilotRecover;
  //	delete	rdsHilbertFilter;
  //	delete	rdsBandFilter;
  //	delete	pilotBandFilter;
  //	delete	audioDecimator;
  //	delete	mySinCos;
  //	delete	spectrum_fft_hf;
  //	delete	spectrum_fft_lf;
  if (mpFmAudioFilter != NULL)
  {
    delete mpFmAudioFilter;
  }
}

void fmProcessor::stop()
{
  if (mRunning)
  {
    mRunning = false;
    while (!isFinished())
    {
      usleep(100);
    }
  }
}

DSPFLOAT fmProcessor::get_pilotStrength()
{
  if (mRunning)
  {
    // get_db(0, 128) == 42.14dB
    return get_db(mPilotLevel, 128) - get_db(0, 128);
  }
  return 0.0;
}

DSPFLOAT fmProcessor::get_rdsStrength()
{
  if (mRunning)
  {
    return get_db(mRdsLevel, 128) - get_db(0, 128);
  }
  return 0.0;
}

DSPFLOAT fmProcessor::get_noiseStrength()
{
  if (mRunning)
  {
    return get_db(mNoiseLevel, 128) - get_db(0, 128);
  }
  return 0.0;
}

void fmProcessor::set_squelchValue(int16_t n)
{
  mSquelchValue = n;
}

DSPFLOAT fmProcessor::get_dcComponent()
{
  if (mRunning)
  {
    return mpTheDemodulator->get_DcComponent();
  }
  return 0.0;
}

fm_Demodulator::TDecoderListNames & fmProcessor::listNameofDecoder()
{
  return mpTheDemodulator->listNameofDecoder();
}

const char *fmProcessor::nameofDecoder()
{
  return mpTheDemodulator->nameofDecoder();
}
//
//	changing a filter is in two steps: here we set a marker,
//	but the actual filter is created in the mainloop of
//	the processor
void fmProcessor::setBandwidth(int32_t b)
{
  mFmBandwidth = b;
  mNewFilter   = true;
}

void fmProcessor::setBandfilterDegree(int32_t d)
{
  mFmFilterDegree = d;
  mNewFilter      = true;
}

void fmProcessor::setfmMode(FM_Mode m)
{
  mFmModus = m;
}

void fmProcessor::setFMdecoder(int16_t d)
{
  mpTheDemodulator->setDecoder(d);
}

void fmProcessor::setSoundMode(uint8_t selector)
{
  this->mSelector = selector;
}

void fmProcessor::setStereoPanorama(int16_t iStereoPan)
{
  // iStereoPan range: 0 (Mono) ... +100 (Stereo) ... +200 (Stereo with widen panorama)
  mPanorama = (DSPFLOAT)iStereoPan / 100.0f;
}

void fmProcessor::setSoundBalance(int16_t balance)
{
  // range: -100 <= balance <= +100
  this->mBalance = balance;
  //  leftChannel   = -(balance - 50.0) / 100.0;
  //  rightChannel  = (balance + 50.0) / 100.0;
  mLeftChannel  = (balance > 0 ? (100 - balance) / 100.0 : 1.0f);
  mRightChannel = (balance < 0 ? (100 + balance) / 100.0 : 1.0f);
}

//	Deemphasis	= 50 usec (3183 Hz, Europe)
//	Deemphasis	= 75 usec (2122 Hz US)
//	tau		= 2 * M_PI * Freq = 1000000 / time
void fmProcessor::setDeemphasis(int16_t v)
{
  Q_ASSERT(v >= 1);
  DSPFLOAT Tau;

  Tau   = 1000000.0 / v;
  mAlpha = 1.0 / (DSPFLOAT(mFmRate) / Tau + 1.0);
}

void fmProcessor::setVolume(const float iVolGainDb)
{
  mVolumeFactor = std::pow(10.0f, iVolGainDb / 20.0f);
}

DSPCOMPLEX fmProcessor::audioGainCorrection(DSPCOMPLEX z)
{
  const DSPFLOAT left  = mVolumeFactor * mLeftChannel * real(z);
  const DSPFLOAT right = mVolumeFactor * mRightChannel * imag(z);

#if 0
  {
    static DSPFLOAT leftAbsMax = -1e38f;
    static DSPFLOAT rightAbsMax = -1e38f;
    static DSPFLOAT lastVolume = 0.0f;
    bool printMaxValues = false;

    if (lastVolume != mVolumeFactor)
    {
      lastVolume = mVolumeFactor;
      leftAbsMax = rightAbsMax = -1e38f;
    }

    if (abs(left) > leftAbsMax)
    {
      leftAbsMax = abs(left);
      printMaxValues = true;
    }

    if (abs(right) > rightAbsMax)
    {
      rightAbsMax = abs(right);
      printMaxValues = true;
    }

    if (printMaxValues)
    {
      qInfo("leftAbsMax: %f, rightAbsMax: %f", leftAbsMax, rightAbsMax);
    }
  }
#endif

  return DSPCOMPLEX(left, right);
  //return DSPCOMPLEX(mVolumeFactor * leftChannel * real(z), mVolumeFactor * rightChannel * imag(z));
}

void fmProcessor::startDumping(SNDFILE *f)
{
  if (mDumping)
  {
    return;
  }
  //	do not change the order here, another thread might get confused
  mpDumpFile = f;
  mDumping  = true;
}

void fmProcessor::stopDumping(void)
{
  mDumping = false;
}

void fmProcessor::setAttenuation(int16_t l, int16_t r)
{
  mLgain = l;
  mRgain = r;
}

void fmProcessor::startScanning(void)
{
  mScanning = true;
}

void fmProcessor::stopScanning(void)
{
  mScanning = false;
}

//
//	In this variant, we have a separate thread for the
//	fm processing

void fmProcessor::run()
{
  DSPCOMPLEX    result;
  DSPFLOAT      rdsData;
  const int32_t bufferSize = 2 * 8192;
  DSPCOMPLEX    dataBuffer[bufferSize];
  double        displayBuffer_hf[mDisplaySize];
  double        displayBuffer_lf[mDisplaySize];
  DSPCOMPLEX    pcmSample;
  int32_t       hfCount = 0;
  int32_t       lfCount = 0;
  squelch       mySquelch(1, mWorkingRate / 10, mWorkingRate / 20, mWorkingRate);
  int32_t       audioAmount;
  //float         audioGainAverage = 0;
  int32_t       scanPointer      = 0;
  common_fft    *scan_fft        = new common_fft(1024);
  DSPCOMPLEX    *scanBuffer      = scan_fft->getVector();
  int           localP           = 0;

#define RDS_DECIMATOR    8
  mpMyRdsDecoder = new rdsDecoder(mMyRadioInterface, mFmRate / RDS_DECIMATOR, mpMySinCos);

  mRunning = true; // will be set from the outside

  while (mRunning)
  {
    while (mRunning && (mMyRig->Samples() < bufferSize))
    {
      msleep(1); // should be enough
    }

    if (!mRunning)
    {
      break;
    }

    //	First: update according to potentially changed settings
    if (mNewFilter && (mFmBandwidth < 0.95 * mFmRate))
    {
      delete mpFmFilter;
      mpFmFilter = new LowPassFIR(mFmFilterDegree, mFmBandwidth / 2, mFmRate);
    }
    mNewFilter = false;

    if (mSquelchValue != mOldSquelchValue)
    {
      mySquelch.setSquelchLevel(mSquelchValue);
      mOldSquelchValue = mSquelchValue;
    }

    const int32_t amount = mMyRig->getSamples(dataBuffer, bufferSize, mInputMode);
    const int32_t aa = (amount >= mSpectrumSize ? mSpectrumSize : amount);

    //	for the HFscope
    if (++hfCount > (mInputRate / bufferSize) / mRepeatRate)
    {
      double Y_Values[mDisplaySize];

      for (int32_t i = 0; i < aa; i++)
      {
        mpSpectrumBuffer_hf[i] = dataBuffer[i];
      }

      for (int32_t i = aa; i < mSpectrumSize; i++)
      {
        mpSpectrumBuffer_hf[i] = 0;
      }

      mpSpectrum_fft_hf->do_FFT();
      mapSpectrum(mpSpectrumBuffer_hf, Y_Values);

//      if (freezer > 0)
//      {
//        for (int32_t i = 0; i < displaySize; i++)
//        {
//          displayBuffer_hf[i] = 1.0 / freezer * Y_Values[i] + (freezer - 1.0) / freezer * displayBuffer_hf[i];
//        }
//        freezer++;
//      }
//      else
      {
        if (mFillAverageHfBuffer)
        {
          fill_average_buffer(Y_Values, displayBuffer_hf);
          mFillAverageHfBuffer = false;
        }
        else
        {
          add_to_average(Y_Values, displayBuffer_hf);
        }
      }

      mpHfBuffer->putDataIntoBuffer(displayBuffer_hf, mDisplaySize);
      hfCount = 0;

      // and signal the GUI thread that we have data
      emit hfBufferLoaded();
    }

    if (mDumping)
    {
      float dumpBuffer[2 * amount];

      for (int32_t i = 0; i < amount; i++)
      {
        dumpBuffer[2 * i]     = real(dataBuffer[i]);
        dumpBuffer[2 * i + 1] = imag(dataBuffer[i]);
      }
      sf_writef_float(mpDumpFile, dumpBuffer, amount);
    }
    //	Here we really start

    //	We assume that if/when the pilot is no more than 3 db's above
    //	the noise around it, it is better to decode mono
    for (int32_t i = 0; i < amount; i++)
    {
      DSPCOMPLEX v = DSPCOMPLEX(real(dataBuffer[i]) * mLgain, imag(dataBuffer[i]) *mRgain);
      v = v * mpLocalOscillator->nextValue(mLoFrequency);
      //
      //	first step: decimating (and filtering)
      if ((mDecimatingScale > 1) && !mpFmBandfilter->Pass(v, &v))
      {
        continue;
      }

      //	second step: if we are scanning, do the scan
      if (mScanning)
      {
        scanBuffer[scanPointer++] = v;

        if (scanPointer >= 1024)
        {
          scanPointer = 0;
          scan_fft->do_FFT();
          float signal = getSignal(scanBuffer, 1024);
          float Noise  = getNoise(scanBuffer, 1024);

          if (get_db(signal, 256) - get_db(Noise, 256) > this->mThresHold)
          {
            fprintf(stderr, "signal found %f %f\n", get_db(signal, 256), get_db(Noise, 256));
            emit scanresult();
          }
        }
        continue; // no signal processing!!!!
      }

      //	third step: if requested, apply filtering
      if (mFmBandwidth < 0.95 * mFmRate)
      {
        v = mpFmFilter->Pass(v);
      }
      //	Now we have the signal ready for decoding
      //	keep track of the peaklevel, we take segments
//      if (abs(v) > peakLevel)
//      {
//        peakLevel = abs(v);
//      }
//      if (++peakLevelcnt >= fmRate / 4)
//      {
//        DSPFLOAT ratio = (DSPFLOAT)max_freq_deviation / (DSPFLOAT)norm_freq_deviation;
//        if (peakLevel > 0)
//        {
//          this->audioGain = (ratio / peakLevel) / AUDIO_FREQ_DEV_PROPORTION;
//        }
//        if (audioGain <= 0.1)
//        {
//          audioGain = 0.1;
//        }
//        audioGain        = 0.99 * audioGainAverage + 0.01 * audioGain;
//        audioGainAverage = audioGain;
//        peakLevelcnt     = 0;
//        //	         fprintf (stderr, "peakLevel = %f\n", peakLevel);
//        peakLevel = -100;
//      }

      DSPFLOAT demod = mpTheDemodulator->demodulate(v);

      mpSpectrumBuffer_lf[localP++] = demod;

      if (localP >= mSpectrumSize)
      {
        localP = 0;
      }

      if (++lfCount > mFmRate / mRepeatRate)
      {
        double Y_Values[mDisplaySize];
        mpSpectrum_fft_lf->do_FFT();
        mapSpectrum(mpSpectrumBuffer_lf, Y_Values);

        if (mFillAverageLfBuffer)
        {
          fill_average_buffer(Y_Values, displayBuffer_lf);
          mFillAverageLfBuffer = false;
        }
        else
        {
          add_to_average(Y_Values, displayBuffer_lf);
        }

        extractLevels(displayBuffer_lf, mFmRate);
        mpLfBuffer->putDataIntoBuffer(displayBuffer_lf, mDisplaySize);
        lfCount = 0;
        //	and signal the GUI thread that we have data
        emit lfBufferLoaded();
      }

      if (mFmModus != FM_Mode::Mono)
      {
        stereo(demod, &result, &rdsData);

        const DSPFLOAT sumLR  = real(result);
        const DSPFLOAT diffLR = imag(result);
        const DSPFLOAT diffLRWeightend = diffLR * (mFmModus == FM_Mode::StereoPano ? mPanorama : 1.0f);

        const DSPFLOAT left  = sumLR + diffLRWeightend;  // 2L = (L+R) + (L-R)
        const DSPFLOAT right = sumLR - diffLRWeightend;  // 2R = (L+R) - (L-R)

        switch (mSelector)
        {
        default:
        case S_STEREO:
          result = DSPCOMPLEX(left, right);
          break;

        case S_LEFT:
          result = DSPCOMPLEX(left, left);
          break;

        case S_RIGHT:
          result = DSPCOMPLEX(right, right);
          break;

        case S_LEFTplusRIGHT:
          result = DSPCOMPLEX(sumLR, sumLR);
          break;

        case S_LEFTminusRIGHT:
          result = DSPCOMPLEX(diffLRWeightend, diffLRWeightend);
          break;
        }
      }
      else
      {
        mono(demod, &result, &rdsData);
      }

      if (mpFmAudioFilter != nullptr)
      {
        result = mpFmAudioFilter->Pass(result);
      }

      // "result" now contains the audio sample, either stereo or mono
      result = audioGainCorrection(result);

      if (mpAudioDecimator->convert(result, mpAudioOut, &audioAmount))
      {
        // here the sample rate is "workingRate" (typ. 48000Ss)
        for (int32_t k = 0; k < audioAmount; k++)
        {
          if (mSquelchOn)
          {
            pcmSample = mySquelch.do_squelch(mpAudioOut[k]);
          }
          else
          {
            pcmSample = mpAudioOut[k];
          }

          evaluatePeakLevel(pcmSample);
          sendSampletoOutput(pcmSample);
        }
      }

      if ((mRdsModus != rdsDecoder::NO_RDS))
      {
        static int cnt = 0;

        if (++cnt >= RDS_DECIMATOR)
        {
          DSPFLOAT mag;
          mpMyRdsDecoder->doDecode(rdsData, &mag, (rdsDecoder::RdsMode)1);
          cnt = 0;
        }
      }

      if (++mMyCount > mFmRate) // each second ...
      {
        mMyCount = 0;
        emit showStrength(get_pilotStrength(), get_dcComponent());
      }
    }
  }
}

void fmProcessor::mono(float demod, DSPCOMPLEX *audioOut, DSPFLOAT *rdsValue)
{
  DSPFLOAT   Re, Im;
  DSPCOMPLEX rdsBase;

  //	deemphasize
  Re        = mXkm1 = (demod - mXkm1) * mAlpha + mXkm1;
  Im        = mYkml = (demod - mYkml) * mAlpha + mYkml;
  *audioOut = DSPCOMPLEX(Re, Im);
  //
  //	fully inspired by cuteSDR, we try to decode the rds stream
  //	by simply am decoding it (after creating a decent complex
  //	signal by Hilbert filtering)
  rdsBase = DSPCOMPLEX(5 * demod, 5 * demod);
  rdsBase = mpRdsHilbertFilter->Pass(mpRdsBandFilter->Pass(rdsBase));
  mpRds_plldecoder->do_pll(rdsBase);
  DSPFLOAT rdsDelay = imag(mpRds_plldecoder->getDelay());

  *rdsValue = mpRdsLowPassFilter->Pass(5 * rdsDelay);
}

void fmProcessor::stereo(float demod, DSPCOMPLEX *audioOut,
                         DSPFLOAT *rdsValue)
{
  DSPFLOAT LRPlus = 0;
  DSPFLOAT LRDiff = 0;
  DSPFLOAT pilot  = 0;
  DSPFLOAT currentPilotPhase;
  DSPFLOAT PhaseforLRDiff = 0;
  DSPFLOAT PhaseforRds    = 0;

  /*
   */
  LRPlus = LRDiff = pilot = demod;
  /*
   *	get the phase for the "carrier to be inserted" right
   */
  pilot             = mpPilotBandFilter->Pass(5 * pilot);
  currentPilotPhase = mpPilotRecover->getPilotPhase(5 * pilot);
  /*
   *	Now we have the right - i.e. synchronized - signal to work with
   */
  PhaseforLRDiff = 2 * (currentPilotPhase + mPilotDelay);
  PhaseforRds    = 3 * (currentPilotPhase + mPilotDelay);
  //
  //	Due to filtering the real amplitude of the LRDiff might have
  //	to be adjusted, we guess
  LRDiff = 2.0 * mpMySinCos->getCos(PhaseforLRDiff) * LRDiff;
  DSPFLOAT MixerValue = mpMySinCos->getCos(PhaseforRds);

  *rdsValue = 5 * mpRdsLowPassFilter->Pass(MixerValue * demod);

  //	apply deemphasis
  LRPlus    = mXkm1 = (LRPlus - mXkm1) * mAlpha + mXkm1;
  LRDiff    = mYkml = (LRDiff - mYkml) * mAlpha + mYkml;

  if (mpPilotRecover->isLocked() || mAutoMono == false)
  {
    *audioOut = DSPCOMPLEX(LRPlus, LRDiff);
  }
  else
  {
    *audioOut = DSPCOMPLEX(LRPlus, 0); // force to mono audio (TODO: what is with RDS when pilot is unlocked?)
  }
}
//
//	Since setLFcutoff is only called from within the "run"  function
//	from where the filter is also called, it is safe to remove
//	it here
//
void fmProcessor::setLFcutoff(int32_t Hz)
{
  LowPassFIR *tempFmAudioFilter = mpFmAudioFilter;

  mpFmAudioFilter = nullptr; // set to null first due to other thread is using pointer while deletion

  delete tempFmAudioFilter;

  if (Hz > 0)
  {
    mpFmAudioFilter = new LowPassFIR(55, Hz, mFmRate);  // 11 is too less (55 is also arbitrary) for this high sample rate fmRate = 256000S/s
  }
}

void fmProcessor::evaluatePeakLevel(const DSPCOMPLEX s)
{
  const DSPFLOAT absLeft  = std::abs(real(s));
  const DSPFLOAT absRight = std::abs(imag(s));

  if (absLeft  > mAbsPeakLeft)  mAbsPeakLeft  = absLeft;
  if (absRight > mAbsPeakRight) mAbsPeakRight = absRight;

  if (++mPeakLevelCurSampleCnt > mPeakLevelSampleMax)
  {
    mPeakLevelCurSampleCnt = 0;

    const float leftDb  = (mAbsPeakLeft  > 0.0f ? 20.0f * std::log10(mAbsPeakLeft)  : -40.0f);
    const float rightDb = (mAbsPeakRight > 0.0f ? 20.0f * std::log10(mAbsPeakRight) : -40.0f);

    emit showPeakLevel(leftDb, rightDb);

    mAbsPeakLeft = mAbsPeakRight = 0.0f;
  }
}

void fmProcessor::sendSampletoOutput(DSPCOMPLEX s)
{
  if (mAudioRate == mWorkingRate)
  {
    mAudioSink->putSample(s);
  }
  else
  {
    DSPCOMPLEX out[mpTheConverter->getOutputsize()];
    int32_t    amount;
    if (mpTheConverter->convert(s, out, &amount))
    {
      for (int32_t i = 0; i < amount; i++)
      {
        mAudioSink->putSample(out[i]);
      }
    }
  }
}

void fmProcessor::setfmRdsSelector(int8_t m)
{
  mRdsModus = m;
}

void fmProcessor::resetRds()
{
  if (mpMyRdsDecoder == NULL)
  {
    return;
  }
  mpMyRdsDecoder->reset();
}

void fmProcessor::set_localOscillator(int32_t lo)
{
  mLoFrequency = lo;
}

bool fmProcessor::ok()
{
  return mRunning;
}

bool fmProcessor::isPilotLocked(float & oLockStrength) const
{
  if (mFmModus != FM_Mode::Mono && mpPilotRecover)
  {
    oLockStrength = mpPilotRecover->getLockedStrength();
    return mpPilotRecover->isLocked();
  }
  else
  {
    oLockStrength = 0;
    return false;
  }
}

//void fmProcessor::setFreezer(bool b)
//{
//  freezer = (b ? 10 : 0);
//}

void fmProcessor::set_squelchMode(bool b)
{
  mSquelchOn = b;
}

void fmProcessor::setInputMode(uint8_t m)
{
  mInputMode = m;
}

DSPFLOAT fmProcessor::getSignal(DSPCOMPLEX *v, int32_t size)
{
  DSPFLOAT sum = 0;
  int16_t  i;

  for (i = 5; i < 25; i++)
  {
    sum += abs(v[i]);
  }
  for (i = 5; i < 25; i++)
  {
    sum += abs(v[size - 1 - i]);
  }
  return sum / 40;
}

DSPFLOAT fmProcessor::getNoise(DSPCOMPLEX *v, int32_t size)
{
  DSPFLOAT sum = 0;
  int16_t  i;

  for (i = 5; i < 25; i++)
  {
    sum += abs(v[size / 2 - 1 - i]);
  }
  for (i = 5; i < 25; i++)
  {
    sum += abs(v[size / 2 + 1 + i]);
  }
  return sum / 40;
}

void fmProcessor::mapSpectrum(const DSPCOMPLEX * const in, double * const out)
{
  int i, j;

  for (i = 0; i < mDisplaySize / 2; i++)
  {
    int16_t factor = mSpectrumSize / mDisplaySize;
    double  f      = 0;
    for (j = 0; j < factor; j++)
    {
      f += abs(in[i * factor + j]);
    }
    out[mDisplaySize / 2 + i] = f / factor;
    f                        = 0;
    for (j = 0; j < factor; j++)
    {
      f += abs(in[mSpectrumSize / 2 + factor * i + j]);
    }
    out[i] = f / factor;
  }
}

void fmProcessor::fill_average_buffer(const double * const in, double * const buffer)
{
  for (int32_t i = 0; i < mDisplaySize; i++)
  {
    buffer[i] = in[i];
  }
}

void fmProcessor::add_to_average(const double * const in, double * const buffer)
{
  for (int32_t i = 0; i < mDisplaySize; i++)
  {
    buffer[i] = 1.0 / mAverageCount * in[i] + (mAverageCount - 1.0) / mAverageCount * buffer[i];
  }
}

void fmProcessor::extractLevels(double *in, int32_t range)
{
  float binWidth    = (float)range / mDisplaySize;
  int   pilotOffset = mDisplaySize / 2 - 19000 / binWidth;
  int   rdsOffset   = mDisplaySize / 2 - 57000 / binWidth;
  int   noiseOffset = mDisplaySize / 2 - 70000 / binWidth;
  int   i;
  int   a = mMyRig->bitDepth() - 1;
  int   b = 1;

  while (--a > 0)
  {
    b <<= 1;
  }
  float temp1 = 0, temp2 = 0, temp3 = 0;

  for (i = 0; i < 7; i++)
  {
    temp1 += in[noiseOffset - 3 + i];
    temp3 += in[rdsOffset - 3 + i];
  }
  for (i = 0; i < 3; i++)
  {
    temp2 += in[pilotOffset - 1 + i];
  }
  mNoiseLevel = 0.95 * mNoiseLevel + 0.05 * temp1 / 7;
  mPilotLevel = 0.95 * mPilotLevel + 0.05 * temp2 / 3;
  mRdsLevel   = 0.95 * mRdsLevel   + 0.05 * temp3 / 7;
}
