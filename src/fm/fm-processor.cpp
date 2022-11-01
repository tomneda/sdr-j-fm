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
//#include "rds-decoder.h"
#include "sincos.h"

//#define AUDIO_FREQ_DEV_PROPORTION    0.85f
#define PILOT_FREQUENCY              19000
#define RDS_FREQUENCY                (3 * PILOT_FREQUENCY)
#define RDS_RATE                     19000   // 16 samples for one RDS sympols
//#define OMEGA_DEMOD                  (2 * M_PI / fmRate)
#define OMEGA_PILOT                  ((DSPFLOAT(PILOT_FREQUENCY)) / fmRate) * (2 * M_PI)
//#define OMEGA_RDS                    ((DSPFLOAT)RDS_FREQUENCY / fmRate) * (2 * M_PI)
//#define RDS_DECIMATOR                8

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
  mSpectrumSampleRate = fmRate;
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
  mSquelchMode      = ESqMode::OFF;
  mScanning         = false;
  mLgain            = 20;
  mRgain            = 20;

  mPeakLevelSampleMax = workingRate / 5;  // workingRate is typ. 48000Ss -> so eval each 9600 samples for 200ms for peak level meter
  mpMyRdsDecoder = nullptr;

  mpLocalBuffer = new double[displaySize];
  //	we trust that neither displaySize nor SpectrumSize are 0
  //

  mSpectrumSize = ((spectrumSize & (spectrumSize - 1)) != 0 ? 4 * displaySize : spectrumSize);
  mpSpectrum_fft_hf   = new common_fft(mSpectrumSize);
  mpSpectrumBuffer_hf = mpSpectrum_fft_hf->getVector();
  mpSpectrum_fft_lf   = new common_fft(mSpectrumSize);
  mpSpectrumBuffer_lf.set_data_ptr(mpSpectrum_fft_lf->getVector(), mSpectrumSize);

  mpLocalOscillator = new Oscillator(inputRate);
  mpRdsOscillator   = new Oscillator(fmRate);
  mpMySinCos        = new SinCos(fmRate);
  mLoFrequency      = 0;
  mOmegaDemod       = 2 * M_PI / fmRate;
  mFmBandwidth      = 0.95 * fmRate;
  mFmFilterDegree   = 21;
  mpFmFilter        = new LowPassFIR(21, 0.95 * fmRate / 2, fmRate);
  mNewFilter        = false;
  /*
   *	default values, will be set through the user interface
   *	to their appropriate values
   */
  mFmModus        = FM_Mode::Stereo;
  mSelector       = S_STEREO;
  mInputMode      = IandQ;
  //mpAudioDecimator = new newConverter(fmRate, workingRate, workingRate / 200);
  mpAudioDecimator = new newConverter(fmRate, workingRate, fmRate / 1000);
  mpAudioOut = new DSPCOMPLEX[mpAudioDecimator->getOutputsize()];

  mMaxFreqDeviation  = 0.95 * (0.5 * fmRate);
  mNormFreqDeviation = 0.6 * mMaxFreqDeviation;
  //this->audioGain           = 0;
  //

#ifdef USE_EXTRACT_LEVELS
  mNoiseLevel = 0;
  mPilotLevel = 0;
  mRdsLevel   = 0;
#endif

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
  mpFmAudioFilter = new fftFilter(1024, 431);
  mFmAudioFilterActive = false;

  mpRdsLowPassFilter = new fftFilter(FFT_SIZE, RDSLOWPASS_SIZE);
  mpRdsLowPassFilter->setLowPass(RDS_WIDTH, fmRate);
  mpRdsDecimator = new newConverter(fmRate, RDS_RATE, fmRate / 1000);
  mpRdsOut = new DSPCOMPLEX[mpRdsDecimator->getOutputsize()];

  //
  //	the constant K_FM is still subject to many questions
  DSPFLOAT F_G     = 0.65 * fmRate / 2;// highest freq in message
  DSPFLOAT Delta_F = 0.95 * fmRate / 2;//
  DSPFLOAT B_FM    = 2 * (Delta_F + F_G);

  mK_FM           = B_FM * M_PI / F_G;
  mpTheDemodulator = new fm_Demodulator(fmRate, mpMySinCos, mK_FM);
  //
  //	In the case of mono we do not assume a pilot
  //	to be available. We borrow the approach from CuteSDR
  //mpRdsHilbertFilter = new HilbertFilter(HILBERT_SIZE, (DSPFLOAT)RDS_FREQUENCY / fmRate, fmRate);
  mpRdsHilbertFilter = new fftFilterHilbert(FFT_SIZE, RDSBANDFILTER_SIZE);

  mpRdsBandFilter = new fftFilter(FFT_SIZE, RDSBANDFILTER_SIZE);
  mpRdsBandFilter->setSimple(RDS_FREQUENCY - RDS_WIDTH / 2, RDS_FREQUENCY + RDS_WIDTH / 2, fmRate);

  //mpRds_plldecoder = new pllC(fmRate, RDS_FREQUENCY, RDS_FREQUENCY - 50, RDS_FREQUENCY + 50, 200, mpMySinCos);
  //mRdsSampleCnt = 0;

  // for the deemphasis we use an in-line filter with
  mLastAudioSample = 0;
  mDeemphAlpha = 1.0 / (fmRate / (1000000.0 / 50.0 + 1));

  mDumping  = false;
  mpDumpFile = nullptr;

  mMySquelch = new squelch(1, 70000, mFmRate / 20, mFmRate);

  mpDisplayBuffer_lf = new double[mDisplaySize];

  connect(mMySquelch, &squelch::setSquelchIsActive, mMyRadioInterface, &RadioInterface::setSquelchIsActive);
  connect(this, &fmProcessor::hfBufferLoaded, mMyRadioInterface, &RadioInterface::hfBufferLoaded);
  connect(this, &fmProcessor::lfBufferLoaded, mMyRadioInterface, &RadioInterface::lfBufferLoaded);
  //connect(this, SIGNAL(lfBufferLoaded(uint32_t,bool)), mMyRadioInterface, SLOT(lfBufferLoaded(uint32_t,bool)));
  connect(this, &fmProcessor::showPeakLevel, mMyRadioInterface, &RadioInterface::showPeakLevel);
  connect(this, SIGNAL(showDcComponents(float,float)), mMyRadioInterface, SLOT(showDcComponents(float,float)));
  connect(this, SIGNAL(scanresult()), mMyRadioInterface,SLOT(scanresult()));

  mSquelchValue     = 0;
  mOldSquelchValue = 0;

  mpTheConverter = nullptr;
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
  delete[] mpDisplayBuffer_lf;
  delete mpFmAudioFilter;
  delete mMySquelch;
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

#ifdef USE_EXTRACT_LEVELS

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
#endif

void fmProcessor::set_squelchValue(int16_t n)
{
  mSquelchValue = n;
}

DSPFLOAT fmProcessor::get_demodDcComponent()
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

void fmProcessor::setLfPlotType(ELfPlot m)
{
  mLfPlotType = m;
  mShowFullSpectrum = (m == ELfPlot::IF_FILTERED || m == ELfPlot::RDS);
  mSpectrumSampleRate = (m == ELfPlot::RDS ? RDS_RATE : mFmRate);
  triggerDrawNewLfSpectrum();
}

void fmProcessor::setLfPlotZoomFactor(int32_t iZoomFactor)
{
  mZoomFactor = iZoomFactor;
  triggerDrawNewLfSpectrum();
}

void fmProcessor::setFMdecoder(int16_t d)
{
  mpTheDemodulator->setDecoder(d);
}

void fmProcessor::setSoundMode(uint8_t selector)
{
  mSelector = selector;
}

void fmProcessor::setStereoPanorama(int16_t iStereoPan)
{
  // iStereoPan range: 0 (Mono) ... +100 (Stereo) ... +200 (Stereo with widen panorama)
  mPanorama = (DSPFLOAT)iStereoPan / 100.0f;
}

void fmProcessor::setSoundBalance(int16_t balance)
{
  // range: -100 <= balance <= +100
  mBalance = balance;
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
  mDeemphAlpha = 1.0 / (DSPFLOAT(mFmRate) / Tau + 1.0);
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

  return { left, right };
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
  const int32_t bufferSize = 2 * 8192;
  DSPCOMPLEX    dataBuffer[bufferSize];
  double        displayBuffer_hf[mDisplaySize];
  int32_t       mHfCount = 0;
  int32_t       mLfCount = 0;
  //float         audioGainAverage = 0;
  int32_t       scanPointer      = 0;
  common_fft    *scan_fft        = new common_fft(1024);
  DSPCOMPLEX    *scanBuffer      = scan_fft->getVector();
  const float rfDcAlpha = 1.0f / mInputRate;

  assert(mpMyRdsDecoder == nullptr); // check whether not calling next news twice
  //mpMyRdsDecoder = new rdsDecoder(mMyRadioInterface, mFmRate / RDS_DECIMATOR, mpMySinCos);
  mpMyRdsDecoder = new rdsDecoder(mMyRadioInterface, RDS_RATE, mpMySinCos);

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
      mMySquelch->setSquelchLevel(mSquelchValue);
      mOldSquelchValue = mSquelchValue;
    }


    const int32_t amount = mMyRig->getSamples(dataBuffer, bufferSize, mInputMode);
    const int32_t aa = (amount >= mSpectrumSize ? mSpectrumSize : amount);

    if (mDCREnabled)
    {
      for (int32_t i = 0; i < amount; i++)
      {
        mRfDC = (dataBuffer[i] - mRfDC) * rfDcAlpha + mRfDC;

        // limit the maximum DC correction because an AM carrier at exactly 0Hz could has been suppressed, too
        constexpr DSPFLOAT DCRlimit = 0.01f;
        DSPFLOAT rfDcReal = real(mRfDC);
        DSPFLOAT rfDcImag = imag(mRfDC);
        if      (rfDcReal > +DCRlimit) rfDcReal = +DCRlimit;
        else if (rfDcReal < -DCRlimit) rfDcReal = -DCRlimit;
        if      (rfDcImag > +DCRlimit) rfDcImag = +DCRlimit;
        else if (rfDcImag < -DCRlimit) rfDcImag = -DCRlimit;

        dataBuffer[i] -= DSPCOMPLEX(rfDcReal, rfDcImag);
      }
    }

    //	for the HFscope
    if (++mHfCount > (mInputRate / bufferSize) / mRepeatRate)
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

      int32_t zoomFactor = 1;
      mapSpectrum(mpSpectrumBuffer_hf, Y_Values, zoomFactor);

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
      mHfCount = 0;

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
      DSPCOMPLEX v = DSPCOMPLEX(real(dataBuffer[i]) * mLgain, imag(dataBuffer[i]) * mRgain);

      v = v * mpLocalOscillator->nextValue(mLoFrequency);

      // first step: decimating (and filtering)
      if ((mDecimatingScale > 1) && !mpFmBandfilter->Pass(v, &v))
      {
        continue;
      }

      // second step: if we are scanning, do the scan
      if (mScanning)
      {
        scanBuffer[scanPointer++] = v;

        if (scanPointer >= 1024)
        {
          scanPointer = 0;
          scan_fft->do_FFT();
          float signal = getSignal(scanBuffer, 1024);
          float Noise  = getNoise(scanBuffer, 1024);

          if (get_db(signal, 256) - get_db(Noise, 256) > mThresHold)
          {
            fprintf(stderr, "signal found %f %f\n", get_db(signal, 256), get_db(Noise, 256));
            emit scanresult();
          }
        }
        continue; // no signal processing!!!!
      }

      // third step: if requested, apply filtering
      if (mFmBandwidth < 0.95 * mFmRate)
      {
        v = mpFmFilter->Pass(v);
      }

      DSPFLOAT demod = mpTheDemodulator->demodulate(v);


      switch (mSquelchMode)
      {
      case ESqMode::NSQ: demod = mMySquelch->do_noise_squelch(demod); break;
      case ESqMode::LSQ: demod = mMySquelch->do_level_squelch(demod, mpTheDemodulator->get_carrier_ampl()); break;
      default:;
      }

      DSPCOMPLEX audio;
      DSPFLOAT rdsData;
      DSPCOMPLEX rdsDataCmpl;

      process_stereo_or_mono_with_rds(demod, &audio, &rdsData, &rdsDataCmpl);

      const DSPFLOAT sumLR  = real(audio);
      const DSPFLOAT diffLR = imag(audio);
      const DSPFLOAT diffLRWeightend = diffLR * (mFmModus == FM_Mode::StereoPano ? mPanorama : 1.0f);

      const DSPFLOAT left  = sumLR + diffLRWeightend;  // 2L = (L+R) + (L-R)
      const DSPFLOAT right = sumLR - diffLRWeightend;  // 2R = (L+R) - (L-R)

      switch (mSelector)
      {
      default:
      case S_STEREO:         audio = DSPCOMPLEX(left,  right); break;
      case S_STEREO_SWAPPED: audio = DSPCOMPLEX(right, left); break;
      case S_LEFT:           audio = DSPCOMPLEX(left,  left); break;
      case S_RIGHT:          audio = DSPCOMPLEX(right, right); break;
      case S_LEFTplusRIGHT:  audio = DSPCOMPLEX(sumLR, sumLR); break;
      case S_LEFTminusRIGHT: audio = DSPCOMPLEX(diffLRWeightend, diffLRWeightend); break;
      }

      if (mRdsModus != rdsDecoder::ERdsMode::NO_RDS)
      {
        int32_t rdsAmount;

        //int abc = mpRdsDecimator->getOutputsize();
        //++mRdsSampleCntSrc;

        if (mpRdsDecimator->convert(rdsDataCmpl, mpRdsOut, &rdsAmount))
        {
          //mRdsSampleCntDst += rdsAmount;
          //double ratio = (double)mRdsSampleCntDst / (double)mRdsSampleCntSrc;
          //double rate = ratio * mFmRate;

          // here the sample rate is rdsRate (typ. 19000S/s)
          for (int32_t k = 0; k < rdsAmount; k++)
          {
            const DSPCOMPLEX pcmSample = mpRdsOut[k];

            if ((mRdsModus != rdsDecoder::ERdsMode::RDS3))
            {
              DSPFLOAT mag;
              mpMyRdsDecoder->doDecode(imag(pcmSample), &mag, mRdsModus); // data rate 19000S/s
              if (mLfPlotType == ELfPlot::RDS) { mpSpectrumBuffer_lf = mag; }
            }
            else
            {
              DSPCOMPLEX magCplx;
              mpMyRdsDecoder->doDecode(pcmSample, &magCplx); // data rate 19000S/s
              if (mLfPlotType == ELfPlot::RDS) { mpSpectrumBuffer_lf = magCplx; }
            }
          }
        }
      }
      else
      {
        if (mLfPlotType == ELfPlot::RDS) { mpSpectrumBuffer_lf = 0; }
      }

      if (mFmAudioFilterActive)
      {
        audio = mpFmAudioFilter->Pass(audio);
      }

      //	apply deemphasis
      audio = mLastAudioSample = (audio - mLastAudioSample) * mDeemphAlpha + mLastAudioSample;

      switch (mLfPlotType)
      {
      case ELfPlot::OFF:               mpSpectrumBuffer_lf = 0; break;
      case ELfPlot::IF_FILTERED:       mpSpectrumBuffer_lf = v; break;
      case ELfPlot::DEMODULATOR:       mpSpectrumBuffer_lf = demod; break;
      case ELfPlot::AF_SUM:            mpSpectrumBuffer_lf = sumLR; break;
      case ELfPlot::AF_DIFF:           mpSpectrumBuffer_lf = diffLR; break;
      case ELfPlot::AF_MONO_FILTERED:  mpSpectrumBuffer_lf = (audio.real() + audio.imag()); break;
      case ELfPlot::AF_LEFT_FILTERED:  mpSpectrumBuffer_lf = audio.real(); break;
      case ELfPlot::AF_RIGHT_FILTERED: mpSpectrumBuffer_lf = audio.imag(); break;
      //case ELfPlot::RDS:               mpSpectrumBuffer_lf = rdsDataCmpl; break;
      default:;
      }

      // "result" now contains the audio sample, either stereo or mono
      audio = audioGainCorrection(audio);

      int32_t audioAmount;
      if (mpAudioDecimator->convert(audio, mpAudioOut, &audioAmount))
      {
        // here the sample rate is "workingRate" (typ. 48000Ss)
        for (int32_t k = 0; k < audioAmount; k++)
        {
          const DSPCOMPLEX pcmSample = mpAudioOut[k];
          evaluatePeakLevel(pcmSample);
          sendSampletoOutput(pcmSample);
        }
      }

      if (++mLfCount > (mFmRate / mRepeatRate) && mpSpectrumBuffer_lf.is_full())
      {
        processLfSpectrum();
        mpSpectrumBuffer_lf.reset_write_pointer();
        mLfCount = 0;
      }

      if (++mMyCount > (mFmRate >> 1)) // each 500ms ...
      {
#ifdef USE_EXTRACT_LEVELS
        emit showDcComponents((mDCREnabled ? 20 * log10(abs(mRfDC) + 1.0f/32768) : get_pilotStrength()), get_demodDcComponent());
#else
        emit showDcComponents((mDCREnabled ? 20 * log10(abs(mRfDC) + 1.0f/32768) : -99.9), get_demodDcComponent());
#endif
        //emit showStrength( 20 * log10(mpTheDemodulator->get_carrier_ampl()), get_demodDcComponent());
        mMyCount = 0;
      }
    }
  }
}

void fmProcessor::process_stereo_or_mono_with_rds(const float demod, DSPCOMPLEX *audioOut, DSPFLOAT *rdsValue, DSPCOMPLEX *rdsValueCmpl)
{
  // Get the phase for the "carrier to be inserted" right.
  // Do this alwas to be able to check of a locked pilot PLL.
  const DSPFLOAT pilot = mpPilotBandFilter->Pass(5 * demod);
  const DSPFLOAT currentPilotPhase = mpPilotRecover->getPilotPhase(5 * pilot);

  if (mFmModus != FM_Mode::Mono && (mpPilotRecover->isLocked() || mAutoMono == false))
  {
    // Now we have the right - i.e. synchronized - signal to work with
    const DSPFLOAT PhaseforLRDiff = 2 * (currentPilotPhase + mPilotDelay);
    //const DSPFLOAT PhaseforRds = 3 * (currentPilotPhase + mPilotDelay);

    //	Due to filtering the real amplitude of the LRDiff might have
    //	to be adjusted, we guess
    DSPFLOAT LRDiff = 2.0 * mpMySinCos->getCos(PhaseforLRDiff) * demod;
    //const DSPFLOAT MixerValue = mpMySinCos->getCos(PhaseforRds);

    //*rdsValue = mpRdsLowPassFilter->Pass(5 * MixerValue * demod);

    DSPFLOAT LRPlus = demod;

    *audioOut = DSPCOMPLEX(LRPlus, LRDiff);
  }
  else
  {
    *audioOut = DSPCOMPLEX(demod, 0);
  }

  // process RDS
  {
    const DSPFLOAT rdsBaseBp = mpRdsBandFilter->Pass(5 * demod);
    const DSPCOMPLEX rdsBaseHilb = mpRdsHilbertFilter->Pass(rdsBaseBp);
    //mpRds_plldecoder->do_pll(rdsBaseHilb);
    //DSPCOMPLEX rdsDelayCplx = mpRds_plldecoder->getDelay();

    DSPCOMPLEX rdsDelayCplx = rdsBaseHilb * mpRdsOscillator->nextValue(RDS_FREQUENCY); // the oscillator works other direction (== -57000 Hz shift)
    //rdsDelayCplx = mpRdsLowPassFilter->Pass(rdsDelayCplx);

    DSPFLOAT rdsDelay = imag(rdsDelayCplx);
    //*rdsValue = mpRdsLowPassFilter->Pass(rdsDelay);
    *rdsValue = rdsDelay;
    //*rdsValueCmpl = *rdsValue;
    //*rdsValueCmpl = rdsDelay;
    *rdsValueCmpl = rdsDelayCplx;
  }
}
//
//	Since setLFcutoff is only called from within the "run"  function
//	from where the filter is also called, it is safe to remove
//	it here
//
void fmProcessor::setLFcutoff(int32_t Hz)
{
  if (Hz > 0)
  {
    mFmAudioFilterActive = false; // in case the filter routine works in an extra thread
    mpFmAudioFilter->setLowPass(Hz, mFmRate);
    mFmAudioFilterActive = true;
  }
  else
  {
    mFmAudioFilterActive = false;
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

void fmProcessor::setfmRdsSelector(rdsDecoder::ERdsMode m)
{
  mRdsModus = m;

  if (mLfPlotType == ELfPlot::RDS)
  {
    triggerDrawNewLfSpectrum();
  }
}

void fmProcessor::resetRds()
{
  if (mpMyRdsDecoder == nullptr)
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

void fmProcessor::set_squelchMode(ESqMode iSqMode)
{
  mSquelchMode = iSqMode;
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

void fmProcessor::mapSpectrum(const DSPCOMPLEX * const in, double * const out, int32_t &ioZoomFactor)
{
  int16_t factor = mSpectrumSize / mDisplaySize;  // typ factor = 4 (whole divider)

  if (factor / ioZoomFactor >= 1)
  {
    factor /= ioZoomFactor;
  }
  else
  {
    ioZoomFactor = factor;
    factor = 1;
  }

  // work from inside (0Hz) to outside for filling display data
  for (int32_t i = 0; i < mDisplaySize / 2; i++)
  {
    double f = 0;

    for (int32_t j = 0; j < factor; j++)
    {
      f += abs(in[i * factor + j]); // read 0Hz to rate/2 -> map to mid to end of display
    }

    out[mDisplaySize / 2 + i] = f / factor;

    f = 0;

    for (int32_t j = 0; j < factor; j++)
    {
      f += abs(in[mSpectrumSize - 1 - (i * factor + j)]); // read rate/2 down to 0Hz -> map to begin to mid of display
    }

    out[mDisplaySize / 2 - 1 - i] = f / factor;
  }
}

void fmProcessor::mapHalfSpectrum(const DSPCOMPLEX * const in, double * const out, int32_t &ioZoomFactor)
{
  int16_t factor = mSpectrumSize / mDisplaySize / 2;  // typ factor = 2 (whole divider)

  if (factor / ioZoomFactor >= 1)
  {
    factor /= ioZoomFactor;
  }
  else
  {
    ioZoomFactor = factor;
    factor = 1;
  }

  for (int32_t i = 0; i < mDisplaySize; i++)
  {
    double f = 0;

    for (int32_t j = 0; j < factor; j++)
    {
      f += abs(in[i * factor + j]); // read 0Hz to rate/2 -> map to mid to end of display
    }

    out[i] = f / factor;
  }
}

void fmProcessor::processLfSpectrum()
{
  double Y_Values[mDisplaySize];

  mpSpectrum_fft_lf->do_FFT();

  int32_t zoomFactor = mZoomFactor; // copy value because it may be changed

  if (mShowFullSpectrum)
  {
    mapSpectrum(mpSpectrumBuffer_lf.get_ptr(), Y_Values, zoomFactor);
  }
  else
  {
    mapHalfSpectrum(mpSpectrumBuffer_lf.get_ptr(), Y_Values, zoomFactor);
  }

  if (mFillAverageLfBuffer)
  {
    fill_average_buffer(Y_Values, mpDisplayBuffer_lf);
    mFillAverageLfBuffer = false;
  }
  else
  {
    add_to_average(Y_Values, mpDisplayBuffer_lf);
  }

#ifdef USE_EXTRACT_LEVELS
  if (mShowFullSpectrum)
  {
    extractLevels(mpDisplayBuffer_lf, mFmRate);
  }
  else
  {
    extractLevelsHalfSpectrum(mpDisplayBuffer_lf, mFmRate);
  }
#endif

  mpLfBuffer->putDataIntoBuffer(mpDisplayBuffer_lf, mDisplaySize);

  //	and signal the GUI thread that we have data
  emit lfBufferLoaded(mShowFullSpectrum, mSpectrumSampleRate / zoomFactor);
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
  const double alpha = 1.0 / mAverageCount;
  const double beta = (mAverageCount - 1.0) / mAverageCount;

  for (int32_t i = 0; i < mDisplaySize; i++)
  {
    buffer[i] = alpha * in[i] + beta * buffer[i];
  }
}

#ifdef USE_EXTRACT_LEVELS
void fmProcessor::extractLevels(const double * const in, const int32_t range)
{
  const float binWidth = (float)range / mZoomFactor / mDisplaySize;
  const int32_t pilotOffset = mDisplaySize / 2 - 19000 / binWidth;
  const int32_t rdsOffset   = mDisplaySize / 2 - 57000 / binWidth;
  const int32_t noiseOffset = mDisplaySize / 2 - 70000 / binWidth;

//  int   a = mMyRig->bitDepth() - 1;
//  int   b = 1;

//  while (--a > 0)
//  {
//    b <<= 1;
//  }

  float noiseAvg = 0, pilotAvg = 0, rdsAvg = 0;

  for (int32_t i = 0; i < 7; i++)
  {
    noiseAvg += in[noiseOffset - 3 + i];
    rdsAvg += in[rdsOffset - 3 + i];
  }

  for (int32_t i = 0; i < 3; i++)
  {
    pilotAvg += in[pilotOffset - 1 + i];
  }

  mNoiseLevel = 0.95 * mNoiseLevel + 0.05 * noiseAvg / 7;
  mPilotLevel = 0.95 * mPilotLevel + 0.05 * pilotAvg / 3;
  mRdsLevel   = 0.95 * mRdsLevel   + 0.05 * rdsAvg / 7;
}

void fmProcessor::extractLevelsHalfSpectrum(const double * const in, const int32_t range)
{
  const float binWidth = (float)range / mZoomFactor / mDisplaySize / 2;
  const int32_t pilotOffset = 19000 / binWidth;
  const int32_t rdsOffset   = 57000 / binWidth;
  const int32_t noiseOffset = 70000 / binWidth;

  constexpr int32_t avgNoiseRdsSize = 1 + 2 * 6; // mid plus two times sidebands
  constexpr int32_t avgPilotSize    = 1 + 2 * 2;

  float noiseAvg = 0, pilotAvg = 0, rdsAvg = 0;

  for (int32_t i = 0; i < avgNoiseRdsSize; i++)
  {
    noiseAvg += in[noiseOffset - 3 + i];
    rdsAvg += in[rdsOffset - 3 + i];
  }

  for (int32_t i = 0; i < avgPilotSize; i++)
  {
    pilotAvg += in[pilotOffset - 1 + i];
  }

  constexpr float ALPHA = 0.2f;
  mNoiseLevel = (1.0f - ALPHA) * mNoiseLevel + ALPHA * noiseAvg / avgNoiseRdsSize;
  mPilotLevel = (1.0f - ALPHA) * mPilotLevel + ALPHA * pilotAvg / avgPilotSize;
  mRdsLevel   = (1.0f - ALPHA) * mRdsLevel   + ALPHA * rdsAvg / avgNoiseRdsSize;
}
#endif
