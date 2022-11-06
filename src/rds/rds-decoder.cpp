/*
 *    Copyright (C)  2014
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair Computing
 *
 *    This file is part of the sdr-j-fm
 *
 *    sdr-j-fm is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    sdr-j-fm is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with sdr-j-fm; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdlib.h>
#include <stdio.h>
#include "rds-decoder.h"
#include "radio.h"
#include "iir-filters.h"
#include "sincos.h"

#include <vector>
#include <fstream>


static std::vector<float> root_raised_cosine(double gain, double sampling_freq, double symbol_rate, double alpha, int ntaps)
{
  ntaps |= 1; // ensure that ntaps is odd

  double spb = sampling_freq / symbol_rate;// samples per bit/symbol
  std::vector<float> taps(ntaps);
  double scale = 0;

  for (int i = 0; i < ntaps; i++)
  {
    double x1, x2, x3, num, den;
    double xindx = i - ntaps / 2;
    x1 = M_PI * xindx / spb;
    x2 = 4 * alpha * xindx / spb;
    x3 = x2 * x2 - 1;

    if (fabs(x3) >= 0.000001) // Avoid Rounding errors...
    {
      if (i != ntaps / 2)
      {
        num = cos((1 + alpha) * x1) + sin((1 - alpha) * x1) / (4 * alpha * xindx / spb);
      }
      else
      {
        num = cos((1 + alpha) * x1) + (1 - alpha) * M_PI / (4 * alpha);
      }
      den = x3 * M_PI;
    }
    else
    {
      if (alpha == 1)
      {
        taps[i] = -1;
        scale += taps[i]; // this was missing in gnuradio, seems to be a bug there
        continue;
      }
      x3  = (1 - alpha) * x1;
      x2  = (1 + alpha) * x1;
      num = (sin(x2) * (1 + alpha) * M_PI
             - cos(x3) * ((1 - alpha) * M_PI * spb) / (4 * alpha * xindx)
             + sin(x3) * spb * spb / (4 * alpha * xindx * xindx));
      den = -32 * M_PI * alpha * alpha * xindx / spb;
    }
    taps[i] = 4 * alpha * num / den;
    scale  += taps[i];
  }

  for (int i = 0; i < ntaps; i++)
  {
    taps[i] = taps[i] * gain / scale;
  }

  return taps;
}



AGC::AGC(float rate, float reference, float gain, float max_gain)
  : _rate(rate)
  , _reference(reference)
  , _gain(gain)
  , _max_gain(max_gain)
{
}

DSPCOMPLEX AGC::scale(DSPCOMPLEX input)
{
  DSPCOMPLEX output = input * _gain;

  _gain += _rate * (_reference - sqrt(output.real() * output.real() +
                                      output.imag() * output.imag()));
  if (_max_gain > 0.0 && _gain > _max_gain)
  {
    _gain = _max_gain;
  }

  return output;
}

void AGC::scaleN(DSPCOMPLEX output[], const DSPCOMPLEX input[], uint32_t n)
{
  for (uint32_t i = 0; i < n; i++)
  {
    output[i] = scale(input[i]);
  }
}




//agc_cc::sptr
// agc_cc::make(float rate, float reference, float gain)
// {
//   return gnuradio::get_initial_sptr
//(new agc_cc_impl(rate, reference, gain));
// }

// agc_cc_impl::agc_cc_impl(float rate, float reference, float gain)
//   : sync_block("agc_cc",
//                io_signature::make(1, 1, sizeof(gr_complex)),
//                io_signature::make(1, 1, sizeof(gr_complex))),
//kernel::agc_cc(rate, reference, gain, 65536)
// {
//   const int alignment_multiple =
//volk_get_alignment() / sizeof(gr_complex);
//   set_alignment(std::max(1, alignment_multiple));
// }

// agc_cc_impl::~agc_cc_impl()
// {
// }

// int
// agc_cc_impl::work(int noutput_items,
//       gr_vector_const_void_star &input_items,
//       gr_vector_void_star &output_items)
// {
//   const gr_complex *in = (const gr_complex*)input_items[0];
//   gr_complex *out = (gr_complex*)output_items[0];
//   scaleN(out, in, noutput_items);
//   return noutput_items;
// }
















constexpr uint32_t SMP_PER_MANC_SYM = 16;  // a manchaster sympol has 2 times the samples of a single RDS symbol
constexpr uint32_t TAPS_MF_RRC = 151;
constexpr uint32_t TAPS_MF_RRC_MANC = TAPS_MF_RRC - SMP_PER_MANC_SYM / 2;
constexpr DSPFLOAT RDS_BITCLK_HZ = 1187.5;

/*
 *	RDS is a bpsk-like signal, with a baudrate 1187.5
 *	on a carrier of  3 * 19 k.
 *	48 cycles per bit, 1187.5 bits per second.
 *	With a reduced sample rate of 48K this would mean
 *	48000 / 1187.5 samples per bit, i.e. between 40 and 41
 *	samples per bit.
 *	Notice that mixing to zero IF has been done
 */
rdsDecoder::rdsDecoder(RadioInterface * iRadioIf, int32_t iRate, SinCos * ipSinCos)
  : mAGC(2e-3,0.485 /*0.585*/, 10, 1000)
{
  (void)ipSinCos;

  mpRadioInterface = iRadioIf;
  mSampleRate = iRate;
  mpSinCos = new SinCos(iRate);
  mOmegaRDS = (2 * M_PI * RDS_BITCLK_HZ) / (DSPFLOAT)iRate;
  //
  //	for the decoder a la FMStack we need:
  const DSPFLOAT synchronizerSamples = mSampleRate / (DSPFLOAT)RDS_BITCLK_HZ;
  mSymbolCeiling = ceil(synchronizerSamples);
  mSymbolFloor = floor(synchronizerSamples);
  mSyncBuffer = new DSPFLOAT[mSymbolCeiling];
  mSyncBufferCplx = new DSPCOMPLEX[mSymbolCeiling];
  memset(mSyncBuffer, 0, mSymbolCeiling * sizeof(DSPFLOAT));
  memset(mSyncBufferCplx, 0, mSymbolCeiling * sizeof(DSPCOMPLEX));
  mSyncBuffPtrIdx = 0;
  mSyncBuffPtrIdxCplx = 0;
  mBitIntegrator = 0;
  mBitClkPhase = 0;
  mPrev_clkState = 0;
  mPrevBit = false;
  mResync = true;


  const std::vector<float> rrcImpVec = root_raised_cosine(1.0, mSampleRate, 2 * RDS_BITCLK_HZ, 1.0, TAPS_MF_RRC);

  mRrcImpMancVec.resize(TAPS_MF_RRC_MANC);

  for (size_t idx = 0; idx < TAPS_MF_RRC_MANC; ++idx)
  {
    mRrcImpMancVec[idx] = rrcImpVec[idx] - rrcImpVec[idx + SMP_PER_MANC_SYM/2]; // the negative pulse comes 8 sample before the positive pulse
  }

//  {
//    std::ofstream of("/home/work/rrcMancVec100.txt", std::ios_base::out);
//    assert(of.is_open());
//    for (auto vec : rrcImpMancVec)
//    {
//      of << vec << " ";
//    }
//    of.close();
//  }

  //
  //	The matched filter is a borrowed from the cuteRDS, who in turn
  //	borrowed it from course material
  //      http://courses.engr.illinois.edu/ece463/Projects/RBDS/RBDS_project.doc
  //	Note that the formula down has a discontinuity for
  //	two values of x, we better make the symbollength odd

  //length = (mSymbolCeiling & ~01) + 1;
  //length = mRrcImpMancVec.size();
  //assert(length & 1); // is it odd?
  //mRdsfilterSize = 2 * length + 1;
  mRdsfilterSize = mRrcImpMancVec.size();
  assert(mRdsfilterSize & 1); // is it odd?
  mpRdsBuffer = new DSPFLOAT[mRdsfilterSize];
  mpRdsBufferCplx = new DSPCOMPLEX[mRdsfilterSize];
  memset(mpRdsBuffer, 0, mRdsfilterSize * sizeof(DSPFLOAT));
  memset(mpRdsBufferCplx, 0, mRdsfilterSize * sizeof(DSPCOMPLEX));
  mCurRdsBufferIdx = 0;

  mpRdsKernel = &mRrcImpMancVec[0];

//  mpRdsKernel = new DSPFLOAT[mRdsfilterSize];
//  mpRdsKernel[length] = 0;
//  for (i = 1; i <= length; i++)
//  {
//    DSPFLOAT x = ((DSPFLOAT)i) / iRate * RDS_BITCLK_HZ;
//    mpRdsKernel[length + i] =  0.75 * cos(4 * M_PI * x) * ((1.00 / (1.0 / x - 64 * x)) - ((1.00 / (9.0 / x - 64 * x))));
//    mpRdsKernel[length - i] = -mpRdsKernel[length + i];
//  }

  //	The matched filter is followed by a pretty sharp filter
  //	to eliminate all remaining "noise".
  mpSharpFilter = new BandPassIIR(9, RDS_BITCLK_HZ - 6, RDS_BITCLK_HZ + 6, iRate, S_BUTTERWORTH);
  mRdsLastSyncSlope = 0;
  mRdsLastSync = 0;
  mRdsLastData = 0;
  mPreviousBit = false;

  mpRdsGroup = new RDSGroup();
  mpRdsGroup->clear();
  mpRdsBlockSync = new rdsBlockSynchronizer(mpRadioInterface);
  mpRdsBlockSync->setFecEnabled(true);
  mpRdsGroupDecoder = new rdsGroupDecoder(mpRadioInterface);

  connect(this, SIGNAL(setCRCErrors(int)), mpRadioInterface, SLOT(setCRCErrors(int)));
  connect(this, SIGNAL(setSyncErrors(int)), mpRadioInterface, SLOT(setSyncErrors(int)));
}

rdsDecoder::~rdsDecoder(void)
{
  delete[] mSyncBuffer;
  delete mpRdsGroupDecoder;
  delete mpRdsGroup;
  delete mpRdsBlockSync;
  delete mpRdsKernel;
  delete mpRdsBuffer;
  delete mpRdsBufferCplx;
  delete mpSharpFilter;
}

void rdsDecoder::reset(void) { mpRdsGroupDecoder->reset(); }

DSPFLOAT rdsDecoder::doMatchFiltering(DSPFLOAT v)
{
  DSPFLOAT tmp = 0;

  mpRdsBuffer[mCurRdsBufferIdx] = v;

  for (int16_t i = 0; i < mRdsfilterSize; i++)
  {
    int16_t index = (mCurRdsBufferIdx - i); // read data backwards from current position

    if (index < 0)
    {
      index += mRdsfilterSize; // wrap around index
    }

    tmp += mpRdsBuffer[index] * mpRdsKernel[i];
  }

  mCurRdsBufferIdx = (mCurRdsBufferIdx + 1) % mRdsfilterSize;

  return tmp;
}

DSPCOMPLEX rdsDecoder::doMatchFiltering(DSPCOMPLEX v)
{
  DSPCOMPLEX tmp = 0;

  mpRdsBufferCplx[mCurRdsBufferIdx] = v;

  for (int16_t i = 0; i < mRdsfilterSize; i++)
  {
    int16_t index = (mCurRdsBufferIdx - i); // read data backwards from current position

    if (index < 0)
    {
      index += mRdsfilterSize; // wrap around index
    }

    tmp += mpRdsBufferCplx[index] * mpRdsKernel[i];
  }

  mCurRdsBufferIdx = (mCurRdsBufferIdx + 1) % mRdsfilterSize;

  return tmp;
}
/*
 *	Signal (i.e. "v") is already downconverted and lowpass filtered
 *	when entering this stage. The return value stored in "*m" is used
 *	to display things to the user
 */
void rdsDecoder::doDecode(const DSPFLOAT v, DSPFLOAT * const m, const ERdsMode mode)
{
  if (mode == ERdsMode::NO_RDS)
  {
    return;   // should not happen
  }

  if (mode == ERdsMode::RDS1)
  {
    doDecode1(v, m);
  }
  else
  {
    doDecode2(v, m);
  }
}

void rdsDecoder::doDecode(const DSPCOMPLEX v, DSPCOMPLEX * const m)
{
  const DSPCOMPLEX vMF = doMatchFiltering(v);
  const DSPCOMPLEX vMF_scaled = mAGC.scale(vMF);

  // ---------------------------------------------------------------
  static DSPCOMPLEX out[3] {}; // current and last 2 samples
  static DSPCOMPLEX out_rail[3] {}; // current and last 2 samples
  static DSPFLOAT mu = 0.00; // 0.01;
  static DSPCOMPLEX r = 0;
  static int32_t next_used_smpl = 3;  // 2 to fill out buffer
  static int32_t smpl_cnt = 0;

  constexpr float alpha = 0.01;
  constexpr float sps = 16;

  //mSyncBufferCplx[mSyncBuffPtrIdx] = vMF_scaled;
  //mSyncBuffPtrIdx = (mSyncBuffPtrIdx + 1) % mSymbolCeiling; // points to beginn of last

  out[0] = out[1];
  out[1] = out[2];
  out[2] = vMF_scaled;

  if (++smpl_cnt >= next_used_smpl)
  {
    // get hard decision values (rail to rail)
    for (int32_t i = 0; i < 3; ++i)
    {
      out_rail[i] = DSPCOMPLEX((real(out[i]) > 0.0f ? 1.0f : -1.0f), (imag(out[i]) > 0.0f ? 1.0f : -1.0f));
    }

//    const DSPCOMPLEX x = (out_rail[2] - out_rail[0]) * conj(out[1]);
//    const DSPCOMPLEX y = (out[2] - out[0]) * conj(out_rail[1]);
//    const DSPFLOAT mm_val = real(y - x);

//    const DSPFLOAT x = real((out_rail[2] - out_rail[0]) * conj(out[1]));
//    const DSPFLOAT y = real((out[2] - out[0]) * conj(out_rail[1]));
//    const DSPFLOAT mm_val = y - x;

    const DSPFLOAT x = real(out_rail[2] - out_rail[0]) * real(out[1]) + imag(out_rail[2] - out_rail[0]) * imag(out[1]);
    const DSPFLOAT y = real(out[2] - out[0]) * real(out_rail[1]) + imag(out[2] - out[0]) * imag(out_rail[1]);
    const DSPFLOAT mm_val = y - x;

    mu += sps + alpha * mm_val;
    next_used_smpl = (int32_t)(round(mu)); // round down to nearest int since we are using it as an index
    mu -= next_used_smpl; // remove the integer part of mu

    r = out[2];
    smpl_cnt = 0;
  }

  *m = r;
  // ---------------------------------------------------------------

  //  static DSPFLOAT curArgCorr = 0;
//  static DSPFLOAT curArgErr = 0;

//  const DSPCOMPLEX vArg_corr = v * std::exp(DSPCOMPLEX(0, -curArgCorr));

////  const DSPCOMPLEX rdsMag = mpSharpFilter->Pass((vMF * vMF));
////  const DSPCOMPLEX rdsMag = vMF * conj(vMF);
//  const DSPCOMPLEX vMF_scaled_sq = vMF_scaled * vMF_scaled;
//  const float arg = 0.5f * mAtan.argX(vMF_scaled_sq);

//  //static DSPCOMPLEX vMF_scaled_sq_av = 0;
//  constexpr float alpha = 0.0001;
//  constexpr float beta = 0.0001;
//  const DSPFLOAT argError = arg - 0; // reference is 0 degree
//  curArgErr = argError * alpha + curArgErr * (1.0f - alpha);
//  curArgCorr += beta * curArgErr;

  //constexpr float alpha = 4.0f / RDS_BITCLK_HZ;
  //vMF_scaled_sq_av = vMF_scaled_sq * alpha + vMF_scaled_sq_av * (1.0f - alpha);
  //const float arg = 0.5f * mAtan.argX(vMF_scaled_sq_av);
  //const DSPCOMPLEX vMF_scaled_phase_corr = vMF_scaled * std::exp(DSPCOMPLEX(0, -arg));
  //const DSPCOMPLEX vMF_scaled_phase_corr = vMF_scaled * std::exp(DSPCOMPLEX(0, -mAtan.argX(vMF_scaled)));
  //*m = vMF_scaled_phase_corr;
  //*m = std::exp(DSPCOMPLEX(0, -arg));
  //*m = std::exp(DSPCOMPLEX(0, curArgCorr));
  //*m = vMF_scaled;

//  const DSPFLOAT vMF_scaled_phase_corr_real = real(vMF_scaled_phase_corr);
//  const DSPFLOAT rdsMag = mpSharpFilter->Pass(vMF_scaled_phase_corr_real * vMF_scaled_phase_corr_real);
//  //*m = (20 * rdsMag + 1.0);
//  const DSPFLOAT rdsSlope = rdsMag - mRdsLastSync;
//  mRdsLastSync = rdsMag;

//  static DSPCOMPLEX a = 0;

//  if ((rdsSlope < 0.0) && (mRdsLastSyncSlope >= 0.0))
//  {
//    a = vMF_scaled_phase_corr;
//    //	top of the sine wave: get the data
//    const bool bit = vMF_scaled_phase_corr_real >= 0;
//    //const bool bit = mRdsLastData >= 0;
//    processBit(bit ^ mPreviousBit);
//    //*m = (bit ^ mPreviousBit ? 0.5 : -0.5);
//    mPreviousBit = bit;
//  }
  //*m = a;
  //*m = vMF_scaled_phase_corr;

//  mRdsLastData = rdsMag;
//  mRdsLastSyncSlope = rdsSlope;
//  mpRdsBlockSync->resetResyncErrorCounter();
}

void rdsDecoder::doDecode1(const DSPFLOAT v, DSPFLOAT * const m)
{
  const DSPFLOAT vMF = doMatchFiltering(v);
  const DSPFLOAT rdsMag = mpSharpFilter->Pass(vMF * vMF);
  *m = 20*rdsMag;
  //*m = (20 * rdsMag + 1.0);
  const DSPFLOAT rdsSlope = rdsMag - mRdsLastSync;
  mRdsLastSync = rdsMag;

  if ((rdsSlope < 0.0) && (mRdsLastSyncSlope >= 0.0))
  {
    //	top of the sine wave: get the data
    const bool bit = mRdsLastData >= 0;
    processBit(bit ^ mPreviousBit);
    //*m = (bit ^ mPreviousBit ? 0.5 : -0.5);
    mPreviousBit = bit;
  }

  mRdsLastData = v;
  mRdsLastSyncSlope = rdsSlope;
  mpRdsBlockSync->resetResyncErrorCounter();
}

void rdsDecoder::doDecode2(const DSPFLOAT v, DSPFLOAT * const m)
{
  mSyncBuffer[mSyncBuffPtrIdx] = v;
  //*m = mSyncBuffer[mSyncBuffPtrIdx] + 1;
  mSyncBuffPtrIdx = (mSyncBuffPtrIdx + 1) % mSymbolCeiling;
  //v = mSyncBuffer[mSyncBuffPtrIdx];   // an old one

  if (mResync || (mpRdsBlockSync->getNumSyncErrors() > 3))
  {
    synchronizeOnBitClk(mSyncBuffer, mSyncBuffPtrIdx);
    mpRdsBlockSync->resync();
    mpRdsBlockSync->resetResyncErrorCounter();
    mResync = false;
  }

  const DSPFLOAT clkState = mpSinCos->getSin(mBitClkPhase);
  mBitIntegrator += v * clkState;
  *m = mBitIntegrator;

  //	rising edge -> look at integrator
  if (mPrev_clkState <= 0 && clkState > 0)
  {
    bool currentBit = mBitIntegrator >= 0;
    processBit(currentBit ^ mPreviousBit);
    mBitIntegrator = 0;   // we start all over
    mPreviousBit = currentBit;
  }

  mPrev_clkState = clkState;
  mBitClkPhase = fmod(mBitClkPhase + mOmegaRDS, 2 * M_PI);
}

void rdsDecoder::processBit(bool bit)
{
  switch (mpRdsBlockSync->pushBit(bit, mpRdsGroup))
  {
  case rdsBlockSynchronizer::RDS_WAITING_FOR_BLOCK_A: break;   // still waiting in block A

  case rdsBlockSynchronizer::RDS_BUFFERING: break;   // just buffer

  case rdsBlockSynchronizer::RDS_NO_SYNC:
    //	      resync if the last sync failed
    setSyncErrors(mpRdsBlockSync->getNumSyncErrors());
    mpRdsBlockSync->resync();
    break;

  case rdsBlockSynchronizer::RDS_NO_CRC:
    setCRCErrors(mpRdsBlockSync->getNumCRCErrors());
    mpRdsBlockSync->resync();
    break;

  case rdsBlockSynchronizer::RDS_COMPLETE_GROUP:
    if (!mpRdsGroupDecoder->decode(mpRdsGroup))
    {
      ;   // error decoding the rds group
    }

    //	      my_rdsGroup -> clear ();
    break;
  }
}

void rdsDecoder::synchronizeOnBitClk(DSPFLOAT * v, int16_t first)
{
  bool isHigh = false;
  int32_t k = 0;
  DSPFLOAT * const correlationVector = (DSPFLOAT *)alloca(mSymbolCeiling * sizeof(DSPFLOAT));

  memset(correlationVector, 0, mSymbolCeiling * sizeof(DSPFLOAT));

  //	synchronizerSamples	= sampleRate / (DSPFLOAT)RDS_BITCLK_HZ;
  for (int32_t i = 0; i < mSymbolCeiling; i++)
  {
    const DSPFLOAT phase = fmod(i * (mOmegaRDS / 2), 2 * M_PI);
    //	reset index on phase change
    if (mpSinCos->getSin(phase) > 0 && !isHigh)
    {
      isHigh = true;
      k = 0;
    }
    else if (mpSinCos->getSin(phase) < 0 && isHigh)
    {
      isHigh = false;
      k = 0;
    }

    correlationVector[k++] += v[(first + i) % mSymbolCeiling];
  }

  //	detect rising edge in correlation window
  int32_t iMin = 0;

  while (iMin < mSymbolFloor && correlationVector[iMin++] > 0)
  {
    ;
  }

  while (iMin < mSymbolFloor && correlationVector[iMin++] < 0)
  {
    ;
  }

  //	set the phase, previous sample (iMin - 1) is obviously the one
  mBitClkPhase = fmod(-mOmegaRDS * (iMin - 1), 2 * M_PI);

  while (mBitClkPhase < 0)
  {
    mBitClkPhase += 2 * M_PI;
  }
}
