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
//#include "iir-filters.h"
#include "sdr/shaping_filter.h"

#include <vector>
#include <fstream>

constexpr uint32_t SMP_PER_MANC_SYM = 16;  // a manchaster sympol has 2 times the samples of a single RDS symbol
constexpr uint32_t TAPS_MF_RRC = 151;
constexpr uint32_t TAPS_MF_RRC_MANC = TAPS_MF_RRC - SMP_PER_MANC_SYM / 2;
constexpr DSPFLOAT RDS_BITCLK_HZ = 1187.5;

/*
 *	RDS is a bpsk-like signal, with a baudrate 1187.5
 *	on a carrier of  3 * 19 k.
 *	48 cycles per bit, 1187.5 bits per second.
 *	With a reduced sample rate of 19k this would mean
 *	19000 / 1187.5 samples per bit, i.e. 16
 *	samples per bit.
 *	Notice that complex mixing to zero IF has been done
 */
rdsDecoder::rdsDecoder(RadioInterface * iRadioIf, int32_t iRate)
  : mAGC(2e-3f, 0.4f, 10.0f)
  , mTimeSync(ceil((float)iRate / (float)RDS_BITCLK_HZ) /*== 16.0*/, 0.01f)
  , mCostas(iRate, 1.0f, 0.02f, 10.0f)
{
  mpRadioInterface = iRadioIf;
  mSampleRate = iRate;

  ShapingFilter sf;
  const std::vector<float> rrcImpVec = sf.root_raised_cosine(1.0, mSampleRate, 2 * RDS_BITCLK_HZ, 1.0, TAPS_MF_RRC);

  mMatchedFltKernelVec.resize(TAPS_MF_RRC_MANC);

  for (size_t idx = 0; idx < TAPS_MF_RRC_MANC; ++idx)
  {
    mMatchedFltKernelVec[idx] = rrcImpVec[idx]; // - rrcImpVec[idx + SMP_PER_MANC_SYM/2]; // the negative pulse comes 8 sample before the positive pulse
  }

  mMatchedFltBufSize = mMatchedFltKernelVec.size();
  assert(mMatchedFltBufSize & 1); // is it odd?
  mpMatchedFltBuf = new DSPCOMPLEX[mMatchedFltBufSize];
  memset(mpMatchedFltBuf, 0, mMatchedFltBufSize * sizeof(DSPCOMPLEX));
  mMatchedFltBufIdx = 0;

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
  delete mpRdsGroupDecoder;
  delete mpRdsGroup;
  delete mpRdsBlockSync;
  delete mpMatchedFltBuf;
}

void rdsDecoder::reset(void) { mpRdsGroupDecoder->reset(); }

DSPCOMPLEX rdsDecoder::doMatchFiltering(DSPCOMPLEX v)
{
  DSPCOMPLEX tmp = 0;

  mpMatchedFltBuf[mMatchedFltBufIdx] = v;

  for (int16_t i = 0; i < mMatchedFltBufSize; i++)
  {
    int16_t index = (mMatchedFltBufIdx - i); // read data backwards from current position

    if (index < 0)
    {
      index += mMatchedFltBufSize; // wrap around index
    }

    tmp += mpMatchedFltBuf[index] * mMatchedFltKernelVec[i];
  }

  mMatchedFltBufIdx = (mMatchedFltBufIdx + 1) % mMatchedFltBufSize;

  return tmp;
}

bool rdsDecoder::doDecode(DSPCOMPLEX v, DSPCOMPLEX * const m)
{
  // this is called typ. 19000 1/s
  v = doMatchFiltering(v);
  v = mAGC.process_sample(v);

  DSPCOMPLEX r;

  if (mTimeSync.process_sample(v, r))
  {
    // this runs 19000/16 = 1187.5 1/s times
    r = mCostas.process_sample(r);

    const bool bit = (real(r) >= 0);
    processBit(bit ^ mPreviousBit);
    mPreviousBit = bit;

    *m = r;

    return true; // tell caller a changed m value
  }

  return false;
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
