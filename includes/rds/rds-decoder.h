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
  void doDecode(const DSPCOMPLEX, DSPCOMPLEX * const);
  void reset(void);

private:
  void processBit(bool);
  void doDecode1(const DSPFLOAT, DSPFLOAT * const);
  void doDecode2(const DSPFLOAT, DSPFLOAT * const);
  void doDecode3(const DSPFLOAT, DSPFLOAT * const);

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

  void synchronizeOnBitClk(DSPFLOAT *, int16_t);

signals:
  void setCRCErrors(int);
  void setSyncErrors(int);
  void setbitErrorRate(int);
};

#endif
