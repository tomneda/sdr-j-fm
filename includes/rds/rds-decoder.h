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

/* massively adapted by tomneda https://github.com/tomneda */

#ifndef __RDS_DECODER
#define __RDS_DECODER

#include <QObject>
#include "fm-constants.h"
#include "rds-group.h"
#include "rds-blocksynchronizer.h"
#include "rds-groupdecoder.h"
#include "sdr/agc.h"
#include "sdr/costas.h"
#include "sdr/time_sync.h"


class RadioInterface;

class rdsDecoder : public QObject
{
  Q_OBJECT
public:
  rdsDecoder(RadioInterface *, int32_t);
  ~rdsDecoder(void);

  enum class ERdsMode
  {
    RDS_OFF,
    RDS_ON,
  };

  bool doDecode(const DSPCOMPLEX, DSPCOMPLEX * const);
  void reset(void);

private:
  void processBit(bool);

  AGC mAGC;
  TimeSync mTimeSync;
  Costas mCostas;

  int32_t mSampleRate;
  RadioInterface * mpRadioInterface;

  RDSGroup * mpRdsGroup;
  rdsBlockSynchronizer * mpRdsBlockSync;
  rdsGroupDecoder * mpRdsGroupDecoder;

  std::vector<DSPFLOAT> mMatchedFltKernelVec;
  DSPCOMPLEX * mpMatchedFltBuf;
  int16_t mMatchedFltBufIdx;
  int16_t mMatchedFltBufSize;

  bool mPreviousBit;

  DSPFLOAT doMatchFiltering(DSPFLOAT);
  DSPCOMPLEX doMatchFiltering(DSPCOMPLEX);

signals:
  void setCRCErrors(int);
  void setSyncErrors(int);
  void setbitErrorRate(int);
};

#endif
