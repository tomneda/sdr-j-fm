/*
 *    Copyright (C) 2008, 2009, 2010
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair programming
 *
 *    This file is part of the SDR-J (JSDR).
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

#ifndef __SQUELCHCLASS
#define __SQUELCHCLASS

#include "iir-filters.h"
//
//	just a simple class to include elementary squelch handling
//	The basic idea is that when there is no signal, the noise
//	in the upper bands will be roughly as high as in the lowerbands.
//	Measurement shows that the average amplitude of the noise in the
//	upper band is roughly 0.6 times that of the lower part.
//	If the average signal value of the upper part is larger
//	than factor times the average signal value of the lower part,
//	where factor is a value between 0 .. 1, set by the user.
constexpr DSPFLOAT SQUELCH_HYSTERESIS = 0.001;
constexpr DSPFLOAT LEVELREDUCTIONFACTOR = 0.05; // 23dB lower output level

class squelch
{
private:
  DSPFLOAT mSquelchThreshold; // value between 0 and 1
  int32_t mKeyFrequency;
  int32_t mHoldPeriod;
  int32_t mSampleRate;
  bool mSquelchSuppress;
  int32_t mSquelchCount;
  DSPFLOAT mAverage_High;
  DSPFLOAT mAverage_Low;
  HighPassIIR mSquelchHighpass;
  LowPassIIR mSquelchLowpass;

public:
  squelch(const int32_t iSquelchThreshold, const int32_t iKeyFrequency, const int32_t iBufsize, const int32_t iSampleRate) :
    mSquelchHighpass(20, iKeyFrequency - 100, iSampleRate, S_CHEBYSHEV),
    mSquelchLowpass(20, iKeyFrequency, iSampleRate, S_CHEBYSHEV)
  {
    setSquelchLevel(iSquelchThreshold); // convert 0..100 to 1.0..0.0
    mKeyFrequency = iKeyFrequency;
    mHoldPeriod = iBufsize;
    mSampleRate = iSampleRate;

    mSquelchSuppress = false;
    mSquelchCount = 0;
    mAverage_High = 0;
    mAverage_Low = 0;
  }

  ~squelch(void) {}

  void setSquelchLevel(int n) { mSquelchThreshold = 1.0f - n / 100.0f; } // convert 0..100 to 1.0..0.0

  static inline DSPFLOAT decayingAverage(DSPFLOAT old, DSPFLOAT input, DSPFLOAT weight)
  {
    if (weight <= 1)
    {
      return input;
    }
    return input * (1.0 / weight) + old * (1.0 - (1.0 / weight));
  }

  DSPFLOAT do_squelch(const DSPFLOAT soundSample)
  {
    const DSPFLOAT val_1 = abs(mSquelchHighpass.Pass(soundSample));
    const DSPFLOAT val_2 = abs(mSquelchLowpass.Pass(soundSample));

    mAverage_High = decayingAverage(mAverage_High, val_1, mSampleRate / 100);
    mAverage_Low = decayingAverage(mAverage_Low, val_2, mSampleRate / 100);

    if (++mSquelchCount >= mHoldPeriod)
    {
      mSquelchCount = 0;

      //	looking for a new squelch state
      if (mSquelchThreshold < SQUELCH_HYSTERESIS)   // force squelch if zero
      {
        mSquelchSuppress = true;
      }
      else if (mAverage_High < mAverage_Low * mSquelchThreshold - SQUELCH_HYSTERESIS)
      {
        mSquelchSuppress = false;
      }
      else if (mAverage_High >= mAverage_Low * mSquelchThreshold + SQUELCH_HYSTERESIS)
      {
        mSquelchSuppress = true;
      }
      //	else just keep old squelchSuppress value
    }

    return mSquelchSuppress ? soundSample * LEVELREDUCTIONFACTOR : soundSample;
  }
};

#endif
