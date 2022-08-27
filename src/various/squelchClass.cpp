#include "squelchClass.h"
#include "radio.h"

constexpr DSPFLOAT SQUELCH_HYSTERESIS = 0.001;
constexpr DSPFLOAT LEVELREDUCTIONFACTOR = 0.05; // 23dB lower output level

squelch::squelch(const int32_t iSquelchThreshold, const int32_t iKeyFrequency, const int32_t iBufsize, const int32_t iSampleRate) :
  mSquelchHighpass(20, iKeyFrequency - 100, iSampleRate, S_CHEBYSHEV),
  mSquelchLowpass(20, iKeyFrequency, iSampleRate, S_CHEBYSHEV)
{
  setSquelchLevel(iSquelchThreshold); // convert 0..100 to 1.0..0.0
  mKeyFrequency = iKeyFrequency;
  mHoldPeriod = iBufsize;
  mSampleRate = iSampleRate;

  mSquelchSuppress = false;
  mSquelchSuppressLast = !mSquelchSuppress;
  mSquelchCount = 0;
  mAverage_High = 0;
  mAverage_Low = 0;
}

static inline DSPFLOAT decayingAverage(DSPFLOAT old, DSPFLOAT input, DSPFLOAT weight)
{
  if (weight <= 1)
  {
    return input;
  }
  return input * (1.0 / weight) + old * (1.0 - (1.0 / weight));
}

DSPFLOAT squelch::do_squelch(const DSPFLOAT soundSample)
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

  if (mSquelchSuppress != mSquelchSuppressLast)
  {
    mSquelchSuppressLast = mSquelchSuppress;
    emit setSquelchIsActive(mSquelchSuppress);
  }

  return mSquelchSuppress ? soundSample * LEVELREDUCTIONFACTOR : soundSample;
}
