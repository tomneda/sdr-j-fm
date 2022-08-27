/*
 *    Copyright (C)  2014
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair Programming
 *
 *    This file is part of the SDR-J-FM program.
 *    Many of the ideas as implemented in SDR-J-FM are derived from
 *    other work, made available through the GNU general Public License.
 *    All copyrights of the original authors are recognized.
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
 */

#include  "fm-demodulator.h"
#include  "Xtan2.h"
#include  <assert.h>

fm_Demodulator::TDecoderListNames fm_Demodulator::sIdx2DecoderName =
{
 "AM (experimental)",
 "PLL Decoder",
 "Mixed Demod",
 "Complex Baseband Delay",
 "Real Baseband Delay",
 "Difference Based"
};

//	Just to play around a little, I implemented 5 common
//	fm decoders. The main source of inspiration is found in
//	a Diploma Thesis "Implementation of FM demodulator Algorithms
//	on a High Performance Digital Signal Processor", especially
//	chapter 3.
fm_Demodulator::fm_Demodulator (int32_t rateIn,
                                SinCos  *mySinCos,
                                DSPFLOAT K_FM)
//  mAmHighpass(20, 100, rateIn, S_CHEBYSHEV),
//  mAmLowpass(1, 1, rateIn, S_CHEBYSHEV),
//  mAmBandpass(20, 100, 4500, rateIn, S_CHEBYSHEV)
{
  int32_t i;

  this->rateIn   = rateIn;
  this->mySinCos = mySinCos;
  this->K_FM     = 2 * K_FM;

  this->selectedDecoder = 3;
  this->max_freq_deviation = 0.95 * (0.5 * rateIn);
  myfm_pll = new pllC(rateIn, 0, -max_freq_deviation, +max_freq_deviation, 0.85 * rateIn, mySinCos);

  Arcsine = new DSPFLOAT[ARCSINESIZE + 1]; // use also the top limit with +1 (happens with heavy noise)

  for (i = 0; i <= ARCSINESIZE; i++)
  {
    Arcsine[i] = asin(2.0 * i / ARCSINESIZE - 1.0); // maps +/-1 [0..8192] -> +/-PI/2
  }

  Imin1 = 0.2;
  Qmin1  = 0.2;
  Imin2  = 0.2;
  Qmin2  = 0.2;
  am_carr_ampl = 0;
  fm_afc = 0;
  fm_cvt = 1.0;
  // fm_cvt = 0.50 * (rateIn / (M_PI * 150000));
}

fm_Demodulator::~fm_Demodulator()
{
  delete  Arcsine;
  delete  myfm_pll;
}

void fm_Demodulator::setDecoder(int16_t nc)
{
  this->selectedDecoder = nc;
}

fm_Demodulator::TDecoderListNames & fm_Demodulator::listNameofDecoder() const
{
  return sIdx2DecoderName;
}

const char * fm_Demodulator::nameofDecoder() const
{
  return sIdx2DecoderName.at(selectedDecoder);
}

DSPFLOAT fm_Demodulator::demodulate(DSPCOMPLEX z)
{
  constexpr DSPFLOAT DCAlpha = 0.0001;

  DSPFLOAT res;
  DSPFLOAT I, Q;

  if (abs(z) <= 0.001)
  {
    I = Q = 0.001;  // do not make these 0 too often
  }
  else
  {
    I = real(z) / abs(z);
    Q = imag(z) / abs(z);
  }

  if (selectedDecoder == 0) // AM
  {
    // get carrier offset to have AFC for AM, too
    myfm_pll->do_pll(DSPCOMPLEX(I, Q));
    res = myfm_pll->getPhaseIncr();
    fm_afc = (1 - DCAlpha) * fm_afc + DCAlpha * res;

    const DSPFLOAT zAbs = abs(z);

    // get DC component or mean carrier power
    constexpr float alpha = 0.001f;
    am_carr_ampl = (1.0f - alpha) * am_carr_ampl + alpha * zAbs;

    // remove DC component from signal and norm level to carrier power
    constexpr float gainLimit = 0.05f;
    res = (zAbs - am_carr_ampl) / (am_carr_ampl < gainLimit ? gainLimit : am_carr_ampl);

    // this avoids short spikes which would cause the auto level limitter to reduce audio level too much
    constexpr float audioLimit = 1.0f;
    if      (res >  audioLimit) res =  audioLimit;
    else if (res < -audioLimit) res = -audioLimit;

    return res;
  }

  z = DSPCOMPLEX(I, Q);
  int32_t arcSineIdx;

  // see https://docplayer.net/32191874-Dsp-implementation-of-fm-demodulator-algorithms-on-a-high-performance-digital-signal-processor-diploma-thesis-11-1.html

  switch (selectedDecoder)
  {
  default:
    [[fallthrough]];

  case 1: // PllDecoder
    myfm_pll->do_pll(z);
    res = myfm_pll->getPhaseIncr();
    break;

  case 2: // MixedDemodulator
    res = -myAtan.atan2(Q * Imin1 - I * Qmin1, I * Imin1 + Q * Qmin1); // is same as ComplexBasebandDelay (expanded complex multiplication)
    break;

  case 3: // ComplexBasebandDelay
    res = -myAtan.argX(z * DSPCOMPLEX(Imin1, -Qmin1)); // is same as MixedDemodulator
    break;

  case 4: // RealBasebandDelay
    res = Imin1 * Q - Qmin1 * I; // theoretical interval [-2 .. +2], practical interval [-1 .. +1]
    arcSineIdx = (int32_t)(ARCSINESIZE * (res + 1.0f) / 2.0f + 0.5f);
    assert(arcSineIdx >= 0 && arcSineIdx <= ARCSINESIZE);
    res = -Arcsine[arcSineIdx]; // Arcsine center is at ArcsineSize/2
    break;

  case 5: // DifferenceBased
    res    = -(Imin1 * (Q - Qmin2) - Qmin1 * (I - Imin2));
    res   /= (Imin1 * Imin1 + Qmin1 * Qmin1) * M_SQRT2;
    Imin2  = Imin1;
    Qmin2  = Qmin1;
    break;
  }

  fm_afc = (1 - DCAlpha) * fm_afc + DCAlpha * res;
  res = 20.0f * (res - fm_afc) * fm_cvt / K_FM;

  // and shift ...
  Imin1 = I;
  Qmin1 = Q;

  return res;
}

DSPFLOAT fm_Demodulator::get_DcComponent()
{
  return fm_afc;
}

