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

fm_Demodulator::TDecoderListNames fm_Demodulator::sIdx2DecoderName =
{
 "Difference Based",
 "Compl. Baseb. Delay",
 "Mixed Demod",
 "PLL Decoder",
 "Real Baseb. Delay"
};



//
//	Just to play around a little, I implemented 5 common
//	fm decoders. The main source of inspiration is found in
//	a Diploma Thesis "Implementation of FM demodulator Algorithms
//	on a High Performance Digital Signal Processor", especially
//	chapter 3.
fm_Demodulator::fm_Demodulator (int32_t rateIn,
                                SinCos  *mySinCos,
                                DSPFLOAT K_FM)
{
  int32_t i;

  this->rateIn   = rateIn;
  this->mySinCos = mySinCos;
  this->K_FM     = 2 * K_FM;

  this->selectedDecoder = 3;
  this->max_freq_deviation = 0.95 * (0.5 * rateIn);
  myfm_pll = new pllC(rateIn,
                      0,
                      -max_freq_deviation,
                      +max_freq_deviation,
                      0.85 * rateIn,
                      mySinCos);
  ArcsineSize = 4 * 8192;
  Arcsine     = new DSPFLOAT [ArcsineSize];

  for (i = 0; i < ArcsineSize; i++)
  {
    Arcsine [i] = asin(2.0 * i / ArcsineSize - 1.0) / 2.0;
  }

  Imin1  = 0.2;
  Qmin1  = 0.2;
  Imin2  = 0.2;
  Qmin2  = 0.2;
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
  DSPFLOAT res;
  DSPFLOAT I, Q;

#define DCAlpha    0.0001
//#define	DCAlpha	0.000001

  if (abs(z) <= 0.001)
  {
    I = Q = 0.001;  // do not make these 0 too often
  }
  else
  {
    I = real(z) / abs(z);
    Q = imag(z) / abs(z);
  }

  z = DSPCOMPLEX(I, Q);
  switch (selectedDecoder)
  {
  default:
    [[fallthrough]];

  case 0: // DifferenceBased
    res    = -(Imin1 * (Q - Qmin2) - Qmin1 * (I - Imin2));
    res   /= Imin1 * Imin1 + Qmin1 * Qmin1;
    Imin2  = Imin1;
    Qmin2  = Qmin1;
    break;

  case 1: // ComplexBasebandDelay
    res = -myAtan.argX(z * DSPCOMPLEX(Imin1, -Qmin1));
    break;

  case 2: // MixedDemodulator
    res = -myAtan.atan2(Q * Imin1 - I * Qmin1, I * Imin1 + Q * Qmin1);
    break;

  case 3: // PllDecoder
    myfm_pll->do_pll(z);
    res = myfm_pll->getPhaseIncr();
    break;

  case 4: // RealBasebandDelay
    res = (Imin1 * Q - Qmin1 * I + 1.0) / 2.0;
    res = -Arcsine [(int)(res * ArcsineSize)];
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

