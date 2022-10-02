/*
 *    Copyright (C) 2008, 2009, 2010
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair Programming
 *
 *    This file is part of the SDR-J
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
#ifndef __FFT_FILTER
#define __FFT_FILTER

#include "fft.h"
#include "fm-constants.h"

class fftFilter {
public:
  fftFilter(int32_t, int16_t);
  ~fftFilter(void);

  void setBand(int32_t, int32_t, int32_t);
  void setSimple(int32_t, int32_t, int32_t);
  void setLowPass(int32_t, int32_t);
  DSPCOMPLEX Pass(DSPCOMPLEX);
  DSPFLOAT Pass(DSPFLOAT);

private:
  int32_t mFftSize;
  int16_t mFilterDegree;
  int16_t mOverlapSize;
  int16_t mNumofSamples;
  common_fft *mpMyFFT;
  DSPCOMPLEX *mpFFT_A;
  common_ifft *mpMyIFFT;
  DSPCOMPLEX *mpFFT_C;
  common_fft *mpFilterFFT;
  DSPCOMPLEX *mpFilterVector;
  DSPFLOAT *mpRfilterVector;
  DSPCOMPLEX *mpOverloop;
  int32_t mInpIdx;
};

#endif
