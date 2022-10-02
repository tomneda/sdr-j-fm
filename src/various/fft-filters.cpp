/*
 *    Copyright (C) 2008, 2009, 2010
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

#include "fft-filters.h"
#include "fir-filters.h"
#include <cstring>

fftFilter::fftFilter(int32_t size, int16_t degree)
{
  int32_t i;

  mFftSize      = size;
  mFilterDegree = degree;
  mOverlapSize  = mFilterDegree;
  mNumofSamples = mFftSize - mOverlapSize;

  mpMyFFT  = new common_fft(mFftSize);
  mpFFT_A  = mpMyFFT->getVector();
  mpMyIFFT = new common_ifft(mFftSize);
  mpFFT_C  = mpMyIFFT->getVector();

  mpFilterFFT     = new common_fft(mFftSize);
  mpFilterVector  = mpFilterFFT->getVector();
  mpRfilterVector = new DSPFLOAT[mFftSize];

  mpOverloop = new DSPCOMPLEX[mOverlapSize];
  mInpIdx      = 0;

  for (i = 0; i < mFftSize; i++)
  {
    mpFFT_A[i]         = 0;
    mpFFT_C[i]         = 0;
    mpFilterVector[i]  = 0;
    mpRfilterVector[i] = 0;
  }
}

fftFilter::~fftFilter()
{
  delete mpMyFFT;
  delete mpMyIFFT;
  delete mpFilterFFT;
  delete[] mpRfilterVector;
  delete[] mpOverloop;
}

void fftFilter::setSimple(int32_t low, int32_t high, int32_t rate)
{
  BasicBandPass * const BandPass = new BasicBandPass((int16_t)mFilterDegree, low, high, rate);

  for (int32_t i = 0; i < mFilterDegree; i++)
  {
    mpFilterVector[i] = (BandPass->getKernel())[i];
  }

  for (int32_t i = mFilterDegree; i < mFftSize; i++)
  {
    mpFilterVector[i] = 0.0f;
  }

  //memset(&filterVector[filterDegree], 0, (fftSize - filterDegree) * sizeof(DSPCOMPLEX));

  mpFilterFFT->do_FFT();

  mInpIdx = 0;

  delete BandPass;
}

void fftFilter::setBand(int32_t low, int32_t high, int32_t rate)
{
  BandPassFIR * const BandPass = new BandPassFIR((int16_t)mFilterDegree, low, high, rate);

  for (int32_t i = 0; i < mFilterDegree; i++)
  {
    mpFilterVector[i] = (BandPass->getKernel())[i];
  }

  for (int32_t i = mFilterDegree; i < mFftSize; i++)
  {
    mpFilterVector[i] = 0.0f;
  }

  // filterVector [i] = conj ((BandPass -> getKernel ()) [i]);
  //memset(&filterVector[filterDegree], 0, (fftSize - filterDegree) * sizeof(DSPCOMPLEX));

  mpFilterFFT->do_FFT();

  mInpIdx = 0;

  delete BandPass;
}

void fftFilter::setLowPass(int32_t low, int32_t rate)
{
  LowPassFIR * const LowPass = new LowPassFIR((int16_t)mFilterDegree, low, rate);

  for (int32_t i = 0; i < mFilterDegree; i++)
  {
    mpFilterVector[i] = (LowPass->getKernel())[i];
  }

  for (int32_t i = mFilterDegree; i < mFftSize; i++)
  {
    mpFilterVector[i] = 0.0f;
  }

  //memset(&filterVector[filterDegree], 0, (fftSize - filterDegree) * sizeof(DSPCOMPLEX));

  mpFilterFFT->do_FFT();

  mInpIdx = 0;

  delete LowPass;
}

DSPFLOAT fftFilter::Pass(DSPFLOAT x)
{
  DSPFLOAT sample;

  sample = real(mpFFT_C[mInpIdx]);
  mpFFT_A[mInpIdx] = x;

  if (++mInpIdx >= mNumofSamples)
  {
    mInpIdx = 0;

    for (int32_t i = mNumofSamples; i < mFftSize; i++)
    {
      mpFFT_A[i] = 0.0f;
    }

    //memset(&FFT_A[NumofSamples], 0, (fftSize - NumofSamples) * sizeof(DSPCOMPLEX));

    mpMyFFT->do_FFT();

    for (int32_t j = 0; j < mFftSize; j++)
    {
      mpFFT_C[j] = mpFFT_A[j] * mpFilterVector[j];
      mpFFT_C[j] = DSPCOMPLEX(real(mpFFT_C[j]) * 3, imag(mpFFT_C[j]) * 3);
    }

    mpMyIFFT->do_IFFT();

    for (int32_t j = 0; j < mOverlapSize; j++)
    {
      mpFFT_C[j]   += mpOverloop[j];
      mpOverloop[j] = mpFFT_C[mNumofSamples + j];
    }
  }

  return sample;
}

DSPCOMPLEX fftFilter::Pass(DSPCOMPLEX z)
{
  DSPCOMPLEX sample;

  sample     = mpFFT_C[mInpIdx];
  mpFFT_A[mInpIdx] = DSPCOMPLEX(real(z), imag(z));

  if (++mInpIdx >= mNumofSamples)
  {
    mInpIdx = 0;

    for (int32_t i = mNumofSamples; i < mFftSize; i++)
    {
      mpFFT_A[i] = 0.0f;
    }

    //memset(&FFT_A[NumofSamples], 0, (fftSize - NumofSamples) * sizeof(DSPCOMPLEX));

    mpMyFFT->do_FFT();

    for (int32_t j = 0; j < mFftSize; j++)
    {
      mpFFT_C[j] = mpFFT_A[j] * mpFilterVector[j];
    }

    mpMyIFFT->do_IFFT();

    for (int32_t j = 0; j < mOverlapSize; j++)
    {
      mpFFT_C[j]   += mpOverloop[j];
      mpOverloop[j] = mpFFT_C[mNumofSamples + j];
    }
  }

  return sample;
}
