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

  mpMyFFT = new common_fft(mFftSize);
  mpMyFFTVec = mpMyFFT->getVector();

  mpMyIFFT = new common_ifft(mFftSize);
  mpMyIFFTVec = mpMyIFFT->getVector();

  mpFilterFFT     = new common_fft(mFftSize);
  mpFilterVector  = mpFilterFFT->getVector();
  mpRfilterVector = new DSPFLOAT[mFftSize];

  mpOverloop = new DSPCOMPLEX[mOverlapSize];
  mInpIdx      = 0;

  for (i = 0; i < mFftSize; i++)
  {
    mpMyFFTVec[i]         = 0;
    mpMyIFFTVec[i]         = 0;
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
  const DSPFLOAT sample = real(mpMyIFFTVec[mInpIdx]);
  mpMyFFTVec[mInpIdx] = x;

  if (++mInpIdx >= mNumofSamples)
  {
    mInpIdx = 0;

    for (int32_t i = mNumofSamples; i < mFftSize; i++)
    {
      mpMyFFTVec[i] = 0.0f;
    }

    mpMyFFT->do_FFT();

    for (int32_t j = 0; j < mFftSize; j++)
    {
      mpMyIFFTVec[j] = mpMyFFTVec[j] * mpFilterVector[j];
      mpMyIFFTVec[j] = DSPCOMPLEX(real(mpMyIFFTVec[j]) * 3, imag(mpMyIFFTVec[j]) * 3); // ?!?!?
    }

    mpMyIFFT->do_IFFT();

    for (int32_t j = 0; j < mOverlapSize; j++)
    {
      mpMyIFFTVec[j] += mpOverloop[j];
      mpOverloop[j] = mpMyIFFTVec[mNumofSamples + j];
    }
  }

  return sample;
}

DSPCOMPLEX fftFilter::Pass(DSPCOMPLEX z)
{
  const DSPCOMPLEX sample = mpMyIFFTVec[mInpIdx];
  mpMyFFTVec[mInpIdx] = z;

  if (++mInpIdx >= mNumofSamples)
  {
    mInpIdx = 0;

    for (int32_t i = mNumofSamples; i < mFftSize; i++)
    {
      mpMyFFTVec[i] = 0.0f;
    }


    mpMyFFT->do_FFT();

    for (int32_t j = 0; j < mFftSize; j++)
    {
      mpMyIFFTVec[j] = mpMyFFTVec[j] * mpFilterVector[j];
    }

    mpMyIFFT->do_IFFT();

    for (int32_t j = 0; j < mOverlapSize; j++)
    {
      mpMyIFFTVec[j] += mpOverloop[j];
      mpOverloop[j] = mpMyIFFTVec[mNumofSamples + j];
    }
  }

  return sample;
}

fftFilterHilbert::fftFilterHilbert(int32_t size, int16_t degree)
  : fftFilter(size, degree)
{
  setHilbert();
}

DSPCOMPLEX fftFilterHilbert::Pass(DSPFLOAT x)
{
  return fftFilter::Pass(DSPCOMPLEX(x, 0));
}

void fftFilterHilbert::setHilbert()
{
  // set the frequency coefficients directly (set negative spectrum to zero)
  if ((mFftSize & 0x1) == 0)
  {
    mpFilterVector[0] = 1.0f;

    for (int32_t i = 1; i < mFftSize / 2; i++)
    {
      mpFilterVector[i] = 2.0f;
    }

    mpFilterVector[mFftSize / 2] = 1.0f;

    for (int32_t i = mFftSize / 2 + 1; i < mFftSize; i++)
    {
      mpFilterVector[i] = 0.0f;
    }
  }
  else
  {
    mpFilterVector[0] = 1.0f;

    for (int32_t i = 1; i < (mFftSize + 1) / 2; i++)
    {
      mpFilterVector[i] = 2.0f;
    }

    for (int32_t i = (mFftSize - 1) / 2 + 1; i < mFftSize; i++)
    {
      mpFilterVector[i] = 0.0f;
    }
  }

  mInpIdx = 0;
}

