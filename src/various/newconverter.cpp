/*
 *    Copyright (C) 2011, 2012, 2013
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
 *    -sdr-j-fm is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with sdr-j-fm; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include  "newconverter.h"
#include <assert.h>

newConverter::newConverter (int32_t inRate, int32_t outRate, int32_t inSize)
{

  mInRate     = inRate;
  mOutRate    = outRate;
  mInputLimit = inSize;
  mRatio      = double(outRate) / inRate;
  //fprintf(stderr, "ratio = %f\n", mRatio);
  mOutputLimit = inSize * mRatio + 1; // round up as we can get an memory overflow

  int err = 0;
  //mpConverter		= src_new (SRC_SINC_BEST_QUALITY, 2, &err);
  //mpConverter		= src_new (SRC_LINEAR, 2, &err);
  mpConverter = src_new(SRC_SINC_MEDIUM_QUALITY, 2, &err);
  assert(err == 0);

  mpSrc_data               = new SRC_DATA;
  mpInBuffer               = new float [2 * mInputLimit + 20];
  mpOutBuffer              = new float [2 * mOutputLimit + 20];
  mpSrc_data->data_in      = mpInBuffer;
  mpSrc_data->data_out     = mpOutBuffer;
  mpSrc_data->src_ratio    = mRatio;
  mpSrc_data->end_of_input = 0;
  mInpIdx                  = 0;
}

newConverter::~newConverter (void)
{
  src_delete(mpConverter);
  delete [] mpInBuffer;
  delete [] mpOutBuffer;
  delete    mpSrc_data;
}

bool newConverter::convert(DSPCOMPLEX v, DSPCOMPLEX *out, int32_t *amount)
{
  mpInBuffer [2 * mInpIdx]     = real(v);
  mpInBuffer [2 * mInpIdx + 1] = imag(v);
  mInpIdx++;

  if (mInpIdx < mInputLimit)
  {
    return false;
  }

  mpSrc_data->input_frames  = mInpIdx;
  mpSrc_data->output_frames = mOutputLimit + 10;
  const int res = src_process(mpConverter, mpSrc_data);

  if (res != 0)
  {
    fprintf(stderr, "error %s\n", src_strerror(res));
    assert(0);
    return false;
  }

  mInpIdx = 0;
  const int32_t framesOut = mpSrc_data->output_frames_gen;

  for (int32_t i = 0; i < framesOut; i++)
  {
    out [i] = DSPCOMPLEX(mpOutBuffer[2 * i], mpOutBuffer[2 * i + 1]);
  }

  *amount = framesOut;

  return true;
}

int32_t newConverter::getOutputsize(void)
{
  return mOutputLimit;
}

