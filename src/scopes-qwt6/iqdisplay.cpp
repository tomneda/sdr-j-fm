/*
 *    Copyright (C) 2008, 2009, 2010, 2011, 2012, 2013
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair Programming
 *
 *    This file is part of the SDR-J (JSDR).
 *    Many of the ideas as implemented in JSDR are derived from
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
#include "iqdisplay.h"
#include "spectrogramdata.h"
/*
 *	iq circle plotter
 */
static IqScopeData * sIQData = nullptr;

IQDisplay::IQDisplay(QwtPlot * plot, int16_t x) : QwtPlotSpectrogram()
{
  QwtLinearColorMap * colorMap = new QwtLinearColorMap(Qt::darkBlue, Qt::yellow);

  setRenderThreadCount(1);
  mNoPointsPerRadius = 50;
  mNoPointsColOrRow = 2 * mNoPointsPerRadius + 1; // zero position is a extra row and column
  mNoMaxPointsOnField = mNoPointsColOrRow * mNoPointsColOrRow;
  mpQwtPlot = plot;
  mAmount = x;
  mInpInx = 0;
  //mpPoints = new DSPCOMPLEX[mAmount];
  //memset(mpPoints, 0, mAmount * sizeof(DSPCOMPLEX));
  this->setColorMap(colorMap);

  mpPlotData1 = new double[mNoMaxPointsOnField];
  mpPlotData2 = new double[mNoMaxPointsOnField];

  memset(mpPlotData1, 0, mNoMaxPointsOnField * sizeof(double));
  memset(mpPlotData2, 0, mNoMaxPointsOnField * sizeof(double));

  sIQData = new IqScopeData(mpPlotData2, mNoPointsColOrRow, 50.0);
  //sIQData->setResampleMode(QwtMatrixRasterData::BicubicInterpolation);
  //sIQData->setAttribute(QwtMatrixRasterData::WithoutGaps); // this causes a exception in debug mode
  this->setData(sIQData);
  plot->enableAxis(QwtPlot::xBottom, false);
  plot->enableAxis(QwtPlot::yLeft, false);
  plot->enableAxis(QwtPlot::xTop, false);
  plot->enableAxis(QwtPlot::yRight, false);
  this->setDisplayMode(QwtPlotSpectrogram::ImageMode, true);
  this->attach(mpQwtPlot);
  mpQwtPlot->setFixedHeight(2*mNoPointsColOrRow);
  mpQwtPlot->setFixedWidth(2*mNoPointsColOrRow);
  mpQwtPlot->replot();
}

IQDisplay::~IQDisplay()
{
  this->detach();
  delete sIQData;
  delete[] mpPlotData2;
  delete[] mpPlotData1;
  //delete[] mpPoints;
}

template<class T> inline void symmetric_limit(T & ioVal, const T iLimit)
{
    if      (ioVal >  iLimit) ioVal =  iLimit;
    else if (ioVal < -iLimit) ioVal = -iLimit;
  //if ((x) > (l)) x = (l); else if (x < -(l)) x = -(l);
}

void IQDisplay::DisplayIQ(const DSPCOMPLEX z, const float scale)
{
  int32_t h = (int32_t)(mNoPointsPerRadius * scale * real(z));
  int32_t v = (int32_t)(mNoPointsPerRadius * scale * imag(z));
  //int32_t v = (int32_t)(scale * -0.5);

  // the field has width of 201 and height of 201, the middle has index 100

  symmetric_limit(v, mNoPointsPerRadius);
  symmetric_limit(h, mNoPointsPerRadius);

  //mpPoints[mInpInx] = DSPCOMPLEX(x, y);
  const int32_t idx = (v + mNoPointsPerRadius) * mNoPointsColOrRow + h + mNoPointsPerRadius;
  assert(idx >= 0);
  assert(idx < mNoMaxPointsOnField);
  //if (mpPlotData1[idx] < 40) mpPlotData1[idx] += 10;
  mpPlotData1[idx] = 50;

  if (++mInpInx < mAmount)
    return;

  // we need an extra data buffer as mpQwtPlot->replot() seems to take a while in the background
  memcpy(mpPlotData2, mpPlotData1, mNoMaxPointsOnField * sizeof(double));

  {
    // draw cross
    for (int32_t i = -mNoPointsPerRadius; i <= mNoPointsPerRadius; ++i)
    {
      const int32_t ih = mNoPointsPerRadius * mNoPointsColOrRow + i + mNoPointsPerRadius;
      const int32_t iv = (i + mNoPointsPerRadius) * mNoPointsColOrRow + mNoPointsPerRadius;
      mpPlotData2[ih] = mpPlotData2[iv] = 10;
    }

    // draw unit circle
    constexpr int32_t MAX_CIRCLE_POINTS = 45;
    constexpr float SCALE = 0.5f;
    for (int32_t i = 0; i < MAX_CIRCLE_POINTS; ++i)
    {
      const float phase = 0.5f * M_PI * i / MAX_CIRCLE_POINTS;

      const int32_t h = (int32_t)(mNoPointsPerRadius * SCALE * cosf(phase));
      const int32_t v = (int32_t)(mNoPointsPerRadius * SCALE * sinf(phase));

      const int32_t ior = (v + mNoPointsPerRadius) * mNoPointsColOrRow + h + mNoPointsPerRadius;
      const int32_t iol = (v + mNoPointsPerRadius) * mNoPointsColOrRow - h + mNoPointsPerRadius;
      const int32_t ibr = (-v + mNoPointsPerRadius) * mNoPointsColOrRow + h + mNoPointsPerRadius;
      const int32_t ibl = (-v + mNoPointsPerRadius) * mNoPointsColOrRow - h + mNoPointsPerRadius;
      mpPlotData2[ior] = mpPlotData2[iol] = mpPlotData2[ibr] = mpPlotData2[ibl] = 10;
    }
  }

  mpQwtPlot->replot();

  memset(mpPlotData1, 0, mNoMaxPointsOnField * sizeof(double));
  mInpInx = 0;
}

void IQDisplay::DisplayIQVec(const DSPCOMPLEX * const z, const int32_t n, const float scale)
{
  for (int32_t i = 0; i < n; ++i)
  {
    DisplayIQ(z[i], scale);
  }
}
