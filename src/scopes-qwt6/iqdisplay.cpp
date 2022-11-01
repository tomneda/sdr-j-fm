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
static SpectrogramData * sIQData = nullptr;

IQDisplay::IQDisplay(QwtPlot * plot, int16_t x) : QwtPlotSpectrogram()
{
  QwtLinearColorMap * colorMap = new QwtLinearColorMap(Qt::darkBlue, Qt::yellow);

  setRenderThreadCount(1);
  mNoPointsPerRadius = 100;
  mNoMaxPointsOnField = (2 * mNoPointsPerRadius) * (2 * mNoPointsPerRadius);
  mpQwtPlot = plot;
  mAmount = x;
  mInpInx = 0;
  //mpPoints = new DSPCOMPLEX[mAmount];
  //memset(mpPoints, 0, mAmount * sizeof(DSPCOMPLEX));
  this->setColorMap(colorMap);
  mpPlotData1 = new double[mNoMaxPointsOnField];
  mpPlotData2 = new double[mNoMaxPointsOnField];
  memset(mpPlotData1, 0, mNoMaxPointsOnField * sizeof(double));
  sIQData = new SpectrogramData(mpPlotData2, 0, 2 * mNoPointsPerRadius, 2 * mNoPointsPerRadius, 2 * mNoPointsPerRadius, 50.0);
  this->setData(sIQData);
  plot->enableAxis(QwtPlot::xBottom, false);
  plot->enableAxis(QwtPlot::yLeft, false);
  plot->enableAxis(QwtPlot::xTop, false);
  plot->enableAxis(QwtPlot::yRight, false);
  this->setDisplayMode(QwtPlotSpectrogram::ImageMode, true);
  this->attach(mpQwtPlot);
  mpQwtPlot->replot();
}

IQDisplay::~IQDisplay()
{
  this->detach();
  delete[] mpPlotData1;
  //delete[] mpPoints;
  //	delete		IQData;
}

void IQDisplay::DisplayIQ(const DSPCOMPLEX z, const float scale)
{
  int32_t h = (int32_t)(scale * real(z));
  int32_t v = (int32_t)(scale * imag(z));
  //int32_t v = (int32_t)(scale * -0.5);

  if      (v >=  mNoPointsPerRadius) v =   mNoPointsPerRadius - 1;
  else if (v <= -mNoPointsPerRadius) v = -(mNoPointsPerRadius - 1);

  if      (h >= mNoPointsPerRadius)  h =   mNoPointsPerRadius - 1;
  else if (h <= -mNoPointsPerRadius) h = -(mNoPointsPerRadius - 1);

  //mpPoints[mInpInx] = DSPCOMPLEX(x, y);
  mpPlotData1[(v + mNoPointsPerRadius - 1) * 2 * mNoPointsPerRadius + h + mNoPointsPerRadius - 1] = 50;

  if (++mInpInx < mAmount)
    return;

  // we need an extra data buffer as mpQwtPlot->replot() seems to take a while in the background
  memcpy(mpPlotData2, mpPlotData1, mNoMaxPointsOnField * sizeof(double));
  //this->detach();
  //this->setData(IQData);
  //this->setDisplayMode(QwtPlotSpectrogram::ImageMode, true);
  //this->attach(mpQwtPlot);
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
