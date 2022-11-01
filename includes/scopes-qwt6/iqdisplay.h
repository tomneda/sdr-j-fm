/*
 *    Copyright (C) 2008, 2009, 2010
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair Programming
 *
 *    This file is part of the SDR-J.
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
#ifndef __IQDISPLAY
#define __IQDISPLAY

#include <qwt.h>
//#include <qwt_slider.h>
#include <qwt_plot.h>
//#include <qwt_plot_curve.h>
//#include <qwt_plot_marker.h>
//#include <qwt_plot_grid.h>
//#include <qwt_dial.h>
//#include <qwt_dial_needle.h>
#include <qwt_plot_spectrogram.h>
#include <qwt_color_map.h>
//#include <qwt_plot_spectrogram.h>
//#include <qwt_scale_widget.h>
//#include <qwt_scale_draw.h>
//#include <qwt_plot_zoomer.h>
//#include <qwt_plot_panner.h>
//#include <qwt_plot_layout.h>

#include	<stdint.h>
#include	"fm-constants.h"

class IQDisplay : public QObject, public QwtPlotSpectrogram
{
  Q_OBJECT
public:
  IQDisplay(QwtPlot *, int16_t);
  ~IQDisplay(void);
  void DisplayIQ(DSPCOMPLEX, float);
  void DisplayIQVec(DSPCOMPLEX *, float);

private:
  int32_t mAmount;
  double * mpPlotData1;
  double * mpPlotData2;
  //DSPCOMPLEX * mpPoints;
  QwtPlot * mpQwtPlot;
  int32_t mNoPointsPerRadius;
  int32_t mNoMaxPointsOnField;
  int32_t mInpInx;

private slots:

};
#endif
