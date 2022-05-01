/*
 *    Copyright (C)  2014
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

#ifndef __RADIO__
#define __RADIO__

#include "fft.h"
#include "fir-filters.h"
#include "fm-constants.h"
#include "iqdisplay.h"
#include "ringbuffer.h"
#include "scope.h"
#include "ui_radio.h"
#include <QDialog>
#include <QInputDialog>
#include <QLineEdit>
#include <QQueue>
#include <QTimer>
#include <QWheelEvent>
#include <qwt.h>
#include <sndfile.h>

class keyPad;
class QSettings;
class fmProcessor;
class rdsDecoder;
class fft_scope;
class audioSink;
class deviceHandler;
class programList;

/*
 *	The main gui object. It inherits from
 *	QDialog and the generated form
 */
class RadioInterface : public QDialog, private Ui_sdr_j_fm {
  Q_OBJECT
public:
  RadioInterface(QSettings *, QString, int32_t, QWidget *parent = NULL);
  ~RadioInterface();

private:
  enum Keyboard { NORMAL = 0, CONTROL = 1, ALT = 2, SHIFT = 3 };
  //	Processing modes
  enum class ERunStates
  {
   IDLE     = 0100,
   PAUSED   = 0101,
   RUNNING  = 0102,
   STOPPING = 0103
  };

  bool ExtioLock;
  int16_t outputDevice;
  void localConnects();
  void dumpControlState(QSettings *);

  RingBuffer<double> *hfBuffer;
  RingBuffer<double> *lfBuffer;
  Scope *hfScope;
  Scope *lfScope;
  int16_t scopeAmplification;
  bool HFAverager;
  bool mAfcActive{false};
  float mAfcAlpha{1.0f};
  int32_t mAfcCurrOffFreq{0};

  keyPad *mykeyPad;
  QSettings *fmSettings;
  int32_t inputRate;
  int32_t fmRate;
  int32_t workingRate;
  int32_t audioRate;
  int16_t displaySize;
  int16_t rasterSize;
  int16_t spectrumSize;
  int32_t averageCount;
  int16_t repeatRate;
  double *displayBuffer;
  int16_t filterDepth;
  audioSink *our_audioSink;
  int8_t channelSelector;
  deviceHandler *myRig;
  int16_t *outTable;
  int16_t numberofDevices;

  uint8_t HFviewMode;
  uint8_t LFviewMode;
  uint8_t inputMode;
  int16_t currAttSliderValue;
  DSPFLOAT attValueL;
  DSPFLOAT attValueR;

  int32_t fmBandwidth;
  int32_t LOFrequency;
  int32_t currentFreq;

  void restoreGUIsettings(QSettings *);
  void setDetectorScreen(int16_t);
  void reset_afc();

  int32_t mapIncrement(int32_t);
  int32_t IncrementInterval(int16_t);
  int32_t setTuner(int32_t);
  void Display(int32_t);
  QTimer *autoIncrementTimer;
  int16_t IncrementIndex;
  int32_t autoIncrement_amount;
  int32_t fmIncrement;
  int32_t minLoopFrequency;
  int32_t maxLoopFrequency;

  void set_incrementFlag(int16_t);
  void stopIncrementing();
  int32_t get_Increment_for(int16_t);

  void stop_lcdTimer();
  int32_t Panel;
  int16_t CurrentRig;
  QTimer *displayTimer;
  /*
   *	dumping
   */
  bool sourceDumping;
  SNDFILE *dumpfilePointer;

  bool audioDumping;
  SNDFILE *audiofilePointer;

  fmProcessor *myFMprocessor;
  rdsDecoder *myRdsDecoder;
  int8_t rdsModus;
  int8_t viewKeuze;

  QString RadioText;
  QString StationLabel;
  void IncrementFrequency(int32_t);
  int16_t thresHold;
  int32_t currentPIcode;
  int32_t frequencyforPICode;
  int16_t logTime;
  FILE *logFile;
  ERunStates runMode;

  void setup_HFScope();
  void setup_LFScope();
  bool squelchMode;
  void resetSelector();
  int32_t mapRates(int32_t);

  programList *myList;
  QLineEdit *myLine;
  /*
   *	The private slots link the GUI elements
   *	to the GUI code
   */
private slots:
  void setStart();
  void updateTimeDisplay();
  void clickPause();

  void setInputMode(const QString &);
  void setAttenuation(int);
  void setIQBalance(int);

  void setStreamOutSelector(int);
  void abortSystem(int);
  void TerminateProcess();
  void make_newProcessor();
  void stopDumping();
  void setDevice(const QString &);
  //	void	setVolume		(int);
  void set_dumping();
  void set_audioDump();

  void setfmBandwidth(const QString &);
  void setfmBandwidth(int);
  void setfmMode(const QString &);
  void setfmRdsSelector(const QString &);
  void setfmDecoder(const QString &);
  void setfmChannelSelector(const QString &);
  void setfmDeemphasis(const QString &);
  void setfmStereoSlider(int);
  void setfmLFcutoff(const QString &);

  void autoIncrement_timeout();
  void autoIncrementButton();
  void autoDecrementButton();
  void set_fm_increment(int);
  void set_minimum(int);
  void set_maximum(int);
  void IncrementButton();
  void DecrementButton();

  bool setupSoundOut(QComboBox *, audioSink *, int32_t, int16_t *);
  void set_squelchValue(int);
  void set_squelchMode();
  void set_freqSave();
  void handle_myLine();

public slots:
  void setHFplotterView(int);
  void handle_freqButton();
  void newFrequency(int);
  void hfBufferLoaded();
  void lfBufferLoaded();
  void wheelEvent(QWheelEvent *);
  void setLogging(const QString &);
  void setLogsaving();
  void AdjustFrequency(int);
  void setCRCErrors(int);
  void setSyncErrors(int);
  void setbitErrorRate(double);
  void setGroup(int);
  void setPTYCode(int);
  void setPiCode(int);
  void clearStationLabel();
  void setStationLabel(const QString &);
  void clearRadioText();
  void setRadioText(const QString &);
  void setAFDisplay(int);
  void setRDSisSynchronized(bool);
  void setMusicSpeechFlag(int);
  void clearMusicSpeechFlag();
  void showStrength(float, float);
  void scanresult();
  void closeEvent(QCloseEvent *event);

  //
  //	and for the extio handling
  void set_ExtFrequency(int);
  void set_ExtLO(int);
  void set_lockLO();
  void set_unlockLO();
  void set_stopHW();
  void set_startHW();
  void set_changeRate(int);
};

#endif
