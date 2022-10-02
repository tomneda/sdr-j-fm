/*
 *    Copyright (C) 2014
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair Computing
 *
 *    This file is part of the fm software
 *
 *    fm software is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    fm software is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include "radio.h"
#include "audiosink.h"
#include "fm-constants.h"
#include "fm-demodulator.h"
#include "fm-processor.h"
#include "popup-keypad.h"
#include "rds-decoder.h"
#include "scope.h"
#include <QDateTime>
#include <QDebug>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QTabWidget>
#include <QHeaderView>
#include <QSettings>
#include <Qt>

#include "device-handler.h"
#include "filereader.h"
#ifdef HAVE_PMSDR
#include "pmsdr.h"
#endif
#ifdef HAVE_SDRPLAY
#include "sdrplay-handler.h"
#endif
#ifdef HAVE_SDRPLAY_V3
#include "sdrplay-handler-v3.h"
#endif
#ifdef HAVE_AIRSPY
#include "airspy-handler.h"
#endif
#ifdef HAVE_DABSTICK
#include "rtlsdr-handler.h"
#endif
#ifdef HAVE_EXTIO
#include "extio-handler.h"
#endif
#ifdef HAVE_HACKRF
#include "hackrf-handler.h"
#endif
#ifdef HAVE_LIME
#include "lime-handler.h"
#endif
#ifdef HAVE_COLIBRI
#include "colibri-handler.h"
#endif
#ifdef HAVE_PLUTO
#include "pluto-handler.h"
#endif
#ifdef HAVE_ELAD_S1
#include "elad-s1.h"
#endif
#ifdef __MINGW32__
#include <iostream>
#include <windows.h>
#endif

static
// int16_t	delayTable [] = {15, 13, 11, 10, 9, 8, 7, 5, 3, 2, 1};

constexpr int16_t delayTable[] = { 1, 3, 5, 7, 9, 10, 15 };
constexpr int16_t delayTableSize = ((int)(sizeof(delayTable) / sizeof(int16_t)));

static constexpr char TXT_DUMP_INPUT_STREAM[] = "Dump Input Stream";
static constexpr char TXT_DUMP_AUDIO[] = "Dump Audio";
static constexpr char TXT_WRITING[] = "WRITING";

/*
 *	We use the creation function merely to set up the
 *	user interface and make the connections between the
 *	gui elements and the handling agents. All real action
 *	is embedded in actions, initiated by gui buttons
 */
/**
 * @file gui.cpp
 * @brief gui.cpp : Defines the functions for the GUI of the FM software
 * @author Jan van Katwijk
 * @version 0.98
 * @date 2015-01-07
 */
RadioInterface::RadioInterface(QSettings *Si, QString stationList,
                               int32_t outputRate, QWidget *parent)
  : QDialog(parent)
  , mSaveName(stationList)
{
  int16_t i;
  QString h;
  int     k;

  setupUi(this);

  setWindowFlag(Qt::WindowContextHelpButtonHint, false);
  setWindowFlag(Qt::WindowMinimizeButtonHint, true);
  setWindowFlag(Qt::WindowMaximizeButtonHint, true);

  thermoPeakLevelLeft->setFillBrush(Qt::darkBlue);
  thermoPeakLevelRight->setFillBrush(Qt::darkBlue);
  thermoPeakLevelLeft->setAlarmBrush(Qt::red);
  thermoPeakLevelRight->setAlarmBrush(Qt::red);
  thermoPeakLevelLeft->setAlarmEnabled(true);
  thermoPeakLevelRight->setAlarmEnabled(true);



  fmSettings = Si;

  reset_afc();

  runMode     = ERunStates::IDLE;
  squelchMode = false;
  //
  //	dummies, needed for a.o. LFScope
  this->inputRate = 192000;
  this->fmRate    = 192000;
  /**
   *	We allow the user to set the displaysize
   *	(as long as it is reasonable)
   */
  this->displaySize = fmSettings->value("displaySize", 512).toInt();
  if ((displaySize & (displaySize - 1)) != 0)
  {
    displaySize = 1024;
  }
  if (displaySize < 128)
  {
    displaySize = 128;
  }
  displayBuffer = new double[this->displaySize];
  memset(displayBuffer, 0, displaySize * sizeof(double));
  /**
   *	we allow the user to set the spectrumSize,
   *	there is however a sanity check
   */

  this->spectrumSize = fmSettings->value("spectrumSize", 2048).toInt();
  if ((spectrumSize & (spectrumSize - 1)) != 0)
  {
    spectrumSize = 2048;
  }
  if (spectrumSize < displaySize)
  {
    spectrumSize = 4 * displaySize;
  }
  if (spectrumSize % displaySize != 0)
  {
    spectrumSize = 4 * displaySize;
  }

  this->rasterSize   = fmSettings->value("rasterSize", 50).toInt();
  this->repeatRate   = fmSettings->value("repeatRate", 10).toInt();
  this->averageCount = fmSettings->value("averageCount", 5).toInt();
  this->audioRate    = fmSettings->value("audioRate", outputRate).toInt();
  //
  this->workingRate = 48000;
#ifdef HAVE_PMSDR
  deviceSelector->addItem("pmsdr");
#endif
#ifdef HAVE_EXTIO
  deviceSelector->addItem("extio");
#endif
#ifdef HAVE_SDRPLAY
  deviceSelector->addItem("sdrplay");
#endif
#ifdef HAVE_SDRPLAY_V3
  deviceSelector->addItem("sdrplay-v3");
#endif
#ifdef HAVE_DABSTICK
  deviceSelector->addItem("dabstick");
#endif
#ifdef HAVE_AIRSPY
  deviceSelector->addItem("airspy");
#endif
#ifdef HAVE_HACKRF
  deviceSelector->addItem("hackrf");
#endif
#ifdef HAVE_LIME
  deviceSelector->addItem("lime");
#endif
#ifdef HAVE_COLIBRI
  deviceSelector->addItem("colibri");
#endif
#ifdef HAVE_PLUTO
  deviceSelector->addItem("pluto");
#endif
#ifdef HAVE_ELAD_S1
  deviceSelector->addItem("elad-s1");
#endif
  myFMprocessor = nullptr;
  our_audioSink = new audioSink(this->audioRate, 16384);
  outTable      = new int16_t[our_audioSink->numberofDevices() + 1];
  for (i = 0; i < our_audioSink->numberofDevices(); i++)
  {
    outTable[i] = -1;
  }

  if (!setupSoundOut(streamOutSelector, our_audioSink, this->audioRate,
                     outTable))
  {
    fprintf(stderr, "Cannot open any output device\n");
    abortSystem(33);
  }
  /**
   *	Use, if possible, the outputstream the user had previous time
   */
  h = fmSettings->value("streamOutSelector", "default").toString();
  k = streamOutSelector->findText(h);
  if (k != -1)
  {
    streamOutSelector->setCurrentIndex(k);
    setStreamOutSelector(k);
  }

  setup_HFScope();
  setup_LFScope();
  sourceDumping    = false;
  audioDumping     = false;
  dumpfilePointer  = nullptr;
  audiofilePointer = nullptr;
  //
  //	Set relevant sliders etc to the value they had last time
  restoreGUIsettings(fmSettings);
  //
  //
  incrementingFlag->setStyleSheet("QLabel {background-color:blue}");
  incrementingFlag->setText(" ");
  IncrementIndex = 0;
  //	settings for the auto tuner
  IncrementIndex = fmSettings->value("IncrementIndex", 0).toInt();
  fmIncrement    = fmSettings->value("fm_increment", 100).toInt();

  minLoopFrequency = fmSettings->value("min_loop_frequency", 86500).toInt();
  if (minLoopFrequency == 0)
  {
    minLoopFrequency = 86500;
  }

  maxLoopFrequency = fmSettings->value("max_loop_frequency", 110000).toInt();
  if (maxLoopFrequency == 0)
  {
    maxLoopFrequency = 110000;
  }

  fm_increment->setValue(fmIncrement);
  minimumSelect->setValue(KHz(minLoopFrequency) / MHz(1));
  maximumSelect->setValue(KHz(maxLoopFrequency) / MHz(1));

  //	he does the connections from the gui buttons, sliders etc
  localConnects();

  mykeyPad = new keyPad(this);

  connect(freqButton, SIGNAL(clicked()), this, SLOT(handle_freqButton()));

  //	Create a timer for autoincrement/decrement of the tuning
  autoIncrementTimer = new QTimer();
  autoIncrementTimer->setSingleShot(true);
  autoIncrementTimer->setInterval(5000);

  connect(autoIncrementTimer, SIGNAL(timeout()), this, SLOT(autoIncrement_timeout()));

  //	create a timer for displaying the "real" time
  displayTimer = new QTimer();
  displayTimer->setInterval(1000);
  connect(displayTimer, SIGNAL(timeout()), this, SLOT(updateTimeDisplay()));
  //
  //
  //	Display the version
  QString v = QString("Version SHA1: ") + QString(GITHASH).toUpper();
  //QString v = "sdrJ-FM -V";
  //v.append(CURRENT_VERSION);
  systemindicator->setText(v.toLatin1().data());

  ExtioLock = false;
  logFile   = nullptr;
  pauseButton->setText(QString("Pause"));
  dumpButton->setText(TXT_DUMP_INPUT_STREAM);
  sourceDumping = false;
  audioDump->setText(TXT_DUMP_AUDIO);
  audioDumping       = false;
  currentPIcode      = 0;
  frequencyforPICode = 0;
  theSelector->hide();
  myRig = new deviceHandler();
  setTuner(Khz(94700));
  inputRate = myRig->getRate();
  fmRate    = mapRates(inputRate);

  filterDepth = fmSettings->value("filterDepth", 5).toInt();
  hfScope->setBitDepth(myRig->bitDepth());
  lfScope->setBitDepth(myRig->bitDepth());
  //
  thresHold = fmSettings->value("threshold", 20).toInt();

  connect(fm_increment,  SIGNAL(valueChanged(int)), this, SLOT(set_fm_increment(int)));
  connect(minimumSelect, SIGNAL(valueChanged(int)), this, SLOT(set_minimum(int)));
  connect(maximumSelect, SIGNAL(valueChanged(int)), this, SLOT(set_maximum(int)));

  displayTimer->start(1000);

  scrollStationList->setWidgetResizable(true);

  mpTableWidget = new QTableWidget(0, 2, this);

  scrollStationList->setWidget(mpTableWidget);

  mpTableWidget->setHorizontalHeaderLabels(QStringList() << tr("Station") << tr("Frequency"));
  mpTableWidget->setEditTriggers(QAbstractItemView::EditKeyPressed); // only F2 is allowed, no double click (this would delete the entry)
  mpTableWidget->verticalHeader()->setVisible(false); // remove index number column

  loadTable();

  connect(mpTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(tableSelect(int,int)));
  connect(mpTableWidget, SIGNAL(cellDoubleClicked(int,int)), this, SLOT(removeRow(int,int)));

  myLine = nullptr;
  connect(freqSave, SIGNAL(clicked()), this, SLOT(set_freqSave()));
  connect(cbAfc, &QAbstractButton::clicked, this, [this](bool checked){ mAfcActive = checked; reset_afc(); } );

  resetSelector();

  QTimer::singleShot(100, this, [this](){ setDevice("dabstick"); } );
  //setDevice("dabstick");
}

//
//	The end of all
RadioInterface::~RadioInterface()
{
  delete hfScope;
  delete lfScope;

  delete autoIncrementTimer;
  delete displayTimer;
  delete our_audioSink;
  delete[] outTable;
}
//
//	Function used to "dump" settings into the ini file
//	pointed to by s

void RadioInterface::dumpControlState(QSettings *s)
{
  Q_ASSERT(s);

  s->setValue("rasterSize", rasterSize);
  s->setValue("averageCount", averageCount);

  s->setValue("repeatRate", repeatRate);

  s->setValue("fm_increment", fm_increment->value());
  s->setValue("spectrumAmplitudeSlider_hf", spectrumAmplitudeSlider_hf->value());
  s->setValue("spectrumAmplitudeSlider_lf", spectrumAmplitudeSlider_lf->value());
  s->setValue("IQbalanceSlider", IQbalanceSlider->value());
  s->setValue("afc", cbAfc->checkState());

  //	now setting the parameters for the fm decoder
  s->setValue("fmFilterSelect", fmFilterSelect->currentText());
  s->setValue("fmFilterDegree", fmFilterDegree->value());

  s->setValue("fmMode", fmMode->currentText());
  s->setValue("fmDecoder", fmDecoder->currentText());
  s->setValue("volumeHalfDb", volumeSlider->value());
  s->setValue("fmRdsSelector", fmRdsSelector->currentText());
  s->setValue("fmChannelSelect", fmChannelSelect->currentText());
  s->setValue("fmDeemphasisSelector", fmDeemphasisSelector->currentText());
  s->setValue("fmStereoPanoramaSlider", fmStereoPanoramaSlider->value());
  s->setValue("fmStereoBalanceSlider", fmStereoBalanceSlider->value());
  s->setValue("fmLFcutoff", fmLFcutoff->currentText());
  s->setValue("logging", logging->currentText());
  s->setValue("streamOutSelector", streamOutSelector->currentText());

  s->setValue("currentFreq", currentFreq);
  s->setValue("min_loop_frequency", minLoopFrequency);
  s->setValue("max_loop_frequency", maxLoopFrequency);

  //	Note that settings for the device used will be restored
  //	on termination of the device handling class
}

//	On start, we ensure that the streams are stopped so
//	that they can be restarted again.
void RadioInterface::setStart()
{
  bool r = false;

  if (runMode == ERunStates::RUNNING) // someone presses while running
  {
    return;
  }

  r = myRig->restartReader();
  //	qDebug ("Starting %d\n", r);
  if (!r)
  {
    QMessageBox::warning(this, tr("sdr"), tr("Opening  input stream failed\n"));
    return;
  }

  if (myFMprocessor == nullptr)
  {
    make_newProcessor();
  }
  myFMprocessor->start();
  our_audioSink->restart();

  //	and finally: recall that starting overrules pausing
  pauseButton->setText(QString("Pause"));

  set_squelchMode("NSQ"); // toggle sequelch on
  set_squelchMode("SQ OFF"); // toggle sequelch off (TODO: make this nicer)

  // populate FM decoder combo box
  {
    // disconnect GUI link temporary as filling the GUI-list will trigger the signal
    disconnect(fmDecoder, qOverload<int>(&QComboBox::currentIndexChanged), this, &RadioInterface::setfmDecoder);

    for (const auto & dc : myFMprocessor->listNameofDecoder())
    {
      fmDecoder->addItem(dc);
    }

    const QString h = fmSettings->value("fmDecoder", "PLL Decoder").toString();
    const int k = fmDecoder->findText(h);

    if (k != -1)
    {
      fmDecoder->setCurrentIndex(k);
      setfmDecoder(k);
    }
    connect(fmDecoder, qOverload<int>(&QComboBox::currentIndexChanged), this, &RadioInterface::setfmDecoder);
  }

  connect(cbAutoMono, &QCheckBox::clicked, this, [this](bool isChecked){ myFMprocessor->setAutoMonoMode(isChecked); });
  connect(cbDCRemove, &QCheckBox::clicked, this, [this](bool isChecked){ myFMprocessor->setDCRemove(isChecked); });
  connect(volumeSlider, &QSlider::valueChanged, this, &RadioInterface::setAudioGainSlider);

  volumeSlider->setValue(fmSettings->value("volumeHalfDb", -12).toInt());

  runMode = ERunStates::RUNNING;
}
//
//	always tricky to kill tasks
void RadioInterface::TerminateProcess()
{
  runMode = ERunStates::STOPPING;

  if (sourceDumping && myFMprocessor != nullptr)
  {
    myFMprocessor->stopDumping();
    sf_close(dumpfilePointer);
  }

  if (myFMprocessor != nullptr)
  {
    delete myFMprocessor;
  }

  if (audioDumping)
  {
    our_audioSink->stopDumping();
    sf_close(audiofilePointer);
  }

  stopIncrementing();
  saveTable();
  dumpControlState(fmSettings);
  fmSettings->sync();

  //	It is pretty important that no one is attempting to
  //	set things within the FMprocessor when it is
  //	being deleted
  myRig->stopReader();
  //	setDevice (QString ("dummy"));	// will select a virtualinput
  accept();

  qDebug() << "Termination started";

  delete myRig;
  delete mykeyPad;

  delete_station_list();
}

void RadioInterface::abortSystem(int d)
{
  qDebug("aborting for reason %d\n", d);
  accept();
}
//

void RadioInterface::stopDumping()
{
  if (myFMprocessor == nullptr)
  {
    return;
  }
  if (sourceDumping)
  {
    myFMprocessor->stopDumping();
    sf_close(dumpfilePointer);
    sourceDumping = false;
    dumpButton->setText(TXT_DUMP_INPUT_STREAM);
  }

  if (audioDumping)
  {
    our_audioSink->stopDumping();
    sf_close(audiofilePointer);
    audioDumping = false;
    audioDump->setText(TXT_DUMP_AUDIO);
  }
}
//	The following signals originate from the Winrad Extio interface
//
//	Note: the extio interface provides two signals
//	one ExtLO signals that the external LO is set
//	to a different value,
//	the other one, ExtFreq, requests the client program
//	to adapt its (local) tuning settings to a new frequency
void RadioInterface::set_ExtFrequency(int f)
{
  (void)f;
  int32_t vfo = myRig->getVFOFrequency();

  currentFreq = vfo + inputRate / 4;
  LOFrequency = inputRate / 4;
  Display(currentFreq);
  if (myFMprocessor != nullptr)
  {
    myFMprocessor->set_localOscillator(LOFrequency);
    myFMprocessor->resetRds();
  }
}
//
//	From our perspective, the external device only provides us
//	with a vfo
void RadioInterface::set_ExtLO(int f)
{
  set_ExtFrequency(f);
}

void RadioInterface::set_lockLO()
{
  //	fprintf (stderr, "ExtioLock is true\n");
  ExtioLock = true;
}

void RadioInterface::set_unlockLO()
{
  //	fprintf (stderr, "ExtioLock is false\n");
  ExtioLock = false;
}

void RadioInterface::set_stopHW()
{
  myRig->stopReader();
}

void RadioInterface::set_startHW()
{
  if (runMode == ERunStates::RUNNING) // looks strange, but seems right
  {
    myRig->restartReader();
  }
}
//
//	This is a difficult one, everything should go down first
//	and then restart with the new samplerate
void RadioInterface::set_changeRate(int r)
{
  if (r == inputRate)
  {
    return;
  }
  fprintf(stderr, "request for changerate\n");
  myRig->stopReader();
  if (myFMprocessor != nullptr)
  {
    myFMprocessor->stop();
    delete myFMprocessor;
    myFMprocessor = nullptr;
  }

  runMode = ERunStates::IDLE;
  //
  //	Now we need to rebuild the prerequisites for the "new" processor
  inputRate = r;
  if (inputRate < Khz(176)) // rather arbitrarily
  {
    QMessageBox::warning(this, tr("sdr"), tr("Sorry, rate low\n"));
    delete myRig;
    myRig     = new deviceHandler();
    inputRate = myRig->getRate();
  }
  //
  //	compute the new fmRate
  fmRate = mapRates(inputRate);
  //	ask the new for the frequency
  currentFreq = myRig->getVFOFrequency() + fmRate / 4;
  //	and show everything
  Display(currentFreq);
  lcd_fmRate->display((int)this->fmRate);
  lcd_inputRate->display((int)this->inputRate);
  lcd_OutputRate->display((int)this->audioRate);
  //
  //	The device is still the same, so now we wait for a start
}
//
//	@brief setDevice is called upon pressing the device button
//	@params: the name (string) on the button

void RadioInterface::setDevice(const QString &s)
{
  QString file;
  bool    success;

  //	The fm processor is a client of the rig, so the
  //	fm processor has to go first
  if (myRig != nullptr)
  {
    myRig->stopReader();
  }
  myRig = nullptr;
  if (myFMprocessor != nullptr)
  {
    myFMprocessor->stop();
    delete myFMprocessor;
    myFMprocessor = nullptr;
  }

  runMode   = ERunStates::IDLE;
  ExtioLock = false;
  delete myRig;
  success = true; // default for now
#ifdef HAVE_SDRPLAY
  if (s == "sdrplay")
  {
    try {
      myRig = new sdrplayHandler(fmSettings);
    } catch (int e) {
      success = false;
    }
  }
  else
#endif
#ifdef HAVE_SDRPLAY_V3
  if (s == "sdrplay-v3")
  {
    try {
      myRig = new sdrplayHandler_v3(fmSettings);
    } catch (int e) {
      success = false;
    }
  }
  else
#endif
#ifdef HAVE_AIRSPY
  if (s == "airspy")
  {
    try {
      myRig = new airspyHandler(fmSettings, true, &success);
    } catch (int e) {
      success = false;
    }
  }
  else
#endif
#ifdef HAVE_HACKRF
  if (s == "hackrf")
  {
    success = true;
    try {
      myRig = new hackrfHandler(fmSettings);
    } catch (int e) {
      success = false;
    }
  }
  else
#endif
#ifdef HAVE_LIME
  if (s == "lime")
  {
    success = true;
    try {
      myRig = new limeHandler(fmSettings);
    } catch (int e) {
      success = false;
    }
  }
  else
#endif
#ifdef HAVE_COLIBRI
  if (s == "colibri")
  {
    success = true;
    try {
      myRig = new colibriHandler(fmSettings);
    } catch (int e) {
      success = false;
    }
  }
  else
#endif
#ifdef HAVE_PLUTO
  if (s == "pluto")
  {
    success = true;
    try {
      myRig = new plutoHandler(fmSettings);
    } catch (int e) {
      success = false;
    }
  }
  else
#endif
#ifdef HAVE_ELAD_S1
  if (s == "elad-s1")
  {
    myRig = new eladHandler(fmSettings, true, &success);
  }
  else
#endif
#ifdef HAVE_DABSTICK
  if (s == "dabstick")
  {
    success = true;
    try {
      myRig = new rtlsdrHandler(fmSettings, true);
    } catch (int e) {
      success = false;
    }
  }
  else
#endif
#ifdef HAVE_EXTIO
  if (s == "extio")
  {
    myRig = new ExtioHandler(fmSettings, theSelector, &success);
  }
  else
#endif
#ifdef HAVE_PMSDR
  if (s == "pmsdr")
  {
    myRig = new pmsdrHandler(fmSettings, &success);
  }
  else
#endif
  if (s == "filereader")
  {
    success = true;
    try {
      myRig = new fileReader(fmSettings);
    } catch (int e) {
      success = false;
    }
  }
  else
  {
    myRig = new deviceHandler();
  }
  if (!success)
  {
    QMessageBox::warning(this, tr("sdr"), tr("loading device failed"));
    if (myRig == nullptr)
    {
      myRig = new deviceHandler();
    }
    resetSelector();
    return;
  }

  inputRate = myRig->getRate();
  if (inputRate < Khz(176)) // rather arbitrarily
  {
    QMessageBox::warning(this, tr("sdr"), tr("Sorry, rate low\n"));
    delete myRig;
    myRig     = new deviceHandler();
    inputRate = myRig->getRate();
  }
  //
  //	ask the new rig for the frequency
  fmRate      = mapRates(inputRate);

  currentFreq = myRig->defaultFrequency() + fmRate / 4;
  currentFreq = fmSettings->value("currentFreq", currentFreq).toInt();
  Display(currentFreq);

  lcd_fmRate->display((int)this->fmRate);
  lcd_inputRate->display((int)this->inputRate);
  lcd_OutputRate->display((int)this->audioRate);
  connect(myRig, SIGNAL(set_changeRate(int)), this, SLOT(set_changeRate(int)));

#ifdef __MINGW32__
  //	communication from the dll to the main program is through signals
  if (s == "extio")
  {
    //	and for the extio:
    //	The following signals originate from the Winrad Extio interface
    connect(myRig, SIGNAL(set_ExtFrequency(int)), this, SLOT(set_ExtFrequency(int)));
    connect(myRig, SIGNAL(set_ExtLO(int)), this, SLOT(set_ExtLO(int)));
    connect(myRig, SIGNAL(set_lockLO()), this, SLOT(set_lockLO()));
    connect(myRig, SIGNAL(set_unlockLO()), this, SLOT(set_unlockLO()));
    connect(myRig, SIGNAL(set_stopHW()), this, SLOT(set_stopHW()));
    connect(myRig, SIGNAL(set_startHW()), this, SLOT(set_startHW()));
  }
#endif
  myRig->setVFOFrequency(currentFreq);
  setStart();
}
//
//	Just for convenience packed as a function
void RadioInterface::make_newProcessor()
{
  myFMprocessor = new fmProcessor(myRig, this, our_audioSink, inputRate, fmRate,
                                  workingRate, audioRate, displaySize,
                                  spectrumSize, averageCount, repeatRate,
                                  hfBuffer, lfBuffer, filterDepth, thresHold);

  lcd_fmRate->display((int)this->fmRate);
  lcd_inputRate->display((int)this->inputRate);
  lcd_OutputRate->display((int)this->audioRate);
  hfScope->setBitDepth(myRig->bitDepth());

  setAttenuation(1);
  setfmBandwidth(fmFilterSelect->currentText());
  setfmBandwidth(fmFilterDegree->value());
  setfmMode(fmMode->currentText());
  setfmRdsSelector(fmRdsSelector->currentText());
  //setfmDecoder(fmDecoder->currentIndex()); // no valid entries yet
  setfmChannelSelector(fmChannelSelect->currentText());
  setfmDeemphasis(fmDeemphasisSelector->currentText());
  setfmStereoPanoramaSlider(fmStereoPanoramaSlider->value());
  setfmStereoBalanceSlider(fmStereoBalanceSlider->value());
  set_squelchValue(squelchSlider->value());
  setfmLFcutoff(fmLFcutoff->currentText());
  setLogging(logging->currentText());
  hfScope->setBitDepth(myRig->bitDepth());
}

void RadioInterface::setInputMode(const QString &s)
{
  if (s == "I and Q")
  {
    inputMode = IandQ;
  }
  else if (s == "Q and I")
  {
    inputMode = QandI;
  }
  else if (s == "I Only")
  {
    inputMode = I_Only;
  }
  else if (s == "Q Only")
  {
    inputMode = Q_Only;
  }
  else
  {
    inputMode = IandQ;
  }

  if (myFMprocessor != nullptr)
  {
    myFMprocessor->setInputMode(inputMode);
  }
}

void RadioInterface::setfmChannelSelector(const QString &s)
{
  if (s == "L | R")
  {
    channelSelector = fmProcessor::S_STEREO;
  }
  else if (s == "R | L")
  {
    channelSelector = fmProcessor::S_STEREO_SWAPPED;
  }
  else if (s == "L | L")
  {
    channelSelector = fmProcessor::S_LEFT;
  }
  else if (s == "R | R")
  {
    channelSelector = fmProcessor::S_RIGHT;
  }
  else if (s == "M | M")
  {
    channelSelector = fmProcessor::S_LEFTplusRIGHT;
  }
  else if (s == "S | S")
  {
    channelSelector = fmProcessor::S_LEFTminusRIGHT;
  }
  else
  {
    channelSelector = fmProcessor::S_STEREO;
  }

  if (myFMprocessor != nullptr)
  {
    myFMprocessor->setSoundMode(channelSelector);
  }
}

void RadioInterface::setAttenuation(int n)
{
  int16_t f = IQBalanceDisplay->value();
  int16_t bl, br;

  bl                 = 100 - 2 * f;
  br                 = 100 + 2 * f;
  currAttSliderValue = 2 * n;
  attValueL          = currAttSliderValue * (float)bl / 100;
  attValueR          = currAttSliderValue * (float)br / 100;
  if (myFMprocessor != nullptr)
  {
    myFMprocessor->setAttenuation(attValueL, attValueR);
  }
}
/*
 *  the balance slider runs between -30 .. 30
 */
void RadioInterface::setIQBalance(int n)
{
  int16_t bl, br;

  IQBalanceDisplay->display(n);
  bl        = 100 - 2 * n;
  br        = 100 + 2 * n;
  attValueL = currAttSliderValue * (float)bl / 100;
  attValueR = currAttSliderValue * (float)br / 100;
  if (myFMprocessor != nullptr)
  {
    myFMprocessor->setAttenuation(attValueL, attValueR);
  }
}
//
//	Increment frequency: with amount N, depending
//	on the mode of operation
//
int32_t RadioInterface::mapIncrement(int32_t n)
{
  return Khz(n);
}
//
//	IncrementFrequency is called from the handlers
//	for the autoincrement, the increment and the
//	++, + etc knobs.
//	Deviations within 15K of the VFO are handled with an
//	offset "onscreen"
void RadioInterface::IncrementFrequency(int32_t n)
{
  int32_t vfoFreq;

  stopIncrementing();
  vfoFreq     = myRig->getVFOFrequency();
  setTuner(vfoFreq + LOFrequency + n);
}
//	AdjustFrequency is called whenever someone clicks
//	with the button on the screen. The amount
//	has to be multiplied with 1000
void RadioInterface::AdjustFrequency(int n)
{
  IncrementFrequency(Khz(n));
}
//
//	Whenever the mousewheel is changed, the frequency
//	is adapted
void RadioInterface::wheelEvent(QWheelEvent *e)
{
  if (e->delta() > 0)
  {
    IncrementFrequency(KHz(1));
  }
  else
  {
    IncrementFrequency(-KHz(1));
  }
}

//
//	The generic setTuner.
//
void RadioInterface::setTuner(int32_t n)
{
  //	if ((n < Mhz (60)) || (n > Mhz (420)))
  //	   return Khz (94700);
  //	as long as the requested frequency fits within the current
  //	range - i.e. the full width required for fm demodulation fits -
  //	the vfo remains the same, while the LO is adapted.
  int32_t vfo = myRig->getVFOFrequency();
  const int32_t vfoLast = vfo;

  if (ExtioLock)
  {
    currentFreq = vfo;
    return;
  }

  QTimer::singleShot(2000, this, [this](){ mSuppressTransient = false; } );
  mSuppressTransient = true;

  if (abs(n - vfo) > inputRate / 2 - fmRate / 2)
  {
    myRig->setVFOFrequency(n);
    vfo = myRig->getVFOFrequency();

    if (myFMprocessor != nullptr)
    {
      myFMprocessor->triggerDrawNewHfSpectrum(); // as VFO frequency changed, draw new HF spectrum immediately without averaging
    }
  }

  const int32_t loFrequencyLast = LOFrequency;
  LOFrequency = n - vfo;

  //	constrain LOFrequency, since that is used as an index in a table
  if (LOFrequency > inputRate / 2)
  {
    LOFrequency = inputRate / 2;
  }
  else if (LOFrequency < -inputRate / 2)
  {
    LOFrequency = -inputRate / 2;
  }

  if (myFMprocessor != nullptr)
  {
    myFMprocessor->set_localOscillator(LOFrequency);

    // redraw LF frequency only with bigger frequency steps, AFC will trigger this too, else
    if (vfo != vfoLast || std::abs(LOFrequency - loFrequencyLast) >= KHz(100))
    {
      myFMprocessor->resetRds();
      myFMprocessor->triggerDrawNewLfSpectrum(); // any change in frequency, draw new LF spectrum immediately without averaging
    }
  }
  Display(vfo + LOFrequency);
  currentFreq = vfo + LOFrequency;
}
//
//===== code for auto increment/decrement
//	lots of code for something simple,

static inline bool frequencyInBounds(int32_t f, int32_t l, int32_t u)
{
  return l <= f && f <= u;
}

int32_t RadioInterface::IncrementInterval(int16_t index)
{
  if (index < 0)
  {
    index = -index;
  }

  if (index == 0)
  {
    index = 1;
  }
  if (index >= delayTableSize)
  {
    index = delayTableSize;
  }

  return 1000 * delayTable[index - 1];
}

void RadioInterface::set_incrementFlag(int16_t incr)
{
  char temp[128];

  if (incr == 0)
  {
    incrementingFlag->setStyleSheet("QLabel {background-color:blue}");
    incrementingFlag->setText(" ");
    return;
  }
  if (incr < 0)
  {
    sprintf(temp, " << %d", IncrementInterval(incr) / 1000);
  }
  else
  {
    sprintf(temp, "%d >> ", IncrementInterval(incr) / 1000);
  }
  incrementingFlag->setStyleSheet("QLabel {background-color:green}");
  incrementingFlag->setText(temp);
}
//
void RadioInterface::autoIncrement_timeout()
{
  const int32_t low  = KHz(minLoopFrequency);
  const int32_t high = KHz(maxLoopFrequency);
  int32_t amount = fmIncrement;

  if (IncrementIndex < 0)
  {
    amount = -amount;
  }

  int32_t frequency = currentFreq + KHz(amount);

  if ((IncrementIndex < 0) && !frequencyInBounds(frequency, low, high))
  {
    frequency = high;
  }

  if ((IncrementIndex > 0) && !frequencyInBounds(frequency, low, high))
  {
    frequency = low;
  }

  setTuner(frequency);

  autoIncrementTimer->start(IncrementInterval(IncrementIndex));

  if (myFMprocessor != nullptr)
  {
    myFMprocessor->startScanning();
  }
}

void RadioInterface::scanresult()
{
  stopIncrementing();
}
//
//	stopIncrementing is called from various places to
//	just interrupt the autoincrementing
void RadioInterface::stopIncrementing()
{
  set_incrementFlag(0);

  if (autoIncrementTimer->isActive())
  {
    autoIncrementTimer->stop();
  }

  IncrementIndex = 0;
  if (myFMprocessor != nullptr)
  {
    myFMprocessor->stopScanning();
  }
}

void RadioInterface::autoIncrementButton()
{
  if (autoIncrementTimer->isActive())
  {
    autoIncrementTimer->stop();
  }

  if (++IncrementIndex > delayTableSize)
  {
    IncrementIndex = delayTableSize;
  }

  if (IncrementIndex == 0)
  {
    set_incrementFlag(0);
    return;
  }
  //
  autoIncrementTimer->start(IncrementInterval(IncrementIndex));
  set_incrementFlag(IncrementIndex);
}

void RadioInterface::autoDecrementButton()
{
  if (autoIncrementTimer->isActive())
  {
    autoIncrementTimer->stop();
  }

  if (--IncrementIndex < -delayTableSize)
  {
    IncrementIndex = -delayTableSize;
  }

  if (IncrementIndex == 0)
  {
    set_incrementFlag(0);
    return;
  }
  //
  autoIncrementTimer->start(IncrementInterval(IncrementIndex));
  set_incrementFlag(IncrementIndex);
}

void RadioInterface::set_fm_increment(int v)
{
  fmIncrement = v; // in Khz
}

//
//	min and max frequencies are specified in Mhz
void RadioInterface::set_minimum(int f)
{
  minLoopFrequency = Khz(f);
}

void RadioInterface::set_maximum(int f)
{
  maxLoopFrequency = Khz(f);
}

void RadioInterface::IncrementButton()
{
  stopIncrementing();
  setTuner(currentFreq + Khz(fmIncrement));
}

void RadioInterface::DecrementButton()
{
  stopIncrementing();
  setTuner(currentFreq - Khz(fmIncrement));
}
//
void RadioInterface::updateTimeDisplay()
{
  QDateTime currentTime = QDateTime::currentDateTime();

  timeDisplay->setText(currentTime.toString(QString("dd.MM.yy hh:mm:ss")));
}

void RadioInterface::set_dumping()
{
  SF_INFO *sf_info = (SF_INFO *)alloca(sizeof(SF_INFO));

  if (myFMprocessor == nullptr)
  {
    return;
  }

  if (sourceDumping)
  {
    myFMprocessor->stopDumping();
    sf_close(dumpfilePointer);
    sourceDumping = false;
    dumpButton->setText(TXT_DUMP_INPUT_STREAM);
    return;
  }

  QString file = QFileDialog::getSaveFileName(
    this, tr("open file ..."), QDir::homePath(), tr("Sound (*.wav)"));

  file = QDir::toNativeSeparators(file);
  if (!file.endsWith(".wav", Qt::CaseInsensitive))
  {
    file.append(".wav");
  }
  sf_info->samplerate = inputRate;
  sf_info->channels   = 2;
  sf_info->format     = SF_FORMAT_WAV | SF_FORMAT_PCM_24;

  dumpfilePointer = sf_open(file.toLatin1().data(), SFM_WRITE, sf_info);
  if (dumpfilePointer == nullptr)
  {
    qDebug() << "Cannot open " << file.toLatin1().data();
    return;
  }

  dumpButton->setText(TXT_WRITING);
  sourceDumping = true;
  myFMprocessor->startDumping(dumpfilePointer);
}

void RadioInterface::set_audioDump()
{
  SF_INFO *sf_info = (SF_INFO *)alloca(sizeof(SF_INFO));

  if (audioDumping)
  {
    our_audioSink->stopDumping();
    sf_close(audiofilePointer);
    audioDumping = false;
    audioDump->setText(TXT_DUMP_AUDIO);
    return;
  }

  QString file = QFileDialog::getSaveFileName(
    this, tr("open file .."), QDir::homePath(), tr("Sound (*.wav)"));

  file = QDir::toNativeSeparators(file);
  if (!file.endsWith(".wav", Qt::CaseInsensitive))
  {
    file.append(".wav");
  }

  sf_info->samplerate = this->audioRate;
  sf_info->channels   = 2;
  sf_info->format     = SF_FORMAT_WAV | SF_FORMAT_PCM_24;

  audiofilePointer = sf_open(file.toLatin1().data(), SFM_WRITE, sf_info);
  if (audiofilePointer == nullptr)
  {
    qDebug() << "Cannot open " << file.toLatin1().data();
    return;
  }

  audioDump->setText(TXT_WRITING);
  audioDumping = true;
  our_audioSink->startDumping(audiofilePointer);
}

void RadioInterface::localConnects()
{
  connect(pauseButton, SIGNAL(clicked()), this, SLOT(clickPause()));
  connect(streamOutSelector, SIGNAL(activated(int)), this, SLOT(setStreamOutSelector(int)));
  connect(deviceSelector, SIGNAL(activated(const QString&)), this, SLOT(setDevice(const QString&)));
  connect(dumpButton, SIGNAL(clicked()), this, SLOT(set_dumping()));
  connect(audioDump, SIGNAL(clicked()), this, SLOT(set_audioDump()));

  connect(squelchSelector, SIGNAL(activated(const QString&)), this, SLOT(set_squelchMode(const QString&)));
  connect(squelchSlider, SIGNAL(valueChanged(int)), this, SLOT(set_squelchValue(int)));

  connect(IQbalanceSlider, SIGNAL(valueChanged(int)), this, SLOT(setIQBalance(int)));

  connect(fc_plus, SIGNAL(clicked()), this, SLOT(autoIncrementButton()));
  connect(fc_minus, SIGNAL(clicked()), this, SLOT(autoDecrementButton()));
  connect(f_plus, SIGNAL(clicked()), this, SLOT(IncrementButton()));
  connect(f_minus, SIGNAL(clicked()), this, SLOT(DecrementButton()));

  connect(fmChannelSelect, SIGNAL(activated(const QString&)), this, SLOT(setfmChannelSelector(const QString&)));
  connect(logging, SIGNAL(activated(const QString&)), this, SLOT(setLogging(const QString&)));
  connect(logSaving, SIGNAL(clicked()), this, SLOT(setLogsaving()));
  connect(fmFilterSelect, SIGNAL(activated(const QString&)), this, SLOT(setfmBandwidth(const QString&)));
  connect(fmFilterDegree, SIGNAL(valueChanged(int)), this, SLOT(setfmBandwidth(int)));
  connect(fmMode, SIGNAL(activated(const QString&)), this, SLOT(setfmMode(const QString&)));
  connect(fmRdsSelector, SIGNAL(activated(const QString&)), this, SLOT(setfmRdsSelector(const QString&)));
  connect(fmDecoder, qOverload<int>(&QComboBox::currentIndexChanged), this, &RadioInterface::setfmDecoder);
  connect(fmStereoPanoramaSlider, SIGNAL(valueChanged(int)), this, SLOT(setfmStereoPanoramaSlider(int)));
  connect(fmStereoBalanceSlider, SIGNAL(valueChanged(int)), this, SLOT(setfmStereoBalanceSlider(int)));
  connect(fmDeemphasisSelector, SIGNAL(activated(const QString&)), this, SLOT(setfmDeemphasis(const QString&)));
  connect(fmLFcutoff, SIGNAL(activated(const QString&)), this, SLOT(setfmLFcutoff(const QString&)));
  connect(plotSelector, qOverload<const QString &>(&QComboBox::activated), this, &RadioInterface::setLfPlotType);
  connect(plotFactor, qOverload<const QString &>(&QComboBox::activated), this, &RadioInterface::setLfPlotZoomFactor);
}

void RadioInterface::setfmStereoPanoramaSlider(int n)
{
  if (myFMprocessor != nullptr)
  {
    myFMprocessor->setStereoPanorama(n);
  }
}

void RadioInterface::setfmStereoBalanceSlider(int n)
{
  if (myFMprocessor != nullptr)
  {
    myFMprocessor->setSoundBalance(n);
    balanceDisplay->display(n);
  }
}

void RadioInterface::setAudioGainSlider(int n)
{
  if (myFMprocessor != nullptr)
  {
    const float gainDB = (n < -59 ? -99.9f : n / 2.0f);
    myFMprocessor->setVolume(gainDB);
    //audioGainDisplay->display(gainDB);
    audioGainDisplay->display(QString("%1").arg(gainDB, 0, 'f', 1)); // allow one fix digit after decimal point
  }
}

//	Deemphasis	= 50 usec (3183 Hz, Europe)
//	Deemphasis	= 75 usec (2122 Hz US)
void RadioInterface::setfmDeemphasis(const QString &s)
{
  if (myFMprocessor == nullptr)
  {
    return;
  }

  if (s == "Off (AM)")
  {
    myFMprocessor->setDeemphasis(1);
  }
  else
  {
    myFMprocessor->setDeemphasis(std::stol(s.toStdString())); // toInt will not work with text after the number
  }
}

void RadioInterface::setCRCErrors(int n)
{
  crcErrors->display(n);
}

void RadioInterface::setSyncErrors(int n)
{
  syncErrors->display(n);
}

void RadioInterface::setbitErrorRate(double v)
{
  bitErrorRate->display(v);
}

void RadioInterface::setGroup(int n)
{
  (void)n;
  //	rdsGroupDisplay	-> display (n);
}

void RadioInterface::setPTYCode(int n, const QString & iPtyText)
{
  (void)n;
  pty_text->setText(iPtyText);
}

void RadioInterface::setAFDisplay(int n1, int n2)
{
  if (n1 == 0 && n2 == 0) // reset constellation (n1 and n2 are set together to zero)
  {
    //rdsAF1Display->display(n1);
    rdsAF2Display->display(n2);
  }
//  else if (n1 > 0)
//  {
//    rdsAF1Display->display(n1);
//  }
  else //if (n2 > 0)
  {
    rdsAF2Display->display(n2 > 0 ? n2 : n1);
  }
}

void RadioInterface::setPiCode(int n)
{
  int32_t t = currentFreq;

  if ((frequencyforPICode != t) || (n != 0))
  {
    currentPIcode      = n;
    frequencyforPICode = t;
  }

  rdsPiDisplay->display(n);
}

//void RadioInterface::clearStationLabel()
//{
//  StationLabel = QString("");
//  stationLabelTextBox->setText(StationLabel);
//}

void RadioInterface::setStationLabel(const QString &s)
{
  stationLabelTextBox->setText(s);
}

void RadioInterface::setMusicSpeechFlag(int n)
{
  if (n != 0)
  {
    speechLabel->setText(QString("Music"));
  }
  else
  {
    speechLabel->setText(QString("Speech"));
  }
}

void RadioInterface::clearMusicSpeechFlag()
{
  speechLabel->setText(QString(""));
}

//void RadioInterface::clearRadioText()
//{
//  RadioText = QString("");
//  radioTextBox->setText(RadioText);
//}

void RadioInterface::setRadioText(const QString &s)
{
  radioTextBox->setText(s);
}

void RadioInterface::setRDSisSynchronized(bool syn)
{
  if (!syn)
  {
    rdsSyncLabel->setStyleSheet("QLabel {background-color:red}");
  }
  else
  {
    rdsSyncLabel->setStyleSheet("QLabel {background-color:green}");
  }
}

void RadioInterface::setfmMode(const QString &s)
{
  if (myFMprocessor == nullptr)
  {
    return;
  }

  if (s == "Stereo")
  {
    myFMprocessor->setfmMode(fmProcessor::FM_Mode::Stereo);
    fmStereoPanoramaSlider->setEnabled(false);
    //fmChannelSelect->setEnabled(true);
  }
  else if (s == "Stereo (Pano)")
  {
    myFMprocessor->setfmMode(fmProcessor::FM_Mode::StereoPano);
    fmStereoPanoramaSlider->setEnabled(true);
    //fmChannelSelect->setEnabled(true);
  }
  else if (s == "Mono")
  {
    myFMprocessor->setfmMode(fmProcessor::FM_Mode::Mono);
    fmStereoPanoramaSlider->setEnabled(false);
    //fmChannelSelect->setEnabled(false);
  }
  else
  {
    Q_ASSERT(0);
  }
}

void RadioInterface::setLfPlotType(const QString &s)
{
  if (myFMprocessor == nullptr)
  {
    return;
  }

  if      (s == "OFF")               myFMprocessor->setLfPlotType(fmProcessor::ELfPlot::OFF);
  else if (s == "IF Filtered")       myFMprocessor->setLfPlotType(fmProcessor::ELfPlot::IF_FILTERED);
  else if (s == "Demodulator")       myFMprocessor->setLfPlotType(fmProcessor::ELfPlot::DEMODULATOR);
  else if (s == "AF SUM")            myFMprocessor->setLfPlotType(fmProcessor::ELfPlot::AF_SUM);
  else if (s == "AF DIFF")           myFMprocessor->setLfPlotType(fmProcessor::ELfPlot::AF_DIFF);
  else if (s == "AF MONO Filtered")  myFMprocessor->setLfPlotType(fmProcessor::ELfPlot::AF_MONO_FILTERED);
  else if (s == "AF LEFT Filtered")  myFMprocessor->setLfPlotType(fmProcessor::ELfPlot::AF_LEFT_FILTERED);
  else if (s == "AF RIGHT Filtered") myFMprocessor->setLfPlotType(fmProcessor::ELfPlot::AF_RIGHT_FILTERED);
  else if (s == "RDS")               myFMprocessor->setLfPlotType(fmProcessor::ELfPlot::RDS);
  else
  {
    Q_ASSERT(0);
  }

  myFMprocessor->triggerDrawNewLfSpectrum(); // resets the average filter
}

void RadioInterface::setLfPlotZoomFactor(const QString &s)
{
  if (myFMprocessor == nullptr)
  {
    return;
  }

  myFMprocessor->setLfPlotZoomFactor(std::stol(s.toStdString()));
  myFMprocessor->triggerDrawNewLfSpectrum(); // resets the average filter
}

void RadioInterface::setfmRdsSelector(const QString &s)
{
  if (myFMprocessor == nullptr)
  {
    return;
  }

  if      (s == "RDS-1") rdsModus = rdsDecoder::ERdsMode::RDS1;
  else if (s == "RDS-2") rdsModus = rdsDecoder::ERdsMode::RDS2;
  else if (s == "RDS-3") rdsModus = rdsDecoder::ERdsMode::RDS3;
  else                   rdsModus = rdsDecoder::ERdsMode::NO_RDS;

  myFMprocessor->setfmRdsSelector(rdsModus);
}

void RadioInterface::setfmDecoder(const int decoder)
{
  if (myFMprocessor == nullptr || decoder < 0)
  {
    return;
  }

  myFMprocessor->setFMdecoder(decoder);
  //decoderDisplay->setText(QString(myFMprocessor->nameofDecoder()));
  qInfo("Using FM decoder: %s", myFMprocessor->nameofDecoder());
}

void RadioInterface::setfmLFcutoff(const QString &s)
{
  if (myFMprocessor == nullptr)
  {
    return;
  }

  if (s == "Off")
  {
    myFMprocessor->setLFcutoff(-1);
  }
  else
  {
    //myFMprocessor->setLFcutoff((int32_t)(s.toInt()));
    myFMprocessor->setLFcutoff(std::stol(s.toStdString()));
  }
}

inline int32_t numberofDigits(int32_t f)
{
  if (f < 100000)
  {
    return 6;
  }
  if (f < 100000000)
  {
    return 8;
  }
  if (f < 1000000000)
  {
    return 9;
  }
  return 10;
}

void RadioInterface::Display(int32_t freq)
{
  lcd_Frequency->display((freq + KHz(1)/2) / KHz(1));
}

void RadioInterface::setfmBandwidth(const QString &s)
{
  if (s == "Off")
  {
    fmBandwidth = 0.95 * fmRate;
  }
  else
  {
    fmBandwidth = Khz(std::stol(s.toStdString()));
  }

  if (myFMprocessor != nullptr)
  {
    myFMprocessor->setBandwidth(fmBandwidth);
  }
}

void RadioInterface::setfmBandwidth(int32_t b)
{
  if (myFMprocessor != nullptr)
  {
    myFMprocessor->setBandfilterDegree(b);
  }
}


void RadioInterface::showPeakLevel(const float iPeakLeft, const float iPeakRight)
{
  //qInfo("PeakLeft %f, PeakRight %f", iPeakLeft, iPeakRight);
  thermoPeakLevelLeft->setValue(iPeakLeft);
  thermoPeakLevelRight->setValue(iPeakRight);

  // simple overflow avoidance -> reduce volume slider about -0.5dB (one step)
  if ((iPeakLeft > 0.0f || iPeakRight > 0.0f) && mSuppressTransient == false)
  {
    volumeSlider->setValue(volumeSlider->value() - 1);
  }
}

//
//	This signal will arrive once every "inputRate" samples
void RadioInterface::showStrength(float the_pilotStrength, float the_dcComponent)
{
  static std::array<char, 1024> s{};
  static int teller = 0;

  bool triggerLog = false;

  if (logTime > 0 && ++teller == logTime)
  {
    triggerLog = true;
    teller = 0;
  }

  //if (the_pilotStrength > 2.0)
  float lockStrength;

  if (myFMprocessor->isPilotLocked(lockStrength))
  {
    pll_isLocked->setStyleSheet("QLabel {background-color:green}");
    pll_isLocked->setText("Pilot PLL Locked");
  }
  else
  {
    pll_isLocked->setStyleSheet("QLabel {background-color:red}");
    pll_isLocked->setText("Pilot PLL Unlocked");
  }

  //pilotStrength->display(lockStrength);
  //pilotStrength->display(the_pilotStrength);
  pilotStrength->display(QString("%1").arg(the_pilotStrength, 0, 'f', 1)); // allow one fix digit after decimal point

  //dc_component->display(the_dcComponent);
  dc_component->display(QString("%1").arg(the_dcComponent, 0, 'f', 2)); // allow tow fix digit after decimal point

  static const float w = 1.0f / std::log10(2.0f);
  const float dcVal = (the_dcComponent < 0.0f ? 1 : -1) * w * std::log10(std::abs(the_dcComponent) + 1.0f);
  thermoDcComponent->setValue(dcVal);

  // some kind of AFC
  if (mAfcActive)
  {
    const int32_t afcOffFreq = the_dcComponent * 10000; // the_dcComponent is positive with too little frequency
    mAfcCurrOffFreq = (1 - mAfcAlpha) * mAfcCurrOffFreq + mAfcAlpha * afcOffFreq;

//    constexpr int32_t limitOffFreq = 20000;
//    if      (mAfcCurrOffFreq >  limitOffFreq) mAfcCurrOffFreq =  limitOffFreq;
//    else if (mAfcCurrOffFreq < -limitOffFreq) mAfcCurrOffFreq = -limitOffFreq;

    const float absAfcCurrOffFreq = abs(mAfcCurrOffFreq);

    if      (absAfcCurrOffFreq <  10) { mAfcAlpha = 0.005f; }
    else if (absAfcCurrOffFreq < 100) { mAfcAlpha = 0.050f; }
    else                              { mAfcAlpha = 0.800f; }

    uint32_t newFreq = currentFreq + mAfcCurrOffFreq;

    if (triggerLog)
    {
      fprintf(stderr, "AFC:  DC %f, NewFreq %d = CurrFreq %d + AfcOffFreq %d (unfiltered %d), AFC_Alpha %f\n",
              the_dcComponent, newFreq, currentFreq, mAfcCurrOffFreq, afcOffFreq, mAfcAlpha);
    }

    if (absAfcCurrOffFreq > 3) // avoid re-tunings of HW when only a residual frequency offset remains
    {
      setTuner(newFreq);
    }
  }


  if (triggerLog)
  {
    QDateTime currentTime = QDateTime::currentDateTime();
    sprintf(s.begin(), "%s : Freq = %d,\n PI code = %4X, pilot = %f\n",
            currentTime.toString(QString("dd.MM.yy hh:mm:ss"))
            .toStdString()
            .c_str(),
            currentFreq, currentPIcode, the_pilotStrength);

    fputs(s.cbegin(), stderr);
    //	and into the logfile
    if (logFile != nullptr)
    {
      fputs(s.cbegin(), logFile);
    }
  }
}

void RadioInterface::setLogging(const QString &s)
{
  if (s == "log off")
  {
    logTime = 0;
  }
  else if (s == "log 1 sec")
  {
    logTime = 1;
  }
  else if (s == "log 2 sec")
  {
    logTime = 2;
  }
  else if (s == "log 3 sec")
  {
    logTime = 3;
  }
  else if (s == "log 4 sec")
  {
    logTime = 4;
  }
  else
  {
    logTime = 5;
  }
}

void RadioInterface::setLogsaving()
{
  if (logFile != nullptr) // just stop it
  {
    fclose(logFile);
    logFile = nullptr;
    logSaving->setText("save");
    return;
  }
  else
  {
    QString file = QFileDialog::getSaveFileName(
      this, tr("open file .."), QDir::homePath(), tr("text (*.txt)"));
    logFile = fopen(file.toLatin1().data(), "w");
    if (logFile == nullptr)
    {
      QMessageBox::warning(this, tr("sdr"), tr("/dev/null is used"));

      //	      logFile	= nullptr;		// remains zero
    }
    else   // logFile != nullptr
    {
      logSaving->setText("halt");
      fprintf(logFile, "\nlogging starting\n\n\n");
    }
  }
}
//
void RadioInterface::clickPause()
{
  if (runMode == ERunStates::IDLE)
  {
    return;
  }

  if (runMode == ERunStates::RUNNING)
  {
    if (autoIncrementTimer->isActive())
    {
      autoIncrementTimer->stop();
    }

    myRig->stopReader();
    our_audioSink->stop();
    pauseButton->setText(QString("Continue"));
    runMode = ERunStates::PAUSED;
  }
  else if (runMode == ERunStates::PAUSED)
  {
    if (IncrementIndex != 0) // restart the incrementtimer if needed
    {
      autoIncrementTimer->start(IncrementInterval(IncrementIndex));
    }
    myRig->restartReader();
    our_audioSink->restart();
    pauseButton->setText(QString("Pause"));
    runMode = ERunStates::RUNNING;
  }
}
//
//	do not forget that ocnt starts with 1, due
//	to Qt list conventions
bool RadioInterface::setupSoundOut(QComboBox *streamOutSelector,
                                   audioSink *our_audioSink, int32_t cardRate,
                                   int16_t *table)
{
  uint16_t ocnt = 1;
  uint16_t i;

  for (i = 0; i < our_audioSink->numberofDevices(); i++)
  {
    const char *so = our_audioSink->outputChannelwithRate(i, cardRate);
    qDebug("Investigating Device %d\n", i);

    if (so != nullptr)
    {
      streamOutSelector->insertItem(ocnt, so, QVariant(i));
      table[ocnt] = i;
      qDebug(" (output):item %d wordt stream %d\n", ocnt, i);
      ocnt++;
    }
  }

  qDebug() << "added items to combobox";
  return ocnt > 1;
}

void RadioInterface::setStreamOutSelector(int idx)
{
  if (idx == 0)
  {
    return;
  }

  outputDevice = outTable[idx];
  if (!our_audioSink->isValidDevice(outputDevice))
  {
    return;
  }

  our_audioSink->stop();
  if (!our_audioSink->selectDevice(outputDevice))
  {
    QMessageBox::warning(this, tr("sdr"),
                         tr("Selecting  output stream failed\n"));
    return;
  }

  qWarning() << "selected output device " << idx << outputDevice;
  our_audioSink->restart();
}

void RadioInterface::set_squelchValue(int n)
{
  if (myFMprocessor != nullptr)
  {
    myFMprocessor->set_squelchValue(n);
  }
}
//
//	For the HF scope, we only do the displaying (and X axis)
//	in the GUI environment. The FM processor prepares "views"
//	and punt these views into a shared buffer. If the buffer is
//	full, a signal is sent.
void RadioInterface::hfBufferLoaded()
{
  double  *X_axis   = (double *)alloca(displaySize * sizeof(double));
  double  *Y_values = (double *)alloca(displaySize * sizeof(double));
  double  temp      = (double)inputRate / 2 / displaySize;
  int16_t i;
  int32_t vfoFrequency;

  vfoFrequency = myRig->getVFOFrequency();

  //	first X axis labels
  for (i = 0; i < displaySize; i++)
  {
    X_axis[i] = ((double)vfoFrequency - (double)(inputRate / 2) +
                 (double)((i) * (double)2 * temp)) /
                ((double)Khz(1));
  }
  //
  //	get the buffer data
  hfBuffer->getDataFromBuffer(Y_values, displaySize);
  hfScope->Display(X_axis, Y_values, spectrumAmplitudeSlider_hf->value(),
                   (vfoFrequency + LOFrequency) / Khz(1));
}

//	For the LF scope, we only do the displaying (and X axis)
//	in the GUI environment. The FM processor prepares "views"
//	and punt these views into a shared buffer. If the buffer is
//	full, a signal is sent.
void RadioInterface::lfBufferLoaded(bool iShowFullSpectrum, int iZoomFactor)
{
  double  *X_axis   = (double *)alloca(displaySize * sizeof(double));
  double  *Y_values = (double *)alloca(displaySize * sizeof(double));
  double  temp      = (double)fmRate / 2 / displaySize;
  int16_t i;

  //	first X axis labels
  if (iShowFullSpectrum)
  {
    for (i = 0; i < displaySize; i++)
    {
      X_axis[i] = (-(fmRate / 2.0) + (2 * i * temp)) / ((double)Khz(1)) / iZoomFactor; // two side spectrum
    }
  }
  else
  {
    for (i = 0; i < displaySize; i++)
    {
      X_axis[i] = (i * temp) / ((double)Khz(1)) / iZoomFactor; // one-side spectrum
    }
  }

  //	get the buffer data
  lfBuffer->getDataFromBuffer(Y_values, displaySize);
  lfScope->Display(X_axis, Y_values, spectrumAmplitudeSlider_lf->value());
}

void RadioInterface::setHFplotterView(int offset)
{
  (void)offset;
  if (hfScope->currentMode() == WATERFALL_MODE)
  {
    hfScope->SelectView(SPECTRUM_MODE);
  }
  else
  {
    hfScope->SelectView(WATERFALL_MODE);
  }
}

void RadioInterface::setup_HFScope()
{
  hfBuffer   = new RingBuffer<double>(2 * displaySize);
  hfScope    = new Scope(hfscope, this->displaySize, this->rasterSize);
  HFviewMode = SPECTRUM_MODE;
  hfScope->SelectView(SPECTRUM_MODE);
  connect(hfScope, SIGNAL(clickedwithLeft(int)), this, SLOT(AdjustFrequency(int)));
  connect(hfScope, SIGNAL(clickedwithRight(int)), this, SLOT(setHFplotterView(int)));
}

void RadioInterface::setup_LFScope()
{
  lfBuffer   = new RingBuffer<double>(2 * displaySize);
  lfScope    = new Scope(lfscope, this->displaySize, this->rasterSize);
  LFviewMode = SPECTRUM_MODE;
  lfScope->SelectView(SPECTRUM_MODE);
}

//
void RadioInterface::set_squelchMode(const QString& s)
{
  if (myFMprocessor == nullptr)
  {
    return;
  }

  fmProcessor::ESqMode sqMode = fmProcessor::ESqMode::OFF;

  if      (s == "SQ OFF") sqMode = fmProcessor::ESqMode::OFF;
  else if (s == "NSQ")    sqMode = fmProcessor::ESqMode::NSQ;
  else if (s == "LSQ")    sqMode = fmProcessor::ESqMode::LSQ;
  else
  {
    Q_ASSERT(0);
  }

  squelchMode = (sqMode != fmProcessor::ESqMode::OFF);
  //squelchButton->setText(squelchMode ? QString("Squelch is ON") : QString("Squelch is OFF"));
  squelchSlider->setEnabled(squelchMode);
  myFMprocessor->set_squelchMode(sqMode);
  setSquelchIsActive(myFMprocessor->getSquelchObj()->getSquelchActive()); // gray out squelch notification or read current state
}

void RadioInterface::setSquelchIsActive(bool active)
{
  if (squelchMode)
  {
    if (active)
    {
      sqlStatusLabel->setStyleSheet("QLabel {background-color:red}");
    }
    else
    {
      sqlStatusLabel->setStyleSheet("QLabel {background-color:green}");
    }
  }
  else
  {
    sqlStatusLabel->setStyleSheet("QLabel {background-color:gray}");
  }
}

//	Just a simple routine to set the sliders and boxes
//	to their previous values
void RadioInterface::restoreGUIsettings(QSettings *s)
{
  QString h;
  int     k;

  k = s->value("afc", mAfcActive).toInt();
  cbAfc->setCheckState(k ? Qt::CheckState::Checked : Qt::CheckState::Unchecked);
  mAfcActive = (k != 0);

  k = s->value("spectrumAmplitudeSlider_hf", spectrumAmplitudeSlider_hf->value()).toInt();
  spectrumAmplitudeSlider_hf->setValue(k);

  k = s->value("spectrumAmplitudeSlider_lf", spectrumAmplitudeSlider_lf->value()).toInt();
  spectrumAmplitudeSlider_lf->setValue(k);

  k = s->value("fmFilterDegree", fmFilterDegree->value()).toInt();
  fmFilterDegree->setValue(k);

  k = s->value("fmStereoPanoramaSlider", 100).toInt();
  fmStereoPanoramaSlider->setValue(k);

  k = s->value("fmStereoBalanceSlider", fmStereoBalanceSlider->value()).toInt();
  fmStereoBalanceSlider->setValue(k);

  k = s->value("squelchSlider", squelchSlider->value()).toInt();
  squelchSlider->setValue(k);

  h = s->value("fmFilterSelect", "165kHz").toString();
  k = fmFilterSelect->findText(h);
  if (k != -1)
  {
    fmFilterSelect->setCurrentIndex(k);
  }

  h = s->value("fmMode", fmMode->currentText()).toString();
  k = fmMode->findText(h);
  if (k != -1)
  {
    fmMode->setCurrentIndex(k);
  }

  h = s->value("fmRdsSelector", fmRdsSelector->currentText()).toString();
  k = fmRdsSelector->findText(h);
  if (k != -1)
  {
    fmRdsSelector->setCurrentIndex(k);
  }

  // the FM decoder combobox is not filled yet
//  h = s->value("fmDecoder", fmDecoder->currentText()).toString();
//  k = fmDecoder->findText(h);
//  if (k != -1)
//  {
//    fmDecoder->setCurrentIndex(k);
//  }

  h = s->value("fmChannelSelect", fmChannelSelect->currentText()).toString();
  k = fmChannelSelect->findText(h);
  if (k != -1)
  {
    fmChannelSelect->setCurrentIndex(k);
  }

  h = s->value("fmDeemphasisSelector", "50us  (EU)").toString();
  k = fmDeemphasisSelector->findText(h);
  if (k != -1)
  {
    fmDeemphasisSelector->setCurrentIndex(k);
  }

  h = s->value("fmLFcutoff", "15000Hz").toString();
  k = fmLFcutoff->findText(h);
  if (k != -1)
  {
    fmLFcutoff->setCurrentIndex(k);
  }
}

//	For different input rates we select different rates for the
//	fm decoding (the fmrate). Decimating from inputRate to fmRate
//	is always integer. Decimating from fmRate to audioRate maybe
//	fractional which costs a lot of power.
int32_t RadioInterface::mapRates(int32_t inputRate)
{
  int32_t res;

  res = inputRate % 256000 == 0 ? 256000 :
        inputRate % 192000 == 0 ? 192000 :
        inputRate < Khz(400) ? inputRate :
        inputRate < Khz(850) ? inputRate / 4 :
        inputRate < Khz(1300) ? inputRate / 6 :
        inputRate < Khz(1900) ? inputRate / 8 :
        inputRate < Khz(3000) ? inputRate / 10 :
        inputRate < Khz(4000) ? inputRate / 15 :
        inputRate < Khz(5000) ? inputRate / 20 :
        inputRate < Khz(6000) ? inputRate / 25 :
        inputRate / 30;
  return res;
}

//	In case selection of a device did not work out for whatever
//	reason, the device selector is reset to "no device"
//	Qt will trigger on the change of value in the deviceSelector
//	which will cause selectdevice to be called again (while we
//	are in the middle, so we first disconnect the selector
//	from the slot. Obviously, after setting the index of
//	the device selector, we connect again

void RadioInterface::resetSelector()
{
  disconnect(deviceSelector, SIGNAL(activated(const QString&)), this, SLOT(setDevice(const QString&)));
  //int k = deviceSelector->findText(QString("no device"));
  int k = deviceSelector->findText(QString("dabstick"));

  if (k != -1) // should not happen
  {
    deviceSelector->setCurrentIndex(k);
  }

  connect(deviceSelector, SIGNAL(activated(const QString&)), this, SLOT(setDevice(const QString&)));
}

void RadioInterface::handle_freqButton()
{
  if (mykeyPad->isVisible())
  {
    mykeyPad->hidePad();
  }
  else
  {
    mykeyPad->showPad();
  }
}

void RadioInterface::newFrequency(int f)
{
  stopIncrementing();
  setTuner(f);
}

void RadioInterface::set_freqSave()
{
  myLine = new QLineEdit();
  myLine->setText(stationLabelTextBox->text().trimmed());
  myLine->show();
  connect(myLine, SIGNAL(returnPressed()), this, SLOT(handle_myLine()));
}

void RadioInterface::handle_myLine()
{
  int32_t freq        = myRig->getVFOFrequency() + LOFrequency;
  QString programName = myLine->text();

  fprintf(stderr, "adding %s %s\n", programName.toLatin1().data(),
          QString::number(freq / Khz(1)).toLatin1().data());
  addRow(programName, QString::number(freq / Khz(1)));
  delete myLine;
  myLine = nullptr;
}

void RadioInterface::reset_afc()
{
  mAfcAlpha = 1.0f;
  mAfcCurrOffFreq = 0;
}

//#include <QCloseEvent>
void RadioInterface::closeEvent(QCloseEvent *event)
{
  //  QMessageBox::StandardButton resultButton =
  //    QMessageBox::question(this, "fmRadio",
  //                          tr("Are you sure?\n"),
  //                          QMessageBox::No | QMessageBox::Yes,
  //                          QMessageBox::Yes);

  //  if (resultButton != QMessageBox::Yes)
  //  {
  //    event->ignore();
  //  }
  //  else
  {
    TerminateProcess();
    event->accept();
  }
}

void RadioInterface::saveTable()
{
  QFile file(mSaveName);

  if (file.open(QIODevice::WriteOnly))
  {
    QDataStream stream(&file);
    int32_t n = mpTableWidget->rowCount();
    int32_t m = mpTableWidget->columnCount();
    stream << n << m;

    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < m; j++)
      {
        mpTableWidget->item(i, j)->write(stream);
      }
    }

    file.close();
  }
}

static constexpr auto STATIONLISTFACTOR = 0.65; // workaround until better solution is found

void RadioInterface::loadTable()
{
  QFile file(mSaveName);

  if (file.open(QIODevice::ReadOnly))
  {
    QDataStream stream(&file);
    int32_t     n, m;
    stream >> n >> m;
    mpTableWidget->setRowCount(n);
    mpTableWidget->setColumnCount(m);

    for (int i = 0; i < n; i++)
    {
      for (int j = 0; j < m; j++)
      {
        QTableWidgetItem *item = new QTableWidgetItem;
        item->read(stream);
        item->setTextAlignment((j == 0 ? Qt::AlignLeft : Qt::AlignRight) | Qt::AlignVCenter);
        mpTableWidget->setItem(i, j, item);
      }
    }

    file.close();
  }

  mpTableWidget->resizeColumnsToContents();
  scrollStationList->setFixedWidth(mpTableWidget->sizeHint().width() * STATIONLISTFACTOR); // TODO: how to do correct resizing with this factor?
}

void RadioInterface::addRow(const QString &name, const QString &freq)
{
  const int16_t row = mpTableWidget->rowCount();

  mpTableWidget->insertRow(row);

  QTableWidgetItem * const item0 = new QTableWidgetItem;
  item0->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  item0->setText(name);
  mpTableWidget->setItem(row, 0, item0);

  QTableWidgetItem *const item1 = new QTableWidgetItem;
  item1->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
  item1->setText(freq);
  mpTableWidget->setItem(row, 1, item1);

  mpTableWidget->resizeColumnsToContents();
  scrollStationList->setFixedWidth(mpTableWidget->sizeHint().width() * STATIONLISTFACTOR); // TODO: how to do correct resizing with this factor?
}

void RadioInterface::removeRow(int row, int column)
{
  (void)column;
  const QTableWidgetItem * const itemStation = mpTableWidget->item(row, 0);
  const QTableWidgetItem * const itemFreq = mpTableWidget->item(row, 1);

  const QString deletionText(tr("Delete station '") + itemStation->text() + tr("' from station list?\n (Frequency: ") + itemFreq->text() + ")");

  if (QMessageBox::question(this, tr("Confirmation"), deletionText) == QMessageBox::Yes)
  {
    mpTableWidget->removeRow(row);

    mpTableWidget->resizeColumnsToContents();
    scrollStationList->setFixedWidth(mpTableWidget->sizeHint().width() * STATIONLISTFACTOR); // TODO: how to do correct resizing with this factor?
  }
}

void RadioInterface::tableSelect(int row, int column)
{
  (void)column;
  const QTableWidgetItem *const item = mpTableWidget->item(row, 1);
  const int32_t freq = item->text().toInt();

  newFrequency(Khz(freq));
}

void RadioInterface::delete_station_list()
{
  int16_t rows = mpTableWidget->rowCount();

  for (int16_t i = rows; i > 0; i--)
  {
    mpTableWidget->removeRow(i);
  }

  delete mpTableWidget;
}
