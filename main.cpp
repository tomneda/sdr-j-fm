/*
 *    Copyright (C) 2014
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
 *
 *	Main program
 */
#include  <QApplication>
#include  <QSettings>
#include  <QDir>
#include  <unistd.h>
#include  "fm-constants.h"
#include  "radio.h"

void fullPathfor(const char *v, char *out)
{
  int16_t i;
  QString homeDir;

  if (v == nullptr)
  {
    sprintf(out, "%s", "weet niet");
    return;   // should not happen
  }

  if (v [0] == '/')     // full path specified
  {
    sprintf(out, "%s", v);
    return;
  }

  homeDir = QDir::homePath();
  homeDir.append("/");
  homeDir.append(v);
  homeDir = QDir::toNativeSeparators(homeDir);
  sprintf(out, "%s", homeDir.toLatin1().data());
  fprintf(stderr, "ini file = %s\n", out);

  for (i = 0; out [i] != 0; i++)
  {
    ;
  }

  if (out [i - 4] != '.' ||
      out [i - 3] != 'i' ||
      out [i - 2] != 'n' ||
      out [i - 1] != 'i')
  {
    out [i]     = '.';
    out [i + 1] = 'i';
    out [i + 2] = 'n';
    out [i + 3] = 'i';
    out [i + 4] = 0;
  }
}

bool fileExists(char *v)
{
  FILE *f;

  f = fopen(v, "r");

  if (f == nullptr)
  {
    return false;
  }

  fclose(f);
  return true;
}

static const char * const DEFAULT_INI  = ".jsdr-fm.ini";
static const char * const STATION_LIST = ".jsdr-fm-stations.bin";

int main(int argc, char **argv)
{
  int32_t opt;
/*
 *	The default values
 */
  QSettings      *ISettings; /* .ini file	*/
  int32_t        outputRate = 48000;
  RadioInterface *MyRadioInterface;
  QString        iniFile     = QDir::homePath();
  QString        stationList = QDir::homePath();

  iniFile.append("/");
  iniFile.append(DEFAULT_INI);
  iniFile = QDir::toNativeSeparators(iniFile);

  stationList.append("/");
  stationList.append(STATION_LIST);
  stationList = QDir::toNativeSeparators(stationList);

  while ((opt = getopt(argc, argv, "A:B:m:i:o:C:T:dIFEMSG")) != -1)
  {
    switch (opt)
    {
    case 'm': outputRate = 192000;
      break;

    default:
      break;
    }
  }

/*
 *	... and the settings of the "environment"
 */
  ISettings = new QSettings(iniFile, QSettings::IniFormat);
/*
 *	Before we connect control to the gui, we have to
 *	instantiate
 */
#if QT_VERSION >= 0x050600
  QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
  QApplication a(argc, argv);
  MyRadioInterface = new RadioInterface(ISettings, stationList, outputRate);
  MyRadioInterface->show();
  a.exec();

/*
 *	done:
 */
  fflush(stdout);
  fflush(stderr);
  qDebug("It is done\n");
  ISettings->sync();
//	delete MyRadioInterface;
//	ISettings	-> ~QSettings ();

  return 0;
}

