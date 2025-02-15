#
/*
 *    Copyright (C) 2008, 2009, 2010
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair Computing
 *
 *    This file is part of the sdr-j-fm
 *
 *    SDR-J-FM is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    SDR-J-FM is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with SDR-J-FM; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __OSCILLATOR_H
#define __OSCILLATOR_H

#include	"fm-constants.h"

class	Oscillator {
public:
	        	Oscillator (DSPCOMPLEX *, int32_t);
			Oscillator (int32_t);
			~Oscillator ();
	DSPCOMPLEX	nextValue	(int32_t);
private:
	uint8_t		localTable;
	DSPCOMPLEX	*OscillatorValues;
	int32_t		Rate;
	int32_t		LOPhase;
};

#endif

