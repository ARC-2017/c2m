/////////////////////////////////////////////////////////////////////////////
//
//	libakitimerlib.h:
//
//  Shuichi AKIZUKI
//
//	(C) 2012 ISL, Chukyo University All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef INCLUDE_libakitimerlib_h_
#define INCLUDE_libakitimerlib_h_

#include <iostream>
#include <time.h>
#include <sys/timeb.h>

void AkiStartTimer();

time_t AkiTimeFromStart();

void AkiTimeSetFlag();

void AkiTimeLogFromFlag( char text[2048] );

#endif