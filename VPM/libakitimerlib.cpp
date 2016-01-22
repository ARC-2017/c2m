//////////////////////////////////////////////////////////////////////////////
//
//	akitimer.c: Functions for calculating elapsed time.
//
//  Shuichi AKIZUKI
//
//	(C) 2013 ISL, Chukyo University All rights reserved.
//
//  Note:
//		2013.07.23
//			
//////////////////////////////////////////////////////////////////////////////
//#include "stdafx.h"
#include "libakitimerlib.h"


struct	timeb start_timebuffer, timebuffer;
time_t	start_sec, start_msec;
time_t	sec, msec;
double	elapsed_time;

void AkiStartTimer(){

	ftime( &start_timebuffer );
	start_sec = start_timebuffer.time;
	start_msec = start_timebuffer.millitm;
	std::cout << std::endl;
	std::cout << "Timer started." << std::endl;
}

time_t AkiTimeFromStart(){

	struct	timeb now_timebuffer;
	time_t	now_sec, now_msec;

	ftime( &now_timebuffer );	
	now_sec = now_timebuffer.time - start_sec;
	now_msec = now_timebuffer.millitm - start_msec;
	now_msec += now_sec*1000;
	std::cout << std::endl;
	std::cout << " Timer:" << std::endl;
	std::cout << "  Elapsed time from start:" << now_msec << "[msec]" << std::endl;
	return now_msec;
}

void AkiTimeSetFlag(){

	ftime( &timebuffer );
	sec = timebuffer.time;
	msec = timebuffer.millitm;
}

void AkiTimeLogFromFlag( char text[2048] ){

	struct	timeb now_timebuffer;
	time_t	now_sec, now_msec;
	ftime( &now_timebuffer );	
	now_sec = now_timebuffer.time - sec;
	now_msec = now_timebuffer.millitm - msec;
	now_msec += now_sec*1000;
	std::cout << " Timer:" << text << std::endl;
	std::cout << "  Log from flag:" << now_msec << "[msec]" << std::endl;
}