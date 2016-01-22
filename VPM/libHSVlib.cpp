/////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////
//#include "stdafx.h"

#include "libHSVlib.h"

void hsv( double sourse, int *r, int *g, int *b ){

	double	h, s, v, f;
	double	p, q, t;
	int		hi;
	
	sourse = 1.0 - sourse;
	h = (sourse*240.0);
	if( 360.0 <= h ) h-=360.0;
	s = v = 1.0;

	hi = (int)(((int)h/60)%6);
	f  = ((double)h/60.0)-(double)hi;
	p  = (v*(1.0-s)); 
	q  = (v*(1.0-f*s)); 
	t  = (v*(1.0-(1.0-f)*s));

	if( hi == 0 ){
		 *r = (int)(255.0*v);
		 *g = (int)(255.0*t);
		 *b = (int)(255.0*p);
	}else if( hi == 1 ){
		 *r = (int)(255.0*q);
		 *g = (int)(255.0*v);
		 *b = (int)(255.0*p);
	}else if( hi == 2 ){
		 *r = (int)(255.0*p);
		 *g = (int)(255.0*v);
		 *b = (int)(255.0*t);
	}else if( hi == 3 ){
		 *r = (int)(255.0*p);
		 *g = (int)(255.0*q);
		 *b = (int)(255.0*v);
	}else if( hi == 4 ){
		 *r = (int)(255.0*t);
		 *g = (int)(255.0*p);
		 *b = (int)(255.0*v);
	}else{
		 *r = (int)(255.0*v);
		 *g = (int)(255.0*p);
		 *b = (int)(255.0*q);
	}
}

void rgb2hue( unsigned char r, unsigned char g, unsigned char b, double *hue ){

	unsigned char max, min;
	double hue360;

	if( (r<b) && (r<g) ){
		min = r;
	}else if( (g<r) && (g<b) ){
		min = g;
	}else{
		min = b;
	}


	if( (g<r) && (b<r) ){
		max = r;
		hue360 = 60.0*(((double)g-(double)b)/(double)(max-min));
	}else if( (r<g) && (b<g) ){
		max = g;
		hue360 = 60.0*(((double)b-(double)r)/(double)(max-min)) - 120.0;
	}else{
		max = b;
		hue360 = 60.0*(((double)r-(double)g)/(double)(max-min)) - 240.0;
	}

	if( hue360 < 0.0 ){
		hue360 += 360.0;
	}

	*hue = hue360/360.0;
}


