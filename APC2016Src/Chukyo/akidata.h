//////////////////////////////////////////////////////////////////////////////
//
//	akidata.h: Hedder for akizuki software.
//
//  Shuichi AKIZUKI
//
//	(C) 2011 ISL, Chukyo University All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef INCLUDE_akidata_h_
#define INCLUDE_akidata_h_

#include <vector>
#include <list>
#include <iostream>
#include <algorithm>
#define CLEN 65536

typedef struct poc1{	//Pakage of a coordinate
	int		x;		
	int		y;
}poc1;

typedef struct poc2{	//Pakage of 2 coordinates
	int		x1;		
	int		y1;
	int		x2;		
	int		y2;
}poc2;

typedef struct int2{	//2D coordinate
	int		i;		
	int		j;
}int2;

typedef struct double2{
	double	x;
	double	y;
}double2;

typedef struct double3{
	double	x;
	double	y;
	double	z;
}double3;

typedef struct float3{
	float	x;
	float	y;
	float	z;
}float3;

typedef struct chain{
	unsigned char	*code;     //chain code
	int				*i;        //i coordinate
	int				*j;        //j coordinate
	int				length;
}chain;

typedef struct observability_map{

	int		nObserve;
	int		nPoints;
	std::vector< std::vector<char> > obs;
	std::vector<double3> view_point;

}observability_map;

typedef struct multi_mat3x3{
	
	double				m[3][3];
	struct double3		t1;
	struct double3		t2;
}multi_mat3x3;

#endif