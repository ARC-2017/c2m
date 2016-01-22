//////////////////////////////////////////////////////////////////////////
//
//	Header for Quaternion
//	
//		Shuichi AKIZUKI      2011.12
//
//		(C) Toru Nakata, toru-nakata@aist.go.jp      
//		2004 Dec 29    
//
//		Reference:
//			http://www015.upp.so-net.ne.jp/notgeld/quaternion.html
//
//		Note:
//			2011-12-21
//				Rev.A
//
/////////////////////////////////////////////////////////////////////////
#ifndef INCLUDE_quaternion_h_
#define INCLUDE_quaternion_h_

/// Define Data type      
typedef struct quaternion{      
	double	t; // real-component      
	double	x; // x-component      
	double	y; // y-component      
	double	z; // z-component      
}quaternion;

#endif
