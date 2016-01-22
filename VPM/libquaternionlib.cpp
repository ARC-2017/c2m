//////////////////////////////////////////////////////////////////////////
//
//	Quaternion Library
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
//			2014-02-21
//				Rev.B　Quaternion関連の関数を統合して使いやすくした．
//			2014-02-14
//				void RotationByQuaternionEigen( ); を追加
//				Eigen を引数とした関数
//
/////////////////////////////////////////////////////////////////////////
//#include "stdafx.h"
#include <stdio.h>      
#include <stdlib.h>      
#include <math.h> 
#include "quaternion.h"
#include "akidata.h"
//#include "C:\Program Files (x86)\Eigen\include\Eigen\Core"


//// Kakezan      
void Kakezan( quaternion left, quaternion right, quaternion *ans )       
{      
              double   d1, d2, d3, d4;      
       
              d1   =  left.t * right.t;      
              d2   = -left.x * right.x;      
              d3   = -left.y * right.y;      
              d4   = -left.z * right.z;      
                   ans->t = d1+ d2+ d3+ d4; 
       
              d1   =  left.t * right.x;      
              d2   =  right.t * left.x;      
              d3   =  left.y * right.z;      
              d4   = -left.z * right.y;      
                   ans->x =  d1+ d2+ d3+ d4; 
       
              d1   =  left.t * right.y;      
              d2   =  right.t * left.y;      
              d3   =  left.z * right.x;      
              d4   = -left.x * right.z;      
                 ans->y =  d1+ d2+ d3+ d4; 
       
              d1   =  left.t * right.z;      
              d2   =  right.t * left.z;      
              d3   =  left.x * right.y;      
              d4   = -left.y * right.x;      
                   ans->z =  d1+ d2+ d3+ d4; 
                    
}      
       
//// Make Rotational quaternion      
void MakeRotationalQuaternion( double radian, double AxisX, double AxisY, double AxisZ, quaternion *ans )      
{      
              double   norm;      
              double   ccc, sss;      
                    
              ans->t = ans->x = ans->y = ans->z = 0.0;      
       
              norm   = AxisX *  AxisX +  AxisY *  AxisY +  AxisZ *  AxisZ;      
              if(norm   <= 0.0) return;      
       
              norm   = 1.0 / sqrt(norm);      
              AxisX   *= norm;      
              AxisY   *= norm;      
              AxisZ   *= norm;      
       
              ccc   = cos(0.5 * radian);      
              sss   = sin(0.5 * radian);      
       
              ans->t   = ccc;      
              ans->x   = sss * AxisX;      
              ans->y   = sss * AxisY;      
              ans->z   = sss * AxisZ;      
       
}      
       
//// Put XYZ into  quaternion      
void PutXYZToQuaternion( double PosX, double PosY, double PosZ, quaternion *ans )      
{      
       
              ans->t   = 0.0;      
              ans->x   = PosX;      
              ans->y   = PosY;      
              ans->z   = PosZ;      
       
}      

//ここから下の関数は秋月が書いた
void RotationByQuaternion( double3 src, double3 tp, double tpr, double3 *dst ){

	struct quaternion		ppp, qqq, rrr;

	PutXYZToQuaternion( src.x, src.y, src.z, &ppp );
	MakeRotationalQuaternion( tpr, tp.x, tp.y, tp.z, &qqq );
	MakeRotationalQuaternion( -tpr, tp.x, tp.y, tp.z, &rrr );
	Kakezan( rrr, ppp, &ppp );
	Kakezan( ppp, qqq, &ppp );
	dst->x = ppp.x;
	dst->y = ppp.y;
	dst->z = ppp.z;
}       

//void RotationByQuaternionEigen( Eigen::Vector3d &src, Eigen::Vector3d &tp, double tpr, Eigen::Vector3d &dst ){
//
//	struct quaternion		ppp, qqq, rrr;
//
//	PutXYZToQuaternion( src(0), src(1), src(2), &ppp );
//	MakeRotationalQuaternion( tpr, tp(0), tp(1), tp(2), &qqq );
//	MakeRotationalQuaternion( -tpr, tp(0), tp(1), tp(2), &rrr );
//	Kakezan( rrr, ppp, &ppp );
//	Kakezan( ppp, qqq, &ppp );
//	dst(0) = ppp.x;
//	dst(1) = ppp.y;
//	dst(2) = ppp.z;
//}     

void Quaternion2Matrix( quaternion q, double rot[][3] ){

	rot[0][0] = 1.0-(2.0*q.y*q.y)-(2.0*q.z*q.z); 	rot[0][1] = (2.0*q.x*q.y)+(2.0*q.t*q.z); 			rot[0][2] = (2.0*q.x*q.z)-(2.0*q.t*q.y);
	rot[1][0] = (2.0*q.x*q.y)-(2.0*q.t*q.z);		 rot[1][1] = 1.0-(2.0*q.x*q.x)-(2.0*q.z*q.z);	rot[1][2] = (2.0*q.y*q.z)+(2.0*q.t*q.x);
	rot[2][0] = (2.0*q.x*q.z)+(2.0*q.t*q.y);		 rot[2][1] = (2.0*q.y*q.z)-(2.0*q.t*q.x);		rot[2][2] = 1.0-(2.0*q.x*q.x)-(2.0*q.y*q.y);
	
}