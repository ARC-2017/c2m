/////////////////////////////////////////////////////////////////////////////
//
//	libquaternionlib.h:
//
//  Shuichi AKIZUKI
//
//	(C) 2012 ISL, Chukyo University All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef INCLUDE_libquaternionlib_h_
#define INCLUDE_libquaternionlib_h_

void Kakezan( quaternion left, quaternion right, quaternion *ans );       
       
void MakeRotationalQuaternion( double radian, double AxisX, double AxisY, double AxisZ, quaternion *ans );      
       
void PutXYZToQuaternion( double PosX, double PosY, double PosZ, quaternion *ans );      

void RotationByQuaternion( double3 src, double3 tp, double tpr, double3 *dst );

//void RotationByQuaternionEigen( Eigen::Vector3d &src, Eigen::Vector3d &tp, double tpr, Eigen::Vector3d &dst );

void Quaternion2Matrix( quaternion q, double rot[][3] );

#endif