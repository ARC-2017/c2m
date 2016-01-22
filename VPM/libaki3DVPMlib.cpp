//////////////////////////////////////////////////////////////////////////////
//
//	aki3DVPMlib.c: Function for 3-D Vector Pair Matching.
//
//  Shuichi AKIZUKI
//
//	(C) 2012 ISL, Chukyo University All rights reserved.
//  
//	Last Update: 2015.04.29
//
//  Note:
//  2015.04.21
//		RecgVPM() において，点群を保存する際に，点数が0のときにハングアップする
//		ことがわかった(PCLの問題点)ので，そのような場合には保存しないようにする
//		ための処理を追加した．（解決済） 
//  2015.04.29
//      連続実行時にファイルポインタをクローズしなければ，2回目以降で落ちることが
//		がわかった．
//		returnするタイミングで毎回ログファイルのファイルポインタをクローズする
//		ように変更した．(解決済)
//  2015.05.21
//      入力画像上のエッジを入力点群に追加することにした．
//      これにより，箱が正面を向いていても認識できるようになった．
//
//      投票の位置を物体の距離画像の奥のみを有効にした．
//      これにより，物体間の位置のおかしな姿勢仮説を棄却できるようになった．
//
//			
//////////////////////////////////////////////////////////////////////////////
//#include "stdafx.h"

#include "libaki3DVPMlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <math.h>

void AkiCreateInt3D( int isx, int isy, int isz, aki_int_3d *II ){
	
	int	v, j;

	II->sx = isx;	II->sy = isy;	II->sz = isz;
	II->img3d = (int ***)malloc( isz*sizeof(int **) );

	for( v=0 ; v<isz ; v++ ){
		II->img3d[v] = (int **)malloc( isy*sizeof(int *) );
	}
	for( v=0 ; v<isz ; v++ ){
		for( j=0 ; j<isy ; j++ ){
			II->img3d[v][j] = (int *)malloc( isx*sizeof(int) );
		}
	}

}

void AkiClearInt3D( aki_int_3d *II, int value ){

	int		i, j, k;
	int		isx, isy, isz;

	isx = II->sx;
	isy = II->sy;
	isz = II->sz;

	for( k=0 ; k<isz ; k++ ){
		for( j=0 ; j<isy ; j++ ){
			for( i=0 ; i<isx ; i++ ){
				II->img3d[k][j][i] = value;
			}
		}
	}
}

inline void AkiCalcLength( double3 v, double *length ){
	*length = sqrt( (v.x*v.x)+(v.y*v.y)+(v.z*v.z) );
}

void AkiCalcDistance( double3 v1, double3 v2, double *dist ){
	*dist = sqrt( ((v2.x - v1.x)*(v2.x - v1.x))
			    + ((v2.y - v1.y)*(v2.y - v1.y))
		        + ((v2.z - v1.z)*(v2.z - v1.z)));
}

void AkiNormalize( const struct double3 v, double3 *normalized ){

	double length;
	AkiCalcLength( v, &length );
	
	normalized->x = v.x / length;
	normalized->y = v.y / length;
	normalized->z = v.z / length;
	
}

inline void AkiCrossProduct( double3 *va, double3 *vb, double3 *vc ){
	vc->x =    va->y*vb->z - va->z*vb->y;
	vc->y = -( va->x*vb->z - va->z*vb->x );
	vc->z =    va->x*vb->y - va->y*vb->x;
}

inline void AkiInnerProduct( double3 va, double3 vb, double *ip ){

	double	l1, l2;

	//Calculation length
//	l1 = sqrt( (va.x*va.x)+(va.y*va.y)+(va.z*va.z) );
//	l2 = sqrt( (vb.x*vb.x)+(vb.y*vb.y)+(vb.z*vb.z) );
	AkiCalcLength( va, &l1 );
	AkiCalcLength( vb, &l2 );

//	fprintf( SE,"InnerProduct\n" );
//	fprintf( SE,"va(%lf,%lf,%lf) l1 = %lf\n", va.x, va.y, va.z, l1 );
//	fprintf( SE,"vb(%lf,%lf,%lf) l2 = %lf\n", vb.x, vb.y, vb.z, l2 );

	//Calculation inner product
	*ip = ((va.x*vb.x)+(va.y*vb.y)+(va.z*vb.z))/(l1*l2);	
}

inline void Aki3DMatrixx3DPoint( double rot[][3], double3 v, double3 *output ){

	output->x = (rot[0][0]*v.x)+(rot[0][1]*v.y)+(rot[0][2]*v.z);
	output->y = (rot[1][0]*v.x)+(rot[1][1]*v.y)+(rot[1][2]*v.z);
	output->z = (rot[2][0]*v.x)+(rot[2][1]*v.y)+(rot[2][2]*v.z);

}

inline void AkiMatrixxMatrix3D( double a[][3], double b[][3], double ans[][3] ){

	ans[0][0] = (a[0][0]*b[0][0])+(a[0][1]*b[1][0])+(a[0][2]*b[2][0]);
	ans[0][1] = (a[0][0]*b[0][1])+(a[0][1]*b[1][1])+(a[0][2]*b[2][1]);
	ans[0][2] = (a[0][0]*b[0][2])+(a[0][1]*b[1][2])+(a[0][2]*b[2][2]);

	ans[1][0] = (a[1][0]*b[0][0])+(a[1][1]*b[1][0])+(a[1][2]*b[2][0]);
	ans[1][1] = (a[1][0]*b[0][1])+(a[1][1]*b[1][1])+(a[1][2]*b[2][1]);
	ans[1][2] = (a[1][0]*b[0][2])+(a[1][1]*b[1][2])+(a[1][2]*b[2][2]);

	ans[2][0] = (a[2][0]*b[0][0])+(a[2][1]*b[1][0])+(a[2][2]*b[2][0]);
	ans[2][1] = (a[2][0]*b[0][1])+(a[2][1]*b[1][1])+(a[2][2]*b[2][1]);
	ans[2][2] = (a[2][0]*b[0][2])+(a[2][1]*b[1][2])+(a[2][2]*b[2][2]);

}

void AkiCalcInverseMatrix3D( double mat[][3], double matinv[][3] ){

	double det;
	
	//00 01 02
	//10 11 12
	//20 21 22
/*
	fprintf( SE,"    |%8lf %8lf %8lf|\n", mat[0][0], mat[0][1], mat[0][2] ); 
	fprintf( SE,"M = |%8lf %8lf %8lf|\n", mat[1][0], mat[1][1], mat[1][2] ); 
	fprintf( SE,"    |%8lf %8lf %8lf|\n", mat[2][0], mat[2][1], mat[2][2] ); 
*/
	det = (mat[0][0]*mat[1][1]*mat[2][2])+(mat[1][0]*mat[2][1]*mat[0][2])+(mat[2][0]*mat[0][1]*mat[1][2])
		-(mat[2][0]*mat[1][1]*mat[0][2])-(mat[2][1]*mat[1][2]*mat[0][0])-(mat[2][2]*mat[1][0]*mat[0][1]);

//	fprintf( SE,"det = %e\n", det );
	matinv[0][0] = ((mat[1][1]*mat[2][2])-(mat[1][2]*mat[2][1]))/det;
	matinv[0][1] = ((mat[0][2]*mat[2][1])-(mat[0][1]*mat[2][2]))/det;
	matinv[0][2] = ((mat[0][1]*mat[1][2])-(mat[0][2]*mat[1][1]))/det;
	matinv[1][0] = ((mat[1][2]*mat[2][0])-(mat[1][0]*mat[2][2]))/det;
	matinv[1][1] = ((mat[0][0]*mat[2][2])-(mat[0][2]*mat[2][0]))/det;
	matinv[1][2] = ((mat[0][2]*mat[1][0])-(mat[0][0]*mat[1][2]))/det;
	matinv[2][0] = ((mat[1][0]*mat[2][1])-(mat[1][1]*mat[2][0]))/det;
	matinv[2][1] = ((mat[0][1]*mat[2][0])-(mat[0][0]*mat[2][1]))/det;
	matinv[2][2] = ((mat[0][0]*mat[1][1])-(mat[0][1]*mat[1][0]))/det;
		
}

inline void AkiMakeTransposedMatrix3D( double mat[][3], double trans[][3] ){

	//00 01 02
	//10 11 12
	//20 21 22
	trans[0][0] = mat[0][0]; trans[0][1] = mat[1][0]; trans[0][2] = mat[2][0];
	trans[1][0] = mat[0][1]; trans[1][1] = mat[1][1]; trans[1][2] = mat[2][1];
	trans[2][0] = mat[0][2]; trans[2][1] = mat[1][2]; trans[2][2] = mat[2][2];
	
}

void AkiCopyVectorPair( vector_pair *vp1, vector_pair *vp2 ){

	vp2->p.x = vp1->p.x; vp2->p.y = vp1->p.y; vp2->p.z = vp1->p.z;
	vp2->q1.x = vp1->q1.x; vp2->q1.y = vp1->q1.y; vp2->q1.z = vp1->q1.z;
	vp2->q2.x = vp1->q2.x; vp2->q2.y = vp1->q2.y; vp2->q2.z = vp1->q2.z;
	vp2->np.x = vp1->np.x; vp2->np.y = vp1->np.y; vp2->np.z = vp1->np.z;
	vp2->nq1.x = vp1->nq1.x; vp2->nq1.y = vp1->nq1.y; vp2->nq1.z = vp1->nq1.z;
	vp2->nq2.x = vp1->nq2.x; vp2->nq2.y = vp1->nq2.y; vp2->nq2.z = vp1->nq2.z;
	vp2->nvp.x = vp1->nvp.x; vp2->nvp.y = vp1->nvp.y; vp2->nvp.z = vp1->nvp.z;
	vp2->ip_p = vp1->ip_p; vp2->ip_q1 = vp1->ip_q1; vp2->ip_q2 = vp1->ip_q2;
	vp2->vc.x = vp1->vc.x; vp2->vc.y = vp1->vc.y; vp2->vc.z = vp1->vc.z;
	vp2->distinctiveness = vp1->distinctiveness;
	vp2->observability = vp1->observability;
	vp2->normalized_obs = vp1->normalized_obs;
	vp2->normalized_op = vp1->normalized_op;
	vp2->occurrence_prob = vp2->occurrence_prob;
}

inline void AkiInitializeVectorPair( vector_pair *vp, vector_pair *ivp ){

	double		length_p, length_q1, length_q2;

	AkiCopyVectorPair( vp, ivp );

	length_p = sqrt( (vp->p.x*vp->p.x) + (vp->p.y*vp->p.y) + (vp->p.z*vp->p.z) );
	length_q1 = sqrt( (vp->q1.x*vp->q1.x) + (vp->q1.y*vp->q1.y) + (vp->q1.z*vp->q1.z) );
	length_q2 = sqrt( (vp->q2.x*vp->q2.x) + (vp->q2.y*vp->q2.y) + (vp->q2.z*vp->q2.z) );

	if( 0.0001<fabs(length_p) ){
		ivp->p.x = vp->p.x/length_p; ivp->p.y = vp->p.y/length_p; ivp->p.z = vp->p.z/length_p;
	}
	if( 0.0001<fabs(length_q1) ){
		ivp->q1.x = vp->q1.x/length_q1; ivp->q1.y = vp->q1.y/length_q1; ivp->q1.z = vp->q1.z/length_q1;
	}
	if( 0.0001<fabs(length_q2) ){
		ivp->q2.x = vp->q2.x/length_q2; ivp->q2.y = vp->q2.y/length_q2; ivp->q2.z = vp->q2.z/length_q2;
	}

}

void AkiShowVectorPair( vector_pair *vp ){
	
	fprintf( stdout,"idx[P:%d, Q1:%d, Q2:%d]\n", vp->p_idx, vp->q1_idx, vp->q2_idx );
	fprintf( stdout,"p( %5lf, %5lf, %5lf)  np( %5lf, %5lf, %5lf)\n", vp->p.x, vp->p.y, vp->p.z, vp->np.x, vp->np.y, vp->np.z );
	fprintf( stdout,"q1( %5lf, %5lf, %5lf)  nq1( %5lf, %5lf, %5lf)\n", vp->q1.x, vp->q1.y, vp->q1.z, vp->nq1.x, vp->nq1.y, vp->nq1.z );
	fprintf( stdout,"q2( %5lf, %5lf, %5lf)  nq2( %5lf, %5lf, %5lf)\n", vp->q2.x, vp->q2.y, vp->q2.z, vp->nq2.x, vp->nq2.y, vp->nq2.z );
	fprintf( stdout,"nvp( %5lf, %5lf, %5lf)\n", vp->nvp.x, vp->nvp.y, vp->nvp.z );
	fprintf( stdout,"vc( %5lf, %5lf, %5lf)\n", vp->vc.x, vp->vc.y, vp->vc.z );
	fprintf( stdout,"observability: %5lf\n", vp->observability );
	fprintf( stdout,"occurrence_probability: %5lf\n", vp->occurrence_prob );
	fprintf( stdout,"distinctivness: %5lf\n", vp->distinctiveness );
	fprintf( stdout,"\n" );
}

void AkiClearVectorPair( vector_pair *vp ){

	vp->p.x = vp->p.y = vp->p.z = vp->q1.x = vp->q1.y = vp->q1.z = vp->q2.x = vp->q2.y = vp->q2.z = 0.0;
	vp->np.x = vp->np.y = vp->np.z = vp->nq1.x = vp->nq1.y = vp->nq1.z = vp->nq2.x = vp->nq2.y = vp->nq2.z = 0.0;
	vp->nvp.x = vp->nvp.y = vp->nvp.z = 0.0;
	vp->ip_p = vp->ip_q1 = vp->ip_q2 = 0.0;
	vp->vc.x = vp->vc.y = vp->vc.z = 0.0;
	vp->p_idx = vp->q1_idx = vp->q2_idx = 0;
}

inline void AkiVectorPair2Matrix3D( vector_pair *vp, double mat[][3] ){

	double3		cp;
	AkiCrossProduct( &vp->q1, &vp->q2, &cp );
	mat[0][0] = vp->q1.x; mat[0][1] = vp->q2.x; mat[0][2] = cp.x;
	mat[1][0] = vp->q1.y; mat[1][1] = vp->q2.y; mat[1][2] = cp.y;
	mat[2][0] = vp->q1.z; mat[2][1] = vp->q2.z; mat[2][2] = cp.z;
/*
	mat[0][0] = vp->q1.x; mat[0][1] = vp->q1.y; mat[0][2] = vp->q1.z;
	mat[1][0] = vp->q2.x; mat[1][1] = vp->q2.y; mat[1][2] = vp->q2.z;
	mat[2][0] = cp.x; mat[2][1] = cp.y; mat[2][2] = cp.z;
*/
}

void AkiShowMatrix3D( double mat[][3] ){

	fprintf( SE,"|%8lf %8lf %8lf|\n", mat[0][0], mat[0][1], mat[0][2] ); 
	fprintf( SE,"|%8lf %8lf %8lf|\n", mat[1][0], mat[1][1], mat[1][2] ); 
	fprintf( SE,"|%8lf %8lf %8lf|\n", mat[2][0], mat[2][1], mat[2][2] ); 
}
	
inline void AkiCalcRotationalMatrixFrom2VectorPairs( vector_pair *vpM, vector_pair *vpS, double matR[][3] ){

	struct vector_pair		vpMn, vpSn;
	double					matM[3][3] = { 0.0 };
	double					matS[3][3] = { 0.0 };
	double					matMtrans[3][3] = { 0.0 };

	AkiInitializeVectorPair( vpM, &vpMn );
	AkiInitializeVectorPair( vpS, &vpSn );

	AkiVectorPair2Matrix3D( &vpMn, matM );
	AkiVectorPair2Matrix3D( &vpSn, matS );

	AkiMakeTransposedMatrix3D( matM, matMtrans );

	AkiMatrixxMatrix3D( matS, matMtrans, matR );
} 

void AkiVectorPairOrthonormalization( vector_pair *vp, vector_pair *vpo ){

	double3		v_prj;
	double		ip;
	double		length, length_vpo;
	vector_pair	vpn;


	AkiInitializeVectorPair( vp, &vpn );
	AkiInnerProduct( vp->q1, vp->q2, &ip );
	length = sqrt( (vp->q2.x*vp->q2.x) + (vp->q2.y*vp->q2.y) + (vp->q2.z*vp->q2.z) );

	v_prj.x = length*ip*vpn.q1.x;
	v_prj.y = length*ip*vpn.q1.y;
	v_prj.z = length*ip*vpn.q1.z;
	
	AkiCopyVectorPair( &vpn, vpo );


	vpo->q2.x = vp->q2.x - v_prj.x;
	vpo->q2.y = vp->q2.y - v_prj.y;
	vpo->q2.z = vp->q2.z - v_prj.z;
	length_vpo = sqrt( (vpo->q2.x*vpo->q2.x) + (vpo->q2.y*vpo->q2.y) + (vpo->q2.z*vpo->q2.z) );
	vpo->q2.x /= length_vpo;
	vpo->q2.y /= length_vpo;
	vpo->q2.z /= length_vpo;
}

void AkiCalcOccurrenceProbabilityOfVectorPair( std::vector<vector_pair>& vp, struct aki_int_3d *HH3D ){

	int		nVectorPair, reso;
	int		quant_ip_p, quant_ip_q1, quant_ip_q2;

	nVectorPair = vp.size();
	reso = HH3D->sx;

	for( int i=0 ; i<nVectorPair ; i++ ){
		quant_ip_p = (int)(((double)(reso-1)*((vp[i].ip_p+1.0))/2.0)+0.5);
		quant_ip_q1 = (int)(((double)(reso-1)*((vp[i].ip_q1+1.0))/2.0)+0.5);
		quant_ip_q2 = (int)(((double)(reso-1)*((vp[i].ip_q2+1.0))/2.0)+0.5);
		vp[i].occurrence_prob = (double)((double)HH3D->img3d[ quant_ip_p ][ quant_ip_q1 ][ quant_ip_q2 ]/(double)nVectorPair);
	}

}


void AkiNormalizeProbabilityOfVectorPair( std::vector<vector_pair>& vp ){

	int		nVectorPair;
	double	max_op, min_op, max_obs, min_obs;

	nVectorPair = vp.size();
	
	max_op = max_obs = 0.0;
	min_op = min_obs = 1.0;
	for( int i=0 ; i<nVectorPair ; i++ ){
		if( max_op < vp[i].occurrence_prob ) max_op = vp[i].occurrence_prob;
		if( max_obs < vp[i].observability ) max_obs = vp[i].observability;
		if( vp[i].occurrence_prob < min_op ) min_op = vp[i].occurrence_prob;
		if( vp[i].observability < min_obs ) min_obs = vp[i].observability;
	}
	fprintf( stderr,"max_op: %lf, min_op: %lf\n", max_op, min_op );
	fprintf( stderr,"max_obs: %lf, min_obs: %lf\n", max_obs, min_obs );
	for( int i=0 ; i<nVectorPair ; i++ ){
		vp[i].normalized_op = (vp[i].occurrence_prob - min_op)/(max_op - min_op);
		vp[i].normalized_obs = (vp[i].observability - min_obs)/(max_obs - min_obs);
	}

}

void AkiIntegrationOccurenceProbabilityAndObservability( std::vector<vector_pair>& vp ){

	int		nVectorPair;

	nVectorPair = vp.size();

	for( int i=0 ; i<nVectorPair ; i++ ){
		//ここに統合処理を書く
		vp[i].distinctiveness = (1.0-vp[i].normalized_op)*vp[i].normalized_obs;
		//重みつき合成
		//vp[i].distinctiveness = (1.0-vp[i].normalized_op)+vp[i].normalized_obs;
//		vp[i].distinctiveness = (1.0-vp[i].normalized_op);
//		vp[i].distinctiveness = (vp[i].normalized_obs);
	}

}

void AkiIntegrationOccurenceProbability( std::vector<vector_pair>& vp ){

	int		nVectorPair;

	nVectorPair = vp.size();

	for( int i=0 ; i<nVectorPair ; i++ ){
		//ここに統合処理を書く
		vp[i].distinctiveness = (1.0-vp[i].normalized_op);
	}

}

void AkiIntegrationObservability( std::vector<vector_pair>& vp ){

	int		nVectorPair;

	nVectorPair = vp.size();

	for( int i=0 ; i<nVectorPair ; i++ ){
		//ここに統合処理を書く
		vp[i].distinctiveness = (vp[i].normalized_obs);
	}

}

void AkiCombSortVectorPair( std::vector<vector_pair>& vp ){

	static int	n, Ndata;
	int			gap, swap_int;
	double		tmp_d;
	int			tmp_i;

	Ndata = vp.size();

	gap = Ndata;
	swap_int = 1;
	while( gap>1 || swap_int ) {
		gap = (int)((double)gap/1.3) ;
		if(gap == 9 || gap == 10) gap = 11 ;
		swap_int = 0;
		for( n=0; n<(Ndata-gap) ; n++ ){
			if( vp[n].distinctiveness < vp[n+gap].distinctiveness ){
				tmp_d = vp[n].p.x; vp[n].p.x = vp[n+gap].p.x; vp[n+gap].p.x = tmp_d;
				tmp_d = vp[n].p.y; vp[n].p.y = vp[n+gap].p.y; vp[n+gap].p.y = tmp_d;
				tmp_d = vp[n].p.z; vp[n].p.z = vp[n+gap].p.z; vp[n+gap].p.z = tmp_d;

				tmp_d = vp[n].q1.x; vp[n].q1.x = vp[n+gap].q1.x; vp[n+gap].q1.x = tmp_d;
				tmp_d = vp[n].q1.y; vp[n].q1.y = vp[n+gap].q1.y; vp[n+gap].q1.y = tmp_d;
				tmp_d = vp[n].q1.z; vp[n].q1.z = vp[n+gap].q1.z; vp[n+gap].q1.z = tmp_d;

				tmp_d = vp[n].q2.x; vp[n].q2.x = vp[n+gap].q2.x; vp[n+gap].q2.x = tmp_d;
				tmp_d = vp[n].q2.y; vp[n].q2.y = vp[n+gap].q2.y; vp[n+gap].q2.y = tmp_d;
				tmp_d = vp[n].q2.z; vp[n].q2.z = vp[n+gap].q2.z; vp[n+gap].q2.z = tmp_d;

				tmp_d = vp[n].vc.x; vp[n].vc.x = vp[n+gap].vc.x; vp[n+gap].vc.x = tmp_d;
				tmp_d = vp[n].vc.y; vp[n].vc.y = vp[n+gap].vc.y; vp[n+gap].vc.y = tmp_d;
				tmp_d = vp[n].vc.z; vp[n].vc.z = vp[n+gap].vc.z; vp[n+gap].vc.z = tmp_d;

				tmp_i = vp[n].p_idx; vp[n].p_idx = vp[n+gap].p_idx; vp[n+gap].p_idx = tmp_i;
				tmp_i = vp[n].q1_idx; vp[n].q1_idx = vp[n+gap].q1_idx; vp[n+gap].q1_idx = tmp_i;
				tmp_i = vp[n].q2_idx; vp[n].q2_idx = vp[n+gap].q2_idx; vp[n+gap].q2_idx = tmp_i;

				tmp_d = vp[n].distinctiveness; vp[n].distinctiveness = vp[n+gap].distinctiveness; vp[n+gap].distinctiveness = tmp_d;
				tmp_d = vp[n].observability; vp[n].observability = vp[n+gap].observability; vp[n+gap].observability = tmp_d;
				tmp_d = vp[n].occurrence_prob; vp[n].occurrence_prob = vp[n+gap].occurrence_prob; vp[n+gap].occurrence_prob = tmp_d;

				tmp_d = vp[n].np.x; vp[n].np.x = vp[n+gap].np.x; vp[n+gap].np.x = tmp_d;
				tmp_d = vp[n].np.y; vp[n].np.y = vp[n+gap].np.y; vp[n+gap].np.y = tmp_d;
				tmp_d = vp[n].np.z; vp[n].np.z = vp[n+gap].np.z; vp[n+gap].np.z = tmp_d;

				tmp_d = vp[n].nq1.x; vp[n].nq1.x = vp[n+gap].nq1.x; vp[n+gap].nq1.x = tmp_d;
				tmp_d = vp[n].nq1.y; vp[n].nq1.y = vp[n+gap].nq1.y; vp[n+gap].nq1.y = tmp_d;
				tmp_d = vp[n].nq1.z; vp[n].nq1.z = vp[n+gap].nq1.z; vp[n+gap].nq1.z = tmp_d;

				tmp_d = vp[n].nq2.x; vp[n].nq2.x = vp[n+gap].nq2.x; vp[n+gap].nq2.x = tmp_d;
				tmp_d = vp[n].nq2.y; vp[n].nq2.y = vp[n+gap].nq2.y; vp[n+gap].nq2.y = tmp_d;
				tmp_d = vp[n].nq2.z; vp[n].nq2.z = vp[n+gap].nq2.z; vp[n+gap].nq2.z = tmp_d;

				tmp_d = vp[n].nvp.x; vp[n].nvp.x = vp[n+gap].nvp.x; vp[n+gap].nvp.x = tmp_d;
				tmp_d = vp[n].nvp.y; vp[n].nvp.y = vp[n+gap].nvp.y; vp[n+gap].nvp.y = tmp_d;
				tmp_d = vp[n].nvp.z; vp[n].nvp.z = vp[n+gap].nvp.z; vp[n+gap].nvp.z = tmp_d;

				tmp_d = vp[n].ip_p; vp[n].ip_p = vp[n+gap].ip_p; vp[n+gap].ip_p = tmp_d;
				tmp_d = vp[n].ip_q1; vp[n].ip_q1 = vp[n+gap].ip_q1; vp[n+gap].ip_q1 = tmp_d;
				tmp_d = vp[n].ip_q2; vp[n].ip_q2 = vp[n+gap].ip_q2; vp[n+gap].ip_q2 = tmp_d;

				swap_int = 1;
			}
		}
	}

}

void AkiRotateMatrix2EulerAngleXYZ( double mat[][3], double *EulerX, double *EulerY, double *EulerZ ){

	*EulerX = atan2( mat[2][1], mat[2][2] );
	*EulerY = asin( -mat[2][0] );
	*EulerZ = atan2( mat[1][0], mat[0][0] );
}

void AkiEulerAngleXYZ2RotateMatrix( double EulerX, double EulerY, double EulerZ, double mat[][3] ){

	mat[0][0] = cos(EulerY)*cos(EulerZ); mat[0][1] = sin(EulerX)*sin(EulerY)*cos(EulerZ)-(cos(EulerX)*sin(EulerZ)); mat[0][2] = cos(EulerX)*sin(EulerY)*cos(EulerZ)+(sin(EulerX)*sin(EulerZ));
	mat[1][0] = cos(EulerY)*sin(EulerZ); mat[1][1] = sin(EulerX)*sin(EulerY)*sin(EulerZ)+(cos(EulerX)*cos(EulerZ)); mat[1][2] = cos(EulerX)*sin(EulerY)*sin(EulerZ)-(sin(EulerX)*cos(EulerZ));
	mat[2][0] = -sin(EulerY);            mat[2][1] = sin(EulerX)*cos(EulerY);                                       mat[2][2] = cos(EulerX)*cos(EulerY);
}

void AkiCreateVotingSpaceTrans( int sx, int sy, int sz, int nVP, VotingSpaceTrans *VSTrans ){

	VSTrans->sx = sx;
	VSTrans->sy = sy;
	VSTrans->sz = sz;
	VSTrans->nVP = nVP;

	VSTrans->kind.resize( sz );
	VSTrans->id.resize( sz );
	VSTrans->nVote.resize( sz );
	for( int k=0 ; k<sz ; k++ ){
		VSTrans->kind[k].resize( sy );
		VSTrans->id[k].resize( sy );
		VSTrans->nVote[k].resize( sy );
	}
	for( int k=0 ; k<sz ; k++ ){
		for( int j=0 ; j<sy ; j++ ){
			VSTrans->kind[k][j].resize( sx );
			VSTrans->id[k][j].resize( sx );
			VSTrans->nVote[k][j].resize( sx );
		}
	}
	for( int k=0 ; k<sz ; k++ ){
		for( int j=0 ; j<sy ; j++ ){
			for( int i=0 ; i<sx ; i++ ){
				VSTrans->kind[k][j][i].resize( nVP );
			}
		}
	}
}

void AkiClearVotingSpaceTrans( VotingSpaceTrans *VSTrans ){

	int		sx, sy, sz, nVP;
	
	sx = VSTrans->sx;
	sy = VSTrans->sy;
	sz = VSTrans->sz;
	nVP = VSTrans->nVP;
	for( int k=0 ; k<sz ; k++ ){
		for( int j=0 ; j<sy ; j++ ){
			for( int i=0 ; i<sx ; i++ ){
				for( int n=0 ; n<nVP ; n++ ){
					VSTrans->kind[k][j][i][n] = 0;
				}
			}
		}
	}

}

void AkiRot2RollPitchYaw(double rot[][3], double *roll, double *pitch, double *yaw){

	*roll = atan2( rot[2][1], rot[2][2] );
	*pitch = atan2(-rot[2][0], sqrt( rot[2][1]*rot[2][1]+rot[2][2]*rot[2][2] ) );
	*yaw = atan2( rot[1][0], rot[0][0] );
}

void AkiRollPitchYaw2Rot(double roll, double pitch, double yaw, double rot[][3]){

	if(        roll< -3.141 ) roll += 6.282;
	else if( 3.141 < roll )   roll -= 6.282;
	if(      pitch < -3.141 ) pitch += 6.282;
	else if( 3.141 < pitch )  pitch -= 6.282;
	if(        yaw < -3.141 ) yaw += 6.282;
	else if( 3.141 < yaw )    yaw -= 6.282;
	rot[0][0] = cos(yaw)*cos(pitch);
	rot[0][1] = -sin(yaw)*cos(roll) + (cos(yaw)*sin(pitch)*sin(roll));
	rot[0][2] = sin(yaw)*sin(roll) + (cos(yaw)*sin(pitch)*cos(roll));
	rot[1][0] = sin(yaw)*cos(pitch);
	rot[1][1] = cos(yaw)*cos(roll) + (sin(yaw)*sin(pitch)*sin(roll));
	rot[1][2] = -cos(yaw)*sin(roll) + (sin(yaw)*sin(pitch)*cos(roll));
	rot[2][0] = -sin(pitch);
	rot[2][1] = cos(pitch)*sin(roll);
	rot[2][2] = cos(pitch)*cos(roll);

}

void CombSort( const std::vector<float>& values, std::vector<int>& Idx ){

	static int	n, Ndata;
	int			gap, swap_int;
	int			tmp;

	Ndata = values.size();
	if( Ndata != Idx.size() ){
		fprintf( stderr,"Error! Size of vp and Idx is different.\n" );
		fprintf( stderr,"		Size of values : %d\n", values.size() );
		fprintf( stderr,"		Size of Idx: %d\n", Idx.size() );
	}

	gap = Ndata;
	swap_int = 1;
	while( gap>1 || swap_int ) {
		gap = (int)((double)gap/1.3) ;
		if(gap == 9 || gap == 10) gap = 11 ;
		swap_int = 0;
		for( n=0; n<(Ndata-gap) ; n++ ){
			if( values[Idx[n]] < values[Idx[n+gap]] ){
				tmp = Idx[n]; Idx[n] = Idx[n+gap]; Idx[n+gap] = tmp;
				swap_int = 1;
			}
		}
	}

}

void AkiVectorPairRandomReduce( std::vector<vector_pair>& vp, const int n_reduce ){

	

	if( n_reduce <= vp.size() ){

		for( int i=0 ; i<vp.size() ; i++ ){
			vp[i].distinctiveness = (double)rand()/(double)RAND_MAX;
		}

		AkiCombSortVectorPair( vp );


		fprintf( SE,"n_reduce = %d\n", n_reduce );
		fprintf( SE,"vp.size\n" );
		fprintf( SE," %d ->", vp.size() );
		if( n_reduce < vp.size() ){
			int n_pop_back;
			n_pop_back = vp.size()-n_reduce;
			for( int i=0 ; i<n_pop_back ; i++ ){
				vp.pop_back();
			}
		}
		fprintf( SE," %d\n", vp.size() );

	}
}



bool RecgVPM( int *itemIdx, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
			  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge, double *VPMworkPos, double *VPMScore, 
			  int *CenterIdx, std::vector<int>& cloud_cluster_idx ){

	// 時間計測の開始
	AkiStartTimer();
	AkiTimeSetFlag();
	// 距離画像生成用パラメータ
	int				img_cols, img_rows;
	struct double3	transZero;
	double			pixel_pitch;

	img_cols = IMG_COLS;
	img_rows = IMG_ROWS;
	pixel_pitch = PIXEL_PITCH;  //1画素当たりのピッチ
	transZero.x = TRANS_ZERO_X;//オフセット
	transZero.y = TRANS_ZERO_Y; 
	transZero.z = TRANS_ZERO_Z;

	char					vpfname[FNL], model3Dfname[FNL], name[FNL];
	float					s_leaf, n_neighbor, cur, th_flat;
	FILE					*fp, *fp_log;
	struct vector_pair		*mvp;
	int						Nvp;
	struct double3			mc;
	float					l1, l2, theta;
	int						K_Search;
	int						key_x, key_y, key_z;
	int						n_bin;
	int						cnt;
	double					deg2rad, rad2deg;
	double					th_theta;
	double					th_l;
	struct double3			vp_dir;  //センサと正対した法線の方向

	// Logファイル記録用
	fopen_s( &fp_log, "log_VPM.txt", "w" );

	n_bin = N_BIN;
	th_theta = 3.0;//3.0
	th_l = 3.0;//1.0
	deg2rad = M_PI/180.0;
	rad2deg = 180.0/M_PI;
	vp_dir.x = VP_DIR_X;
	vp_dir.y = VP_DIR_Y;
	vp_dir.z = VP_DIR_Z;

	sprintf( model3Dfname,"%s\\%d\\%d.pcd", MODEL_PATH, *itemIdx, *itemIdx );
	sprintf( vpfname,"%s\\%d\\%d.vp", MODEL_PATH, *itemIdx, *itemIdx );

	fprintf( fp_log,"--------------------------------------------\n");
	fprintf( fp_log,"Vector Pair Matching\n");
	fprintf( fp_log,"Vector pair: %s\n", vpfname);
	fprintf( fp_log,"# of bin of the table: %d\n", n_bin);
	fprintf( fp_log,"Object model: %s\n", model3Dfname);
	fprintf( fp_log,"--------------------------------------------\n");

	pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_model_reduced (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointXYZ tmp_cloud;

	//回転成分
	std::vector<double> rotAns11; std::vector<double> rotAns12; std::vector<double> rotAns13;
	std::vector<double> rotAns21; std::vector<double> rotAns22; std::vector<double> rotAns23;
	std::vector<double> rotAns31; std::vector<double> rotAns32; std::vector<double> rotAns33;
	//平行移動成分
	std::vector<double3> transAns;
	std::vector<int> ans_nVote;
#if OUTPUT2
	std::vector< std::vector< int > > VotedVP;  //投票されたvpの種類を覚えておく
	std::vector< int > votedvp;
#endif

	//=======================================================================================================//
	//データの読み込み関連
	//=======================================================================================================//
	//法線付きモデルデータの読み込み -> model_cloud
	if (pcl::io::loadPCDFile<pcl::PointNormal> ( model3Dfname, *model_cloud) == -1){ //* load the file 
		fclose( fp_log );
		PCL_ERROR ("Couldn't read file Model cloud. \n");
		return false;
	}

	//vpの読み込み
	char				dum[256];
	char				modelfname[FNL];
	errno_t				error_check;
	fprintf( fp_log,"Loading extracted distinctive vector pairs.( .vp file )" );
	if( (error_check = fopen_s( &fp, vpfname, "r" )) != 0 ){
		fprintf( fp_log,"!!Error %s cannot be load.\n", vpfname );
		fclose( fp_log );
		return false;
	};
	if( fscanf( fp, "%s %s", dum, modelfname ) < 2 ){
		fprintf( fp_log,"%s\n", modelfname );
		fclose( fp_log );
		return false;
	}
	if( fscanf( fp, "%s %lf %lf %lf", dum, &mc.x, &mc.y, &mc.z ) < 4 ){
		fclose( fp_log );
		return false;
	}
	if( fscanf( fp, "%s %f %f %f", dum, &l1, &l2, &theta ) < 4 ){
		fclose( fp_log );
		return false;
	}
	if( fscanf( fp, "%s %f %f %f %f", dum, &s_leaf, &n_neighbor, &cur, &th_flat ) < 5 ){
		fclose( fp_log );
		return false;
	}
	if( fscanf( fp, "%s %d", dum, &Nvp ) < 2 ){
		fclose( fp_log );
		return false;
	}
	if( fscanf( fp, "%s %d", dum, &K_Search ) < 2 ){
		fclose( fp_log );
		return false;
	}
	fprintf( fp_log,"Nvp:%d\n", Nvp );

	mvp = (vector_pair *)malloc( sizeof(vector_pair)*(Nvp) );

	//Vector pair data is cpied to memory
	for( int n=0; n<Nvp; n++ ){
		if( fscanf( fp, "%lf %lf %lf %lf %lf %lf %lf", &mvp[n].p.x, &mvp[n].p.y, &mvp[n].p.z, &mvp[n].np.x, &mvp[n].np.y, &mvp[n].np.z, &mvp[n].ip_p ) < 7 ){
			fclose( fp_log );
			return false;
		} 
		if( fscanf( fp, "%lf %lf %lf %lf %lf %lf %lf", &mvp[n].q1.x, &mvp[n].q1.y, &mvp[n].q1.z, &mvp[n].nq1.x, &mvp[n].nq1.y, &mvp[n].nq1.z, &mvp[n].ip_q1 ) < 7 ){
			fclose( fp_log );
			return false;
		}  
		if( fscanf( fp, "%lf %lf %lf %lf %lf %lf %lf", &mvp[n].q2.x, &mvp[n].q2.y, &mvp[n].q2.z, &mvp[n].nq2.x, &mvp[n].nq2.y, &mvp[n].nq2.z, &mvp[n].ip_q2 ) < 7 ){ 
			fclose( fp_log );
			return false;
		}  
		if( fscanf( fp, "%lf %lf %lf", &mvp[n].nvp.x, &mvp[n].nvp.y, &mvp[n].nvp.z ) < 3 ){
			fclose( fp_log );
			return false;
		}  
		if( fscanf( fp, "%lf %lf %lf", &mvp[n].vc.x, &mvp[n].vc.y, &mvp[n].vc.z ) < 3 ){ 
			fclose( fp_log );
			return false;
		}  
	}
	fclose( fp ) ;

#if OUTPUT1
	//Log: read vector pair 
	fprintf( fp_log,"Nvp:%d, l1:%lf, l2:%lf, theta:%lf\n", Nvp, l1, l2, theta );
	fprintf( fp_log,"mc(%lf,%lf, %lf)\n", mc.x, mc.y, mc.z );
	fprintf( fp_log, "Size of voxel grid: %f\n", s_leaf );
	fprintf( fp_log, "Neighbor distance for normal estimation: %f\n", n_neighbor );
	fprintf( fp_log, "Neighbor distance for calculating curvatuere: %f\n", cur );
	fprintf( fp_log, "Threshold of flat shape: %lf\n", th_flat );
	fprintf( fp_log, "Using K Neighbor search: %d\n", K_Search );
	for( int n=0; n<Nvp; n++ ){
		fprintf( fp_log, "%lf %lf %lf %lf %lf %lf %lf\n", mvp[n].p.x, mvp[n].p.y, mvp[n].p.z, mvp[n].np.x, mvp[n].np.y, mvp[n].np.z, mvp[n].ip_p );  
		fprintf( fp_log, "%lf %lf %lf %lf %lf %lf %lf\n", mvp[n].q1.x, mvp[n].q1.y, mvp[n].q1.z, mvp[n].nq1.x, mvp[n].nq1.y, mvp[n].nq1.z, mvp[n].ip_q1 );  
		fprintf( fp_log, "%lf %lf %lf %lf %lf %lf %lf\n", mvp[n].q2.x, mvp[n].q2.y, mvp[n].q2.z, mvp[n].nq2.x, mvp[n].nq2.y, mvp[n].nq2.z, mvp[n].ip_q2 );  
		fprintf( fp_log, "%lf %lf %lf\n", mvp[n].nvp.x, mvp[n].nvp.y, mvp[n].nvp.z );  
		fprintf( fp_log, "%lf %lf %lf\n\n", mvp[n].vc.x, mvp[n].vc.y, mvp[n].vc.z );  
	}
#endif

	
	std::list<int>::iterator itr;	
	std::vector< std::vector< std::vector < std::list<int> > > > VPHash;
	//ハッシュテーブルの作成 -> VPHash
	//  領域確保(3次元)
	fprintf( fp_log,"Making Hash Table..." );
	VPHash.resize( n_bin );
	for( int k=0 ; k<n_bin ; k++ ){
		VPHash[k].resize( n_bin );
	}
	for( int k=0 ; k<n_bin ; k++ ){
		for( int j=0 ; j<n_bin ; j++ ){
			VPHash[k][j].resize( n_bin );
		}
	}
	//テーブルの各座標にベクトルペアの番号を記録
	for( int i=0 ; i<Nvp ; i++ ){
		key_x = (int)(((mvp[i].ip_p+1.0)/2.0)*(double)n_bin);
		if( key_x == n_bin ) key_x--;
		key_y = (int)(((mvp[i].ip_q1+1.0)/2.0)*(double)n_bin);
		if( key_y == n_bin ) key_y--;
		key_z = (int)(((mvp[i].ip_q2+1.0)/2.0)*(double)n_bin);
		if( key_z == n_bin ) key_z--;
		VPHash[ key_z ][ key_y ][ key_x ].push_back( i );
	}

	//モデルを原点に移動
	pcl::PointNormal hoge;
	hoge.x = hoge.y = hoge.z = 0.0;
	for( int i=0 ; i<(int)model_cloud->points.size() ; i++ ){
		hoge.x += model_cloud->points[i].x;
		hoge.y += model_cloud->points[i].y;
		hoge.z += model_cloud->points[i].z;
	}
	hoge.x /= (float)model_cloud->points.size();
	hoge.y /= (float)model_cloud->points.size();
	hoge.z /= (float)model_cloud->points.size();
	for( int i=0 ; i<(int)model_cloud->points.size() ; i++ ){
		model_cloud->points[i].x -= hoge.x;
		model_cloud->points[i].y -= hoge.y;
		model_cloud->points[i].z -= hoge.z;
	}
	fprintf( fp_log,"hoge:%f, %f, %f\n", hoge.x, hoge.y, hoge.z );
	fprintf( fp_log,"mc  :%lf, %lf, %lf\n", mc.x, mc.y, mc.z );
	for( int nn=0 ; nn<Nvp ; nn++ ){
		mvp[nn].p.x  -= mc.x; mvp[nn].p.y  -= mc.y; mvp[nn].p.z  -= mc.z;
		mvp[nn].q1.x -= mc.x; mvp[nn].q1.y -= mc.y; mvp[nn].q1.z -= mc.z;
		mvp[nn].q2.x -= mc.x; mvp[nn].q2.y -= mc.y; mvp[nn].q2.z -= mc.z;
	}

	//=======================================================================================================//
	//ここから前処理
	//=======================================================================================================//
	//物体モデルからの点のランダムサンプリング（モデルマッチング用）
	int		N_reduced_model, pIdx;
	N_reduced_model = (int)( MODEL_DOWNSAMPLE*(double)model_cloud->points.size() );
	fprintf( fp_log,"Down sampling of model data.\n");
	cloud_model_reduced->width = N_reduced_model;
	cloud_model_reduced->height = 1;
	cloud_model_reduced->is_dense = false;
	cloud_model_reduced->points.resize(cloud_model_reduced->width);
	srand(1);
	for( int i=0 ; i<N_reduced_model ; i++ ){
		pIdx = rand()%model_cloud->points.size();
		cloud_model_reduced->points[i].x = model_cloud->points[pIdx].x;	
		cloud_model_reduced->points[i].y = model_cloud->points[pIdx].y;	
		cloud_model_reduced->points[i].z = model_cloud->points[pIdx].z;	
		cloud_model_reduced->points[i].normal_x = model_cloud->points[pIdx].normal_x;	
		cloud_model_reduced->points[i].normal_y = model_cloud->points[pIdx].normal_y;	
		cloud_model_reduced->points[i].normal_z = model_cloud->points[pIdx].normal_z;	
	}
	fprintf( fp_log," Npoints[%d] -> Npoints[%d]\n", model_cloud->points.size(), cloud_model_reduced->points.size() );
	fflush( fp_log );


	//デバッグ出力にも使う変数
	double	x_low, x_high, y_low, y_high, z_low, z_high;
	struct VotingSpaceTrans		VSTransB;
	struct double3			transResult;
	double					rot3[3][3];
	double					rotAns[3][3] = { {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0} };
	double					rotResult[3][3] = { {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0} };
	int		range_x, range_y, range_z;
	double	pitch_vst;
	int		max_vote_trans;
	transResult.x = transResult.y = transResult.z = 0.0;

	//シーンデータのダウンサンプリング
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	fprintf( fp_log,"Down sampling of scene data.\n" );
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (s_leaf, s_leaf, s_leaf);
	sor.filter (*cloud_filtered);
	AkiTimeLogFromFlag( "Downsampling" );
	AkiTimeSetFlag();
	fprintf( fp_log," Npoints[%d] -> Npoints[%d]\n", cloud->points.size(), cloud_filtered->points.size() );
	fflush( fp_log );
	//20150521 秋月追記 start
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	sor.setInputCloud (cloud_edge);
	sor.filter (*cloud_edge_filtered);
	//20150521 秋月追記 end


	//シーンデータのアウトライアの削除
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clean (new pcl::PointCloud<pcl::PointXYZ>);
	fprintf( fp_log,"Outlier removal of scene data.\n");
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> ssor;
	ssor.setInputCloud( cloud_filtered );
	ssor.setMeanK( 50 );  //近くの50点を使って推定
	ssor.setStddevMulThresh( 1.0 );
	ssor.filter( *cloud_clean );
	for( int i=0 ; i<N_OUTLIER_REMOVAL-1 ; i++ ){
		ssor.setInputCloud( cloud_clean );
		ssor.filter( *cloud_clean );
	}
	//20150521 秋月追記 start
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_edge_clean (new pcl::PointCloud<pcl::PointXYZ>);
	ssor.setInputCloud (cloud_edge_filtered);
	ssor.filter (*cloud_edge_clean);
	for( int i=0 ; i<N_OUTLIER_REMOVAL-1 ; i++ ){
		ssor.setInputCloud( cloud_edge_clean );
		ssor.filter( *cloud_edge_clean );
	}
#if OUTPUT2
	if( CheckSavePCD( cloud_edge ) == true ){
		pcl::io::savePCDFile( "cloud_edge.pcd", *cloud_edge ); 
	}
	if( CheckSavePCD( cloud_edge_clean ) == true ){
		pcl::io::savePCDFile( "cloud_edge_clean.pcd", *cloud_edge_clean ); 
	}
#endif
	//20150521 秋月追記 end

	AkiTimeLogFromFlag( "Removing outlier of an input scene" );
	AkiTimeSetFlag();
	fprintf( fp_log,"%d times\n", N_OUTLIER_REMOVAL );
	fprintf( fp_log," Npoints[%d] -> Npoints[%d]\n", cloud_filtered->points.size(), cloud_clean->points.size() );
	fflush( fp_log );

	// 単一視点の点群データの生成(主に二重になっている計測データの削除)
	pcl::PointCloud<int> visibleIdx;
	double3 viewpoint;
	viewpoint.x = 0.0;
	viewpoint.y = 0.0;
	viewpoint.z = 500.0;
	HPR_operator( cloud_clean, &viewpoint, 2.5, visibleIdx );
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_visible (new pcl::PointCloud<pcl::PointXYZ>);
	CopyPCD( cloud_clean, visibleIdx, cloud_visible );

	AkiTimeLogFromFlag( "Removing hidden points" );
	AkiTimeSetFlag();
	fprintf( fp_log," Npoints[%d] -> Npoints[%d]\n", cloud_clean->points.size(), cloud_visible->points.size() );
	fflush( fp_log );

	//20150521 秋月追記 start
	//エッジ点の結合
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_marge (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_marge->width = cloud_edge_clean->width + cloud_visible->width;
	cloud_marge->height = 1;
	cloud_marge->points.resize( cloud_marge->width*cloud_marge->height );
	int n_copy;
	n_copy = 0;
	for( int i=0 ; i<cloud_edge_clean->points.size() ; i++ ){
		cloud_marge->points[n_copy] = cloud_edge_clean->points[i];
		n_copy++;
	}
	for( int i=0 ; i<cloud_visible->points.size() ; i++ ){
		cloud_marge->points[n_copy] = cloud_visible->points[i];
		n_copy++;
	}
	fprintf( fp_log," Cloud_visible = %d\n", cloud_visible->points.size() );
	fprintf( fp_log," Cloud_edge_clean = %d\n", cloud_edge_clean->points.size() );
	fprintf( fp_log," Cloud_marge = %d\n", cloud_marge->points.size() );

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_marge_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	sor.setInputCloud( cloud_marge );
	sor.filter( *cloud_marge_filtered );
	fprintf( fp_log," Cloud_marge_filtered = %d\n", cloud_marge_filtered->points.size() );


#if OUTPUT2
	if( CheckSavePCD( cloud_marge ) == true ){
		pcl::io::savePCDFile( "cloud_marge.pcd", *cloud_marge ); 
	}
	if( CheckSavePCD( cloud_visible ) == true ){
		pcl::io::savePCDFile( "cloud_visible.pcd", *cloud_visible ); 
	}
#endif
	//20150521 秋月追記 end

	//シーンのポイントクラウドの法線推定（パラメータ：N_NEIGHBORは推定に使うデータ点の範囲）
	pcl::PointCloud<pcl::PointNormal>::Ptr tmp_pnt_nrm (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr pnt_nrm (new pcl::PointCloud<pcl::PointNormal>);
	fprintf( fp_log,"Estimating surface normal vectors of an input scene.\n");
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	fprintf( fp_log," View point( %.2lf, %.2lf, %.2lf )\n", VIEWPOINT_X, VIEWPOINT_Y, VIEWPOINT_Z );
	ne.setViewPoint( VIEWPOINT_X, VIEWPOINT_Y, VIEWPOINT_Z );

	//20150521 秋月追記 start
	//ne.setInputCloud (cloud_visible);
	ne.setInputCloud (cloud_marge_filtered);
	//20150521 秋月追記 end
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (n_neighbor);
	ne.compute (*cloud_normals);

	//20150521 秋月追記 start
	//pcl::concatenateFields (*cloud_visible, *cloud_normals, *tmp_pnt_nrm );
	pcl::concatenateFields (*cloud_marge_filtered, *cloud_normals, *tmp_pnt_nrm );
	//20150521 秋月追記 end

	AkiTimeLogFromFlag( "Estimating normal vectors of an input scene" );
	AkiTimeSetFlag();

	// NaN remove
	std::vector<int> sceneIdx;
	//PCL1.5.1では実装されていない関数
	//pcl::removeNaNNormalsFromPointCloud( *tmp_pnt_nrm, *pnt_nrm, sceneIdx );
	removeNaNNormalsFromPointCloud( *tmp_pnt_nrm, *pnt_nrm, sceneIdx );
	AkiTimeLogFromFlag( "Nan remove" );
	AkiTimeSetFlag();
	
	fprintf( fp_log," Nan remove\n" );
	fprintf( fp_log," Npoints[%d] -> Npoints[%d]\n", tmp_pnt_nrm->points.size(), pnt_nrm->points.size() );


	fprintf( fp_log," Preprocessings have been done.\n" );
	fflush( fp_log );

	//点群の存在範囲を調べる．この範囲内の投票のみ有効とする．
	double3 vr_min, vr_max;
	vr_min.x = vr_min.y = vr_min.z = 9999.9;
	vr_max.x = vr_max.y = vr_max.z = -9999.9;
	for( int i=0 ; i <(int)pnt_nrm->points.size() ; i++ ){
		if( pnt_nrm->points[i].x < vr_min.x ){
			vr_min.x = pnt_nrm->points[i].x;
		}
		if( pnt_nrm->points[i].y < vr_min.y ){
			vr_min.y = pnt_nrm->points[i].y;
		}
		if( pnt_nrm->points[i].z < vr_min.z ){
			vr_min.z = pnt_nrm->points[i].z;
		}
		if( vr_max.x < pnt_nrm->points[i].x ){
			vr_max.x = pnt_nrm->points[i].x;
		}
		if( vr_max.y < pnt_nrm->points[i].y ){
			vr_max.y = pnt_nrm->points[i].y;
		}
		if( vr_max.z < pnt_nrm->points[i].z ){
			vr_max.z = pnt_nrm->points[i].z;
		}
	}

	//曲率値のチェックによる平面除去
	//曲率マップの作成
	fprintf( fp_log," Make curvature map\n" );
	fflush( fp_log );
	char						*cm;	//curvature map
	int							nPoints_clean;
	int							nLargeCurvature = 0;
	nPoints_clean = pnt_nrm->points.size();
	cm = (char *)malloc( sizeof(char)*nPoints_clean );
	cnt = 0;
	for( int i=0 ; i<nPoints_clean; i++ ){
		if( cur < pnt_nrm->points[i].curvature ){
			cm[i] = 1;
			nLargeCurvature++;
		}else{
			cm[i] = 0;
		}
	}
	fprintf( fp_log," Make curvature map... OK.\n" );
	fflush( fp_log );

#if OUTPUT1
	pcl::PointCloud<pcl::PointNormal>::Ptr reduced (new pcl::PointCloud<pcl::PointNormal>);
	//平面除去後のシーンデータ(曲率マップを反映させる)
	reduced->width = nLargeCurvature;
	reduced->height = 1;
	reduced->is_dense = false;
	reduced->points.resize(reduced->width*reduced->height);
	for( int i=0 ; i<nPoints_clean ; i++ ){
		if( cm[i] ){
			reduced->points[cnt] = pnt_nrm->points[i]; 
			cnt++;
		}
	}
	if( CheckSavePCD( reduced ) == true ){
		pcl::io::savePCDFileASCII( "cloud_reduced_scene.pcd", *reduced );

	}
#endif
	AkiTimeLogFromFlag( "Making curvature map" );
	AkiTimeSetFlag();

	//=======================================================================================================//
	//ベクトルペア選択
	//=======================================================================================================//
	//シーンからのベクトルペア選択
	// Creating k-d tree
	fprintf( fp_log,"Extracting vector pairs...\n" );
	fflush( fp_log );
	std::vector< vector_pair > SVP;
	int	allvote;
	//curvature mapを引数とするから点群も平面除去前を渡す
	//SamplingofVectorPairs( pnt_nrm, mc, cm, l1, l2, theta, TH_L, TH_THETA, SVP );
	SamplingofVectorPairs2( pnt_nrm, mc, cm, l1, l2, theta, th_l, th_theta, s_leaf, SVP );


	//2015.05.23 秋月追記 start
	AkiVectorPairRandomReduce( SVP, 2000000/Nvp  );
	//2015.05.23 秋月追記 end

	allvote = SVP.size();

	fprintf( fp_log," The number of extracted vector pair: %d\n", allvote );
	fflush( fp_log );
	AkiTimeLogFromFlag( "Extracting vector pairs from an input scene" );
	AkiTimeSetFlag();


	//=======================================================================================================//
	//投票処理の開始
	//=======================================================================================================//
	//20150521 秋月追記 start
	cv::Mat im_vote_mask;
	im_vote_mask = cv::Mat::zeros( img_cols, img_rows, CV_8U );
	PCDXYZ2CVMat( cloud, pixel_pitch, transZero, 0, &im_vote_mask );
#if OUTPUT2
	cv::imshow( "im_vote_mask マスク", im_vote_mask );
	cv::imwrite( "im_vote_mask.bmp", im_vote_mask );
	cv::waitKey(0);
#endif
	//20150521 秋月追記 end



	fprintf( fp_log, "Voting is started.\n" );
	struct vector_pair		vp_m, *vp_m_ini, vp_i, vp_i_ini;
	struct double3			rot_vc;
	int						nvote;
	int						key_x2, key_y2, key_z2;
	std::vector<float> vote_x;
	std::vector<float> vote_y;
	std::vector<float> vote_z;
	std::vector<float> R11, R12, R13;
	std::vector<float> R21, R22, R23;
	std::vector<float> R31, R32, R33;
	std::vector<int> KIND;
	nvote = 0;
	transResult.x = transResult.y = transResult.z = -1.0;

	vp_m_ini = (vector_pair *)malloc( sizeof(vector_pair)*Nvp );

	//Converting distinctive VP into local coordinate.
	//特徴的VPを正規直交化する -> vp_m_ini[] 
	for( int nn=0 ; nn<Nvp ; nn++ ){
		vp_m.p.x = 0.0; vp_m.p.y = 0.0; vp_m.p.z = 0.0;	
		vp_m.q1.x = mvp[nn].q1.x-mvp[nn].p.x; vp_m.q1.y = mvp[nn].q1.y-mvp[nn].p.y; vp_m.q1.z = mvp[nn].q1.z-mvp[nn].p.z;
		vp_m.q2.x = mvp[nn].q2.x-mvp[nn].p.x; vp_m.q2.y = mvp[nn].q2.y-mvp[nn].p.y; vp_m.q2.z = mvp[nn].q2.z-mvp[nn].p.z;
		vp_m.vc.x = mvp[nn].vc.x; vp_m.vc.y = mvp[nn].vc.y; vp_m.vc.z = mvp[nn].vc.z;
		vp_m.ip_p = mvp[nn].ip_p;
		vp_m.ip_q1 = mvp[nn].ip_q1;
		vp_m.ip_q2 = mvp[nn].ip_q2;
		AkiVectorPairOrthonormalization( &vp_m, &vp_m_ini[nn] );
	}

	AkiTimeLogFromFlag( " Orhonormalizing" );
	AkiTimeSetFlag();
	//投票処理の開始．(key_x, key_y, key_z)がアクセスする座標
	for( int j=0 ; j<allvote ; j++ ){
		//アクセスする座標の決定
		key_x = (int)(((SVP[j].ip_p+1.0)/2.0)*(double)n_bin);
		if( key_x == n_bin ) key_x--;
		key_y = (int)(((SVP[j].ip_q1+1.0)/2.0)*(double)n_bin);
		if( key_y == n_bin ) key_y--;
		key_z = (int)(((SVP[j].ip_q2+1.0)/2.0)*(double)n_bin);
		if( key_z == n_bin ) key_z--;
		//テーブルへのアクセス
		for( int zzz=-1 ; zzz<=1 ; zzz++ ){
			for( int yyy=-1 ; yyy<=1 ; yyy++ ){
				for( int xxx=-1 ; xxx<=1 ; xxx++ ){
					key_x2 = key_x+xxx; key_y2 = key_y+yyy; key_z2 = key_z+zzz;
					//テーブルの内外判定
					if( (0<=key_x2) && (0<=key_y2) &&(0<=key_z2) && (key_x2<n_bin) && (key_y2<n_bin)&& (key_z2<n_bin) ){

						itr = VPHash[ key_z2 ][ key_y2 ][ key_x2 ].begin();
						if( itr != VPHash[ key_z2 ][ key_y2 ][ key_x2 ].end() ){
							//Converting scene vector pair into local coordinate
							//あらかじめ計算しておいて，高速化可能？対応点数による
							vp_i.p.x = 0.0; vp_i.p.y = 0.0; vp_i.p.z = 0.0;
							vp_i.q1.x = SVP[j].q1.x - SVP[j].p.x;
							vp_i.q1.y = SVP[j].q1.y - SVP[j].p.y;
							vp_i.q1.z = SVP[j].q1.z - SVP[j].p.z;
							vp_i.q2.x = SVP[j].q2.x - SVP[j].p.x;
							vp_i.q2.y = SVP[j].q2.y - SVP[j].p.y;
							vp_i.q2.z = SVP[j].q2.z - SVP[j].p.z;
							vp_i.vc.x = vp_i.vc.y = vp_i.vc.z = 0.0;
							AkiVectorPairOrthonormalization( &vp_i, &vp_i_ini );
						}
						//テーブルに登録してあるvpがなくなるまで投票を続ける
						while( itr != VPHash[ key_z2 ][ key_y2 ][ key_x2 ].end() ){
							//シーン－モデルの変換行列の推定
							AkiCalcRotationalMatrixFrom2VectorPairs( &vp_m_ini[*itr], &vp_i_ini, rot3 );

							//投票位置への変位ベクトル
							Aki3DMatrixx3DPoint( rot3, mvp[*itr].vc, &rot_vc );

							rot_vc.x += SVP[j].p.x;
							rot_vc.y += SVP[j].p.y;
							rot_vc.z += SVP[j].p.z;
							
							//投票位置の内外判定
							if( (rot_vc.x < vr_max.x) && (rot_vc.y < vr_max.y) && ( rot_vc.z < vr_max.z) 
							 &&	(vr_min.x < rot_vc.x) && (vr_min.y < rot_vc.y) && ( vr_min.z < rot_vc.z) ){
								 //20150521 秋月追記 start
								 int imgX, imgY, imgZ;
								 imgX = (int)(((rot_vc.x + transZero.x)/pixel_pitch)+0.5);
								 imgY = (int)(((rot_vc.y + transZero.y)/pixel_pitch)+0.5);
								 imgZ = (int)(((rot_vc.z + transZero.z)/pixel_pitch)+0.5);
								 if( imgZ < (int)im_vote_mask.at<uchar>( imgY, imgX ) ){
								 //20150521 秋月追記 end
									vote_x.push_back( (float)rot_vc.x );
									vote_y.push_back( (float)rot_vc.y );
									vote_z.push_back( (float)rot_vc.z );
									R11.push_back( (float)rot3[0][0] );
									R12.push_back( (float)rot3[0][1] );
									R13.push_back( (float)rot3[0][2] );
									R21.push_back( (float)rot3[1][0] );
									R22.push_back( (float)rot3[1][1] );
									R23.push_back( (float)rot3[1][2] );
									R31.push_back( (float)rot3[2][0] );
									R32.push_back( (float)rot3[2][1] );
									R33.push_back( (float)rot3[2][2] );
									KIND.push_back( *itr );
								 }
							}
							itr++;
						}
					}
				}
			}
		}
	}

	fprintf( fp_log, " The number of vote: %d\n", vote_x.size() );
	AkiTimeLogFromFlag( "Voting" );
	AkiTimeSetFlag();

	// 投票回数が少なすぎたらエラーを返却
	if( vote_x.size() == 0 ){
		fclose( fp_log );
		fprintf( SE,"The number of vote is too small.\n" );
		return false;
	}else if( 2000000 < vote_x.size() ){
		//20150502-秋月追記
		//多すぎたらメモリ確保できないようなのでエラーを返却
		fclose( fp_log );
		fprintf( SE,"The number of vote is too large.\n" );
		return false;
	}
	
	//=======================================================================================================//
	//オフセットベクトル(投票結果)をPCDに変換する
	//=======================================================================================================//
	//Copy vote data
	pcl::PointCloud<pcl::PointXYZ>::Ptr vote (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ seed;
	pcl::PointXYZ GoodSeed;

	vote->width = vote_x.size();
	vote->height = 1;
	vote->is_dense = false;
	vote->points.resize(vote->width*vote->height);
	x_low = x_high = vote_x[0];
	y_low = y_high = vote_y[0];
	z_low = z_high = vote_z[0];


	for( int i=0 ; i<(int)vote->points.size() ; i++ ){
		// オフセットベクトルのxyzのそれぞれの最大値，最小値を求める
		if( vote_x[i] < x_low ) x_low = vote_x[i];
		if( vote_y[i] < y_low ) y_low = vote_y[i];
		if( vote_z[i] < z_low ) z_low = vote_z[i];
		if( x_high < vote_x[i] ) x_high = vote_x[i];
		if( y_high < vote_y[i] ) y_high = vote_y[i];
		if( z_high < vote_z[i] ) z_high = vote_z[i];
		// オフセットベクトルのPCDへの変換部分
		vote->points[i].x = vote_x[i];
		vote->points[i].y = vote_y[i];
		vote->points[i].z = vote_z[i];
	}

	//平行移動成分の投票空間の領域確保
	//pitch_vstによって離散化される
	pitch_vst = S_LEAF*PITCH_VOTING_SPACE;
	range_x = (int)((x_high - x_low)/pitch_vst)+1;
	range_y = (int)((y_high - y_low)/pitch_vst)+1;
	range_z = (int)((z_high - z_low)/pitch_vst)+1;

	//オフセットベクトルの存在範囲の出力
	fprintf( fp_log, " Range of vote\n" ); 
	fprintf( fp_log, "  X( %.2lf - %.2lf )\n", x_low, x_high ); 
	fprintf( fp_log, "  Y( %.2lf - %.2lf )\n", y_low, y_high ); 
	fprintf( fp_log, "  Z( %.2lf - %.2lf )\n", z_low, z_high ); 
	fprintf( fp_log, "Range of voting space(Translation)\n" );
	fprintf( fp_log, " [ %d %d %d ] Pitch: %.2lf mm\n", range_x, range_y, range_z, pitch_vst );
	

	//平行移動成分の投票空間の作成
	std::vector< int> TransHist;  //平行移動投票空間の投票数のヒストグラム
	AkiCreateVotingSpaceTrans( range_x, range_y, range_z, Nvp, &VSTransB );
	AkiClearVotingSpaceTrans( &VSTransB );
	TransHist.resize( Nvp+1, 0 );


	//平行移動成分の投票空間（離散化空間）へのオフセットベクトルの投票
	int		vst_x, vst_y, vst_z;
	for( int x=0 ; x<(int)vote->points.size() ; x++ ){
		//投票座標をlowの分だけオフセットかけて投票（マイナスのIDをなくす）
		vst_x = (int)((vote->points[x].x - x_low)/pitch_vst);
		vst_y = (int)((vote->points[x].y - y_low)/pitch_vst);
		vst_z = (int)((vote->points[x].z - z_low)/pitch_vst);
		VSTransB.id[ vst_z ][ vst_y ][ vst_x ].push_back( x );	//得票があった姿勢仮説のIDを投票空間の座標に記録
		VSTransB.kind[ vst_z ][ vst_y ][ vst_x ][ KIND[x] ] = 1;
	}

	//得票数をカウント
	int	vote_cnt;
	for( int k=0 ; k<range_z ; k++ ){
		for( int j=0 ; j<range_y ; j++ ){
			for( int i=0 ; i<range_x ; i++ ){
				vote_cnt = 0;
				for( int x=0 ; x<Nvp ; x++ ){
					if( VSTransB.kind[k][j][i][x] ){
						vote_cnt++; //(i,j,k)に投票されたベクトルペアの種類数
					}
				}
				VSTransB.nVote[k][j][i] = vote_cnt;
				// 投票数を横軸としたヒストグラムを作成
				TransHist[ VSTransB.nVote[k][j][i] ]++;
			}
		}
	}

	int sumTransHist;
	sumTransHist = 0;
	//TransHistの総投票数を算出
	for( int i=0 ; i<(int)TransHist.size() ; i++ ) sumTransHist += TransHist[i]; 


	AkiTimeLogFromFlag( "Counting the number of vote" );
	AkiTimeSetFlag();

#if OUTPUT2
	fprintf( fp_log,"TransHist\n" ); 
	for( int i=0 ; i<(int)TransHist.size() ; i++ ){
		fprintf( fp_log,"%d = %d\n", i, TransHist[i] ); 
	}
	fprintf( fp_log,"Sum voted = %d\n\n", sumTransHist ); 
#endif

	int		th_nVoteTrans;
	double	sum_tx, sum_ty, sum_tz;
	double	sum_rpyX, sum_rpyY, sum_rpyZ;
	double	ave_rpyX, ave_rpyY, ave_rpyZ;
	double3	ave_t;
	double	tmpRot[3][3];
	double3	rotate;
	co3d	max_trans;
	int		nVoteRot;
	int		*kindRot;

	pcl::PointCloud<pcl::PointXYZ>::Ptr rot_vote (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ	tmp_rot;
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float> > rot_vote_kdtree;
	std::vector<int> pIdxRS;
	std::vector<float> pRSD;
	std::vector<int> trans_id;
	float	radius;
	int		nVaridPoseTrans, nNear, MaxNear, nKindRot, GoodnKindRot;
	int		th_nVoteRot;
	th_nVoteTrans = 1;


	kindRot = (int *)malloc( sizeof(int)*Nvp );
	for( int nn=0 ;nn<Nvp ; nn++ ) kindRot[nn] = 0;

	radius = (float)(6.0*deg2rad);  //+-6[deg]の姿勢を取り扱う
	pcl::PointCloud<pcl::PointXYZ>::Ptr hogehoge (new pcl::PointCloud<pcl::PointXYZ>);
	//th_nVoteTransはTransHistから算出する
	int current_vote, th_TransHist;
	th_TransHist = (int)(RATE_TRANS_VOTE*(double)sumTransHist);
	//この閾値は個数を限定しても良いかもしれない．（検出されうる物体数）
	current_vote = 0;
	for( int i=Nvp ; 0<i ; i-- ){
		current_vote += TransHist[i];
		if( th_TransHist < current_vote ){
			th_nVoteTrans = i;
			break;
		}
	}
	fprintf( fp_log,"Rate of TransHist:%.3lf\n", RATE_TRANS_VOTE );
	fflush( fp_log );


	th_nVoteRot = TH_NVOTE_ROT; //USのときで，nVP200
	max_vote_trans = 0;
	nVaridPoseTrans = 0;
	fprintf( fp_log, "th_nVoteTrans: %d\n", th_nVoteTrans );
	fprintf( fp_log, "th_nVoteRot: %d\n", th_nVoteRot );
	for( int k=0 ; k<range_z ; k++ ){
		for( int j=0 ; j<range_y ; j++ ){
			for( int i=0 ; i<range_x ; i++ ){
				//3.平行移動成分の得票数がしきい値以上だったら
				if( th_nVoteTrans <= VSTransB.nVote[k][j][i] ){
					//有効な姿勢数をカウント
					nVaridPoseTrans++;  //平行移動空間で閾値以上の座標数をカウント
					//最大の得票を調べる
					if( max_vote_trans < VSTransB.nVote[k][j][i] ){
						max_vote_trans = VSTransB.nVote[k][j][i];  //最大の得票数
						max_trans.x = i;
						max_trans.y = j;
						max_trans.z = k;
					}
					rot_vote->points.clear();
					trans_id.clear();
					//4.回転成分の空間でのクラスタリング
					itr = VSTransB.id[k][j][i].begin();
					while( itr != VSTransB.id[k][j][i].end() ){
						//回転成分の空間への投票（voteのIDを記憶・得票数の記憶）
						tmpRot[0][0] = R11[ *itr ]; tmpRot[0][1] = R12[ *itr ]; tmpRot[0][2] = R13[ *itr ];
						tmpRot[1][0] = R21[ *itr ]; tmpRot[1][1] = R22[ *itr ]; tmpRot[1][2] = R23[ *itr ];
						tmpRot[2][0] = R31[ *itr ]; tmpRot[2][1] = R32[ *itr ]; tmpRot[2][2] = R33[ *itr ];
						AkiRot2RollPitchYaw( tmpRot, &rotate.x, &rotate.y, &rotate.z );
						tmp_rot.x = (float)rotate.x; tmp_rot.y = (float)rotate.y; tmp_rot.z = (float)rotate.z;
						rot_vote->points.push_back( tmp_rot );  //回転成分(RollPitchYaw)を投票
						//kd-treeにデータを送るとvoteにおける何番目の姿勢仮説か分からなくなるので，この変数に保存しとく
						trans_id.push_back( *itr );  //回転空間に投票された姿勢仮説のIDを保存
						itr++;
					}

					rot_vote_kdtree.setInputCloud( rot_vote );  //回転空間をkd-tree化
					pIdxRS.clear();
					pRSD.clear();
					nVoteRot = 0;
					MaxNear = 0;

					for( int n=0 ; n<(int)trans_id.size() ; n++ ){
						seed = rot_vote->points[ n ]; //各投票データがseed点

						nNear = rot_vote_kdtree.radiusSearch( seed, radius, pIdxRS, pRSD );	
						nKindRot = 0;
						//何種類のベクトルペアが投票されたかカウント
						for( int nn=0 ; nn<Nvp ; nn++ ) kindRot[nn] = 0;
						for( int nn=0 ; nn<nNear ; nn++ ){
							kindRot[ KIND[trans_id[pIdxRS[nn]]] ] = 1;
						}
						for( int nn=0 ; nn<Nvp ; nn++ ){
							nKindRot += kindRot[nn];
						}
						if( MaxNear < nNear ){ //最大得票を得たseed点を記憶
							MaxNear = nNear;
							GoodSeed = seed;
							GoodnKindRot = nKindRot;
						}
					}

					//しきい値以上の投票を得ていたら平均化する．
					if ( th_nVoteRot < GoodnKindRot ){
						nNear = rot_vote_kdtree.radiusSearch( GoodSeed, radius, pIdxRS, pRSD );	
						//fprintf( fp_log,"nearest: %d\n", rot_vote_kdtree.radiusSearch( seed, radius, pIdxRS, pRSD ));
						nVoteRot = nKindRot; //最大のインライア数を更新する．これによって，最も投票数の多いものだけが保存される．
						sum_rpyX = sum_rpyY = sum_rpyZ = 0.0;
						sum_tx = sum_ty = sum_tz = 0.0;
#if OUTPUT2
						votedvp.clear();
#endif
						for( int xx=0 ; xx<(int)pIdxRS.size() ; xx++ ){

							sum_rpyX += rot_vote->points[pIdxRS[xx]].x; sum_rpyY += rot_vote->points[pIdxRS[xx]].y; sum_rpyZ += rot_vote->points[pIdxRS[xx]].z;
							sum_tx += vote_x[ trans_id[pIdxRS[xx]] ]; sum_ty += vote_y[ trans_id[pIdxRS[xx]] ]; sum_tz += vote_z[ trans_id[pIdxRS[xx]] ];
#if OUTPUT2
							//fprintf( fp_log,"%5.2lf;%5.2lf;%5.2lf;%5.2lf;%5.2lf;%5.2lf\n", rot_vote->points[pIdxRS[xx]].x*rad2deg, rot_vote->points[pIdxRS[xx]].y*rad2deg, rot_vote->points[pIdxRS[xx]].z*rad2deg
							//	, vote_x[ trans_id[pIdxRS[xx]] ], vote_y[ trans_id[pIdxRS[xx]] ], vote_z[ trans_id[pIdxRS[xx]] ] );
							votedvp.push_back( KIND[ trans_id[pIdxRS[xx]] ] );
#endif

						}
						//ロールピッチヨー，平行移動成分を平均化
						ave_rpyX = sum_rpyX/(double)pIdxRS.size(); ave_rpyY = sum_rpyY/(double)pIdxRS.size(); ave_rpyZ = sum_rpyZ/(double)pIdxRS.size();
						ave_t.x = sum_tx/(double)pIdxRS.size(); ave_t.y = sum_ty/(double)pIdxRS.size(); ave_t.z = sum_tz/(double)pIdxRS.size();
						//fprintf( fp_log,"%d Averaged Roll Pitch Yaw XYZ(%5.2lf, %5.2lf, %5.2lf)\n", pIdxRS.size(), ave_rpyX*rad2deg, ave_rpyY*rad2deg, ave_rpyZ*rad2deg );
						AkiRollPitchYaw2Rot( ave_rpyX, ave_rpyY, ave_rpyZ, tmpRot );

						rotAns11.push_back( tmpRot[0][0] ); rotAns12.push_back( tmpRot[0][1] ); rotAns13.push_back( tmpRot[0][2] );
						rotAns21.push_back( tmpRot[1][0] ); rotAns22.push_back( tmpRot[1][1] ); rotAns23.push_back( tmpRot[1][2] );
						rotAns31.push_back( tmpRot[2][0] ); rotAns32.push_back( tmpRot[2][1] ); rotAns33.push_back( tmpRot[2][2] );
						transAns.push_back( ave_t );
						ans_nVote.push_back( GoodnKindRot );
#if OUTPUT2
						VotedVP.push_back( votedvp );
#endif
					}

				}
			}
		}
	}
	fprintf( fp_log, "nVaridPoseTrans: %d\n", nVaridPoseTrans );
	fprintf( fp_log,"The number of candidate pose hypothesis = %d\n", rotAns11.size() );
	if( rotAns11.size() == 0 ){
		fclose( fp_log );
		fprintf( fp_log,"The number of vote is too small.\n" );
		return false;
	}
	fprintf( fp_log,"max_vote_trans = %d\n", max_vote_trans );
	fprintf( fp_log,"(%d, %d, %d)\n", max_trans.x, max_trans.y, max_trans.z );


	AkiTimeLogFromFlag( "Detecting good pose hypothesis by the VPM" );
	AkiTimeSetFlag();

	//Transform of object model.
	pcl::PointCloud<pcl::PointNormal>::Ptr TransModel (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr Inliers_model (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr Inliers_scene (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr pntRot (new pcl::PointCloud<pcl::PointNormal>);
#if OUTPUT2
	pcl::PointCloud<pcl::PointNormal>::Ptr NearestPoints (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr ValidNearestPoints (new pcl::PointCloud<pcl::PointNormal>);
#endif
	pntRot->width = cloud_model_reduced->points.size();
	pntRot->height = 1;
	pntRot->is_dense = false;
	pntRot->points.resize(pntRot->width);
	TransModel->width = cloud_model_reduced->points.size();
	TransModel->height = 1;
	TransModel->is_dense = false;
	TransModel->points.resize(TransModel->width);
	
	//	Creating k-d tree
	pcl::KdTreeFLANN<pcl::PointNormal, flann::L2_Simple<float> > s_kdtree;
	s_kdtree.setInputCloud( pnt_nrm );
	std::vector<int> pIdxNKNS;
	std::vector<float> pNKNSd;
	int K = 1; 
	int	Ngc;


	double		ip_vp_pnt;
	double3		roted_pnt_nrm, nrm_scene;
	double		nrm_error;
	int			nVoteResult;
	int			result_id; //rotAnsのいくつめの要素が最も誤差が少なかったかを記録
	double3		mu_s, mu_m;  //Scene, Modelの平均位置ベクトル		
	int			n_inlier, break_flag, n_outlier;
	double		score, max_score, nrm_sum;
	double		dist_outlier;
	double		in_score, out_score, rate_score;
	double		ramda;

	ramda = 0.02;
	result_id = -1;
	nVoteResult = -1;
	max_score = 0.0;
	break_flag = 0;
	//Checking geometric consistency
	Ngc = 0;
	viewpoint.x = viewpoint.y = 0.0;
	viewpoint.z = 500.0;
	for( int j=0 ; j<(int)rotAns11.size() ; j++ ){
		// 姿勢仮説
		rotAns[0][0] = rotAns11[j]; rotAns[0][1] = rotAns12[j]; rotAns[0][2] = rotAns13[j];
		rotAns[1][0] = rotAns21[j]; rotAns[1][1] = rotAns22[j]; rotAns[1][2] = rotAns23[j];
		rotAns[2][0] = rotAns31[j]; rotAns[2][1] = rotAns32[j]; rotAns[2][2] = rotAns33[j];

		//物体モデルへの姿勢仮説の適用
		CloudTransformPntNrm( cloud_model_reduced, rotAns, transAns[j], pntRot ); 
		//インライア数のカウント，記憶
		score = nrm_sum = 0.0;
		n_inlier = n_outlier = 0;
		dist_outlier = 0.0;
		for( int i=0 ; i<(int)pntRot->points.size() ; i++ ){
			roted_pnt_nrm.x = pntRot->points[ i ].normal_x;
			roted_pnt_nrm.y = pntRot->points[ i ].normal_y;
			roted_pnt_nrm.z = pntRot->points[ i ].normal_z;
			AkiInnerProduct( vp_dir, roted_pnt_nrm, &ip_vp_pnt );
			// 見えるはずの点を取り出す．（法線がセンサに向いているかどうかチェック）
			if( FRONT < ip_vp_pnt ){
				if ( s_kdtree.nearestKSearch( pntRot->points[i], K, pIdxNKNS, pNKNSd) > 0 ){
					if( sqrt( pNKNSd[ 0 ] ) < s_leaf*2.0 ){
						Inliers_model->points.push_back( pntRot->points[i] );
						Inliers_scene->points.push_back( pnt_nrm->points[ pIdxNKNS[0] ] );
#if OUTPUT2
						NearestPoints->points.push_back( pntRot->points[i] );
#endif
					}else{
						n_outlier++;
						dist_outlier += 1.0 - exp( -ramda * sqrt(pNKNSd[0]) );
					}
				}
			}
		}
		n_inlier = Inliers_model->points.size(); //インライア数
		//Inlier点数が少なすぎる場合は棄却
		if( n_inlier < (int)(TH_N_INLIER*(double)cloud_model_reduced->points.size()) ){
			//fprintf( fp_log,"N_INLIER:%d, total model points:%d\n", n_inlier, cloud_model_reduced->points.size() );

		}else{ //Inlier点数が十分なときは，スコアを計算する．
			//fprintf( fp_log,"No.%d/%d n_inlier:%d\n", j, rotAns11.size(), n_inlier );
			for( int i=0 ; i<n_inlier ; i++ ){
				//法線内積の計算
				roted_pnt_nrm.x = Inliers_model->points[i].normal_x;
				roted_pnt_nrm.y = Inliers_model->points[i].normal_y;
				roted_pnt_nrm.z = Inliers_model->points[i].normal_z;
				nrm_scene.x = Inliers_scene->points[i].normal_x;
				nrm_scene.y = Inliers_scene->points[i].normal_y;
				nrm_scene.z = Inliers_scene->points[i].normal_z;
				AkiInnerProduct( roted_pnt_nrm, nrm_scene, &nrm_error );

				nrm_sum += nrm_error; //内積総和
			}

			in_score = nrm_sum/(double)n_inlier;
			out_score = 1.0 - dist_outlier/(double)n_outlier;
			rate_score = (double)n_inlier / (double)(n_inlier+n_outlier);
			score = ( in_score + out_score + rate_score ) / 3.0;  //APCノートの2015.03.12
#if OUTPUT1
			// 姿勢仮説の可視化
			if( max_score < score ){
				cv::Mat hyp_img;
				hyp_img = cv::Mat::zeros( img_cols, img_rows, CV_8UC3 );
				PCD2CVMatHypothesis( cloud, pntRot, pixel_pitch, transZero, hyp_img );
				char hyp_name[FNL];
				sprintf( hyp_name,"pose_hyp_Score%.2lf_ID%d_IN%.2lf_OUT%.2lf_RATE%.2lf.bmp", score, j, in_score, out_score, rate_score );
				cv::imwrite( hyp_name, hyp_img );
			}
#endif

			// スコアの最大値の更新
			if( max_score < score ){
#if OUTPUT2
				fprintf( fp_log,"UPDATE:\n" );
				fprintf( fp_log,"max_score: %lf -> %lf\n", max_score, score );
				fprintf( fp_log,"score_rate: %lf\n", (double)(score/(double)n_inlier) );
#endif
				result_id = j; //このIDの姿勢仮説（平均化後）が最も有望
				max_score = score;

				//位置の微修正のための処理，モデルと対応点の重心点を一致させる
				mu_s.x = mu_s.y = mu_s.z = 0.0;
				mu_m.x = mu_m.y = mu_m.z = 0.0;
				for( int i=0 ; i<n_inlier ; i++ ){
					//クエリと対応点の平均位置ベクトルを計算
					mu_m.x += (double)Inliers_model->points[i].x;
					mu_m.y += (double)Inliers_model->points[i].y;
					mu_m.z += (double)Inliers_model->points[i].z;
					mu_s.x += (double)Inliers_scene->points[i].x;
					mu_s.y += (double)Inliers_scene->points[i].y;
					mu_s.z += (double)Inliers_scene->points[i].z;
				}
				mu_m.x /= n_inlier; mu_m.y /= n_inlier; mu_m.z /= n_inlier;
				mu_s.x /= n_inlier; mu_s.y /= n_inlier; mu_s.z /= n_inlier;


				rotResult[0][0] = rotAns[0][0]; rotResult[0][1] = rotAns[0][1]; rotResult[0][2] = rotAns[0][2];
				rotResult[1][0] = rotAns[1][0]; rotResult[1][1] = rotAns[1][1]; rotResult[1][2] = rotAns[1][2];
				rotResult[2][0] = rotAns[2][0]; rotResult[2][1] = rotAns[2][1]; rotResult[2][2] = rotAns[2][2];
				transResult.x = transAns[j].x + ( mu_s.x-mu_m.x );
				transResult.y = transAns[j].y + ( mu_s.y-mu_m.y );
				transResult.z = transAns[j].z + ( mu_s.z-mu_m.z );
				nVoteResult = ans_nVote[j];
#if OUTPUT2
				ValidNearestPoints->points.clear();
				for( int i=0 ; i<(int)NearestPoints->points.size() ; i++ ){
					ValidNearestPoints->points.push_back( NearestPoints->points[i] );
				}
#endif

			}
		}
#if OUTPUT2
		NearestPoints->points.clear();
#endif
		Ngc++;
		Inliers_model->points.clear();
		Inliers_scene->points.clear();
		n_inlier = 0;
		break_flag = 0;
	}		
	fprintf( fp_log, "The nuber of checked pose: %d\n", Ngc ); 
	AkiTimeLogFromFlag("Checking geometric consistency of candidate poses.");
	AkiTimeSetFlag();

	if( max_score <= TH_Final_VPM){
		fclose( fp_log );
		fprintf( fp_log,"Final socre is too low.\n" );
		return false;
	}



	// シーンにおける物体クラスタの抽出
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr PntResult (new pcl::PointCloud<pcl::PointNormal>);
	PntResult->width = model_cloud->size();
	PntResult->height = 1;
	PntResult->is_dense = false;
	PntResult->points.resize(PntResult->width);
	CloudTransformPntNrm( model_cloud, rotResult, transResult, PntResult );


	////20150428-秋月追記↓↓↓
	//pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	//pcl::PointCloud<pcl::PointNormal>::Ptr PntResult2 (new pcl::PointCloud<pcl::PointNormal>);
	//// Set the input source and target
	//icp.setInputCloud (PntResult);
	//icp.setInputTarget (pnt_nrm);

	//// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	//icp.setMaxCorrespondenceDistance (S_LEAF);
	//// Set the maximum number of iterations (criterion 1)
	//icp.setMaximumIterations (50);
	//// Set the transformation epsilon (criterion 2)
	//icp.setTransformationEpsilon (1e-8);
	//// Set the euclidean distance difference epsilon (criterion 3)
	//icp.setEuclideanFitnessEpsilon (1);

	//// Perform the alignment
	//icp.align (*PntResult2);

	//// Obtain the transformation that aligned cloud_source to cloud_source_registered
	//Eigen::Matrix4f transformation = icp.getFinalTransformation ();
	////20150428-秋月追記↑↑↑

	//クラスタの抽出
	//CloudSubtractionInv( cloud, PntResult2, S_LEAF, cloud_cluster, cloud_cluster_idx );
	CloudSubtractionInv( cloud, PntResult, 5.0, cloud_cluster, cloud_cluster_idx );

	// クラスタの重心点算出
	pcl::PointXYZ CenterOfCluster;
	CalcCenter( cloud_cluster, &CenterOfCluster );
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float> > c_kdtree;
	c_kdtree.setInputCloud( cloud );
	std::vector<int> c_pIdx;
	std::vector<float> c_pSd;
	c_pIdx.resize(1);
	c_pSd.resize(1);
	c_pIdx[0] = 0;
	if ( c_kdtree.nearestKSearch( CenterOfCluster, 1, c_pIdx, c_pSd) > 0 ){
		*CenterIdx = c_pIdx[0];
	}

	AkiTimeLogFromFlag("Cluster extraction.");



	time_t time_s2e;
	AkiTimeSetFlag();
	time_s2e = AkiTimeFromStart();
#if OUTPUT2
	fopen_s( &fp, "log_VPM_time.txt", "a" );
	fprintf( fp,"processing_time: %d\n", time_s2e );
	fclose( fp );
#endif
	//検出できなかったときのフラグ
	int NoDetect;
	NoDetect = 0;
	if( result_id == -1 ){
		fprintf( fp_log,"No detect.\n");
		NoDetect = 1;
	}else{
		fprintf( fp_log, "The result of VPM method is ...\n" ); 
		fprintf( fp_log, "voted kind: %d\n", nVoteResult ); 
		fprintf( fp_log, "hypthesis id: %d\n", result_id ); 
		fprintf( fp_log,"    | %.3lf %.3lf %.3lf |\n", rotResult[0][0], rotResult[0][1], rotResult[0][2] );
		fprintf( fp_log,"R = | %.3lf %.3lf %.3lf |\n", rotResult[1][0], rotResult[1][1], rotResult[1][2] );
		fprintf( fp_log,"    | %.3lf %.3lf %.3lf |\n", rotResult[2][0], rotResult[2][1], rotResult[2][2] );
		fprintf( fp_log,"t = < %.3lf %.3lf %.3lf >\n", transResult.x, transResult.y, transResult.z );
		fprintf( fp_log,"   score = <%.3lf>\n", max_score ); 
		fprintf( fp_log,"\n" ); 
	}

	// 認識結果
	VPMworkPos[0] = rotResult[0][0];
	VPMworkPos[1] = rotResult[0][1];
	VPMworkPos[2] = rotResult[0][2];
	VPMworkPos[3] = rotResult[1][0];
	VPMworkPos[4] = rotResult[1][1];
	VPMworkPos[5] = rotResult[1][2];
	VPMworkPos[6] = rotResult[2][0];
	VPMworkPos[7] = rotResult[2][1];
	VPMworkPos[8] = rotResult[2][2];
	VPMworkPos[9] = transResult.x;
	VPMworkPos[10] = transResult.y;
	VPMworkPos[11] = transResult.z;
	*VPMScore = max_score;



#if OUTPUT1
	fopen_s( &fp, "log_VPM_voted_kind.txt", "a" );
	fprintf( fp,"voted_kind: %d\n", nVoteResult );
	fclose( fp );

	fopen_s( &fp, "log_VPM_result.txt", "a" );
	fprintf( fp,"%lf\n", max_score );
	fclose( fp );
#endif

#if OUTPUT2
	//答えが出なかったときは初期のモデル点を保存
	if( NoDetect ){
		if( CheckSavePCD( model_cloud ) == true ){
			sprintf( name,"ans.pcd" );
			pcl::io::savePCDFileASCII( name, *model_cloud );
		}
	}else{
		//変換した物体モデルの保存
		if( CheckSavePCD( model_cloud ) == true ){
			pcl::PointCloud<pcl::PointNormal>::Ptr PntResult (new pcl::PointCloud<pcl::PointNormal>);
			PntResult->width = model_cloud->size();
			PntResult->height = 1;
			PntResult->is_dense = false;
			PntResult->points.resize(PntResult->width);
			CloudTransformPntNrm( model_cloud, rotResult, transResult, PntResult );
			sprintf( name,"ans.pcd" );
			pcl::io::savePCDFileASCII( name, *PntResult );
		}

		cv::Mat ans_hyp_img, ans_hyp_img2;
		ans_hyp_img = cv::Mat::zeros( img_cols, img_rows, CV_8UC3 );
		PCD2CVMatHypothesis( cloud, PntResult, pixel_pitch, transZero, ans_hyp_img );
		char hyp_name[FNL];
		sprintf( hyp_name,"img_ans.bmp" );
		cv::flip( ans_hyp_img, ans_hyp_img2, 0 );
		cv::imwrite( hyp_name, ans_hyp_img );
	}
#endif

#if OUTPUT1
	//if( CheckSavePCD( PntResult2 ) == true ){
	//	fprintf( fp_log,"Save cloud_filtered_scene.pcd.\n");
	//	pcl::io::savePCDFileASCII( "ans2.pcd", *PntResult2 );
	//}

	//中間データの出力 .pcd
	if( CheckSavePCD( cloud_filtered ) == true ){
		fprintf( fp_log,"Save cloud_filtered_scene.pcd.\n");
		pcl::io::savePCDFileASCII( "cloud_filtered_scene_vpm.pcd", *cloud_filtered );
	}
	if( CheckSavePCD( cloud_clean ) == true ){
		fprintf( fp_log,"Save cloud_clean_scene.pcd.\n");
		pcl::io::savePCDFileASCII( "cloud_clean_scene_vpm.pcd", *cloud_clean );
	}
	if( CheckSavePCD( pnt_nrm ) == true ){
		fprintf( fp_log,"Save cloud_preprocessed_scene.pcd.\n");
		pcl::io::savePCDFileASCII( "cloud_preprocessed_scene_vpm.pcd", *pnt_nrm );
	}
	if( CheckSavePCD( cloud ) == true ){
		fprintf( fp_log,"Save input_scene.pcd.\n");
		pcl::io::savePCDFileASCII( "input_scene_vpm.pcd", *cloud );
	}
	if( CheckSavePCD( cloud_visible ) == true ){
		sprintf( name,"cloud_visible_vpm.pcd" );
		pcl::io::savePCDFile( name, *cloud_visible ); 
	}
	if( !NoDetect ){
		if( CheckSavePCD( cloud_cluster ) == true ){
			sprintf( name,"cloud_item_cluster_vpm.pcd" );
			pcl::io::savePCDFile( name, *cloud_cluster ); 
		}
	}
#endif

#if OUTPUT2

	// 物体モデルの重畳画像の作成
	if( NoDetect ){ //答えが得られなかったら真っ黒画像を出力
		char hyp_name[FNL];
		cv::Mat ans_hyp_img;
		ans_hyp_img = cv::Mat::zeros( img_cols, img_rows, CV_8UC3 );
		sprintf( hyp_name,"img_vpm_result_ans.bmp" );
		cv::imwrite( hyp_name, ans_hyp_img );

	}else{ 
		// 物体モデルの回転
		if( CheckSavePCD( model_cloud ) == true ){
			pcl::PointCloud<pcl::PointNormal>::Ptr PntResult (new pcl::PointCloud<pcl::PointNormal>);
			PntResult->width = model_cloud->size();
			PntResult->height = 1;
			PntResult->is_dense = false;
			PntResult->points.resize(PntResult->width);
			CloudTransformPntNrm( model_cloud, rotResult, transResult, PntResult );
		}

		cv::Mat ans_hyp_img, ans_hyp_img2;
		ans_hyp_img = cv::Mat::zeros( img_cols, img_rows, CV_8UC3 );
		PCD2CVMatHypothesis( cloud, PntResult, pixel_pitch, transZero, ans_hyp_img );
		char hyp_name[FNL];
		sprintf( hyp_name,"img_vpm_result_ans.bmp" );
		cv::flip( ans_hyp_img, ans_hyp_img2, 0 );
		cv::imwrite( hyp_name, ans_hyp_img2 );
	}


	//中間データの出力 .bmp
	fprintf( fp_log,"Save an input image as img_scene.bmp.\n");
	cv::Mat scene_img, scene_img2;
	scene_img = cv::Mat::zeros( img_cols, img_rows, CV_8U );
	PCDXYZ2CVMat( cloud, pixel_pitch, transZero, 0, &scene_img );
	cv::flip( scene_img, scene_img2, 0 );
	cv::imwrite( "img_vpm1_input.bmp", scene_img2 );
	
	fprintf( fp_log,"Save a preprocessed iamge as img_clean.bmp.\n");
	cv::Mat cloud_clean_img, cloud_clean_img2;
	cloud_clean_img = cv::Mat::zeros( img_cols, img_rows, CV_8U );
	PCDXYZ2CVMat( cloud_clean, pixel_pitch, transZero, 0, &cloud_clean_img );
	cv::flip( cloud_clean_img, cloud_clean_img2, 0 );
	cv::imwrite( "img_vpm2_noise_reduction.bmp", cloud_clean_img2 );
	
	fprintf( fp_log,"Save a visible point iamge as img_visible_scene.bmp.\n");
	cv::Mat visible_img, visible_img2;
	visible_img = cv::Mat::zeros( img_cols, img_rows, CV_8U );
	PCDXYZ2CVMat( cloud_visible, pixel_pitch, transZero, 0, &visible_img );
	cv::flip( visible_img, visible_img2, 0 );
	cv::imwrite( "img_vpm3_HPR.bmp", visible_img2 );


	fprintf( fp_log,"Save a cloud_edge_clean point iamge as img_cloud_edge_clean_scene.bmp.\n");
	cv::Mat edge_clean_img, edge_clean_img2;
	edge_clean_img = cv::Mat::zeros( img_cols, img_rows, CV_8U );
	PCDXYZ2CVMat( cloud_edge_clean, pixel_pitch, transZero, 0, &edge_clean_img );
	cv::flip( edge_clean_img, edge_clean_img2, 0 );
	cv::imwrite( "img_vpm4_edge_clean.bmp", edge_clean_img2 );

	fprintf( fp_log,"Save a marge point iamge as img_marge_scene.bmp.\n");
	cv::Mat marge_img, marge_img2;
	marge_img = cv::Mat::zeros( img_cols, img_rows, CV_8U );
	PCDXYZ2CVMat( cloud_marge, pixel_pitch, transZero, 0, &marge_img );
	cv::flip( marge_img, marge_img2, 0 );
	cv::imwrite( "img_vpm5_marge.bmp", marge_img2 );

	//スコアマップの保存
	fprintf( fp_log,"Save a scoremap iamge as scoremap.bmp.\n");
	cv::Mat score_map, score_map2;
	score_map = cv::Mat::zeros( img_cols, img_rows, CV_8UC3 );
	PCD2CVMatScoreMap( vote, pixel_pitch, transZero, 0, scene_img, score_map );
	cv::flip( score_map, score_map2, 0 );
	cv::imwrite( "img_vpm_result_s-map.bmp", score_map2 );

	//重心位置にマークした画像の保存
	fprintf( fp_log,"Save a marked iamge as scoremap_centroid.bmp.\n");
	cv::Mat centroid_img, centroid_img2;
	centroid_img = cv::Mat::zeros( img_cols, img_rows, CV_8UC3 );
	if( !NoDetect ){
		double3 center_d3;
		center_d3.x = cloud->points[c_pIdx[0]].x;
		center_d3.y = cloud->points[c_pIdx[0]].y;
		center_d3.z = cloud->points[c_pIdx[0]].z;
		PCD2CVMatDrawCentroid( center_d3, pixel_pitch, transZero, scene_img, &centroid_img );
		cv::flip( centroid_img, centroid_img2, 0 );
		char	score_c[FNL];
		sprintf( score_c,"score: %.3lf", max_score );
		cv::putText( centroid_img2, score_c, cv::Point( 30, 30 ), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,0), 2, CV_AA);
	}	
	cv::imwrite( "img_vpm_result_s-map_centroid.bmp", centroid_img2 );

	//インライアの点群の保存
	//if( !NoDetect ){
	//	ValidNearestPoints->width = ValidNearestPoints->points.size();
	//	ValidNearestPoints->height = 1;
	//	ValidNearestPoints->is_dense = false;
	//	
	//	if( CheckSavePCD( ValidNearestPoints ) == true ){
	//		fprintf(fp_log,"Size of valid points: %d\n", ValidNearestPoints->width );
	//		pcl::io::savePCDFileASCII( "nearest_points_vpm.pcd", *ValidNearestPoints );
	//	}
	//}
#endif

	// メモリ開放
	fprintf( fp_log,"free\n" );
	free( kindRot );
	free( vp_m_ini );
	free( cm );
	free( mvp );

	fclose( fp_log );

	return true;
}

