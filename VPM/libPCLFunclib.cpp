//////////////////////////////////////////////////////////////////////////////////
//
//	libPCLFunclib.cpp
//
//	Note:
//		2013.02.12
//			CalcShapeIndex()
//				Caluculating Shape index and Curvedness from point cloud.
//			RemoveFlatPoints()
//				Removing point cloud which referes flat shapes by threshold about
//				curvedness.
//			SavePointRGB()
//				To save PointRGB data.
//		2014.01.25
//          SamplingofVectorPairs()
//				引数にTH_L, TH_THETAを追加．（今までは大域変数だった．）
//				vp選択時とマッチング時に同じパラメータになるように注意すること．
//
//////////////////////////////////////////////////////////////////////////////////
//#include "stdafx.h"

#include "libPCLFunclib.h"
#include "Labeling.h"

#define dbgflg (0) //デバッグ用．中間画像の可視化


void RemoveFlatPoints( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr si, 
					   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cc,
					   float TH_FLAT,
					   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out ){

	int	cnt;
	cnt = 0;
	for( size_t i=0 ; i<cc->points.size() ; i++ ){
		if( TH_FLAT < cc->points[i].rgb ){
			cnt++;
		}
	}

	cloud_out->width = cnt;
	cloud_out->height = 1;
	cloud_out->is_dense = false;
	cloud_out->points.resize(cnt);
	cnt = 0;
	for( size_t i=0 ; i<cc->points.size() ; i++ ){
		if( TH_FLAT < cc->points[i].rgb ){
			cloud_out->points[cnt].x = si->points[i].x;
			cloud_out->points[cnt].y = si->points[i].y;
			cloud_out->points[cnt].z = si->points[i].z;
			cloud_out->points[cnt].rgb = si->points[i].rgb;
			cnt++;
		}
	}

}

void SavePointRGB( char ofname[], const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ){

	FILE	*fp;
	int		rgb;
	int		r, g, b;

	fp = fopen( ofname, "w" );
	fprintf( fp,"# .PCD v0.7 - Point Cloud Data file format\n");
	fprintf( fp,"VERSION 0.7\n");
	fprintf( fp,"FIELDS x y z rgb\n");
	fprintf( fp,"SIZE 4 4 4 4\n");
	fprintf( fp,"TYPE F F F I\n");
	fprintf( fp,"COUNT 1 1 1 1\n");
	fprintf( fp,"WIDTH %d\n", (int)cloud->points.size() );
	fprintf( fp,"VIEWPOINT 0 0 0 1 0 0 0\n");
	fprintf( fp,"POINTS %d\n", (int)cloud->points.size() );
	fprintf( fp,"DATA ascii\n");
	for( size_t i=0 ; i<cloud->points.size() ; i++ ){
		hsv( 1.0-cloud->points[i].rgb, &r, &g, &b );
		rgb = ((r<<16)|(g<<8)|b);
		fprintf( fp," %lf %lf %lf %d\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, rgb );
	}
	fclose( fp );
}

void SamplingofVectorPairs( const pcl::PointCloud<pcl::PointNormal>::Ptr pnt,
							double3 mc, char *cm, double l1, double l2, double theta, 
							double TH_L, double TH_THETA,
							std::vector<vector_pair>& sampled ){

	std::vector<double3> iq1;
	std::vector<double3> iq2;
	std::vector<double> iq1_len;
	std::vector<double> iq2_len;
	std::vector<double3> iq1_n;
	std::vector<double3> iq2_n;
	double3 tmp_double3, tmp_nrm;
	vector_pair			cand_vp;
	double3				v1xv2, va, vb;
	double				length, inner_product, nrm_inner_product, ip_tmp;
	double				ip_p, ip_q1, ip_q2;
	//2013.10.01 インデクスの保存用変数の追加
	std::vector<int>	q1_idx;
	std::vector<int>	q2_idx;


	pcl::KdTreeFLANN<pcl::PointNormal, flann::L2_Simple<float> > kdtree;
	kdtree.setInputCloud( pnt );
	std::vector<int> pIdxRS;
	std::vector<float> pRSD; //point radius squared distance
	float			radius;
	if( l1 < l2 ) radius = (float)l2;
	else		  radius = (float)l1;
	radius += (float)TH_L;

	//Making co-occurrence histogram
	for( size_t j=0 ; j<pnt->points.size() ; j++ ){
		//2013.10.09 曲率の条件を追加 
		if( cm[j] ){
			iq1.clear(); iq1_len.clear(); iq1_n.clear(); q1_idx.clear();
			iq2.clear(); iq2_len.clear(); iq2_n.clear(); q2_idx.clear();
			if ( kdtree.radiusSearch( pnt->points[j], radius, pIdxRS, pRSD) > 0 ){	
	/*		std::cout << "Seed( "  <<  pnt->points[ j ].x   
					<< ", " << pnt->points[ j ].y 
					<< ", " << pnt->points[ j ].z 
					<< ") " << std::endl;
	*/
				for ( size_t i = 1; i < pIdxRS.size (); i++){
	/*			std::cout << "	ID:"  <<   pIdxRS[i] 
							<< "( " << pnt->points[ pIdxRS[i] ].x 
							<< ", " << pnt->points[ pIdxRS[i] ].y 
							<< ", " << pnt->points[ pIdxRS[i] ].z 
							<< ", " << pnt->points[ pIdxRS[i] ].normal_x 
							<< ", " << pnt->points[ pIdxRS[i] ].normal_y 
							<< ", " << pnt->points[ pIdxRS[i] ].normal_z 
							<< ") (squared distance: " << pRSD[i] << ")" << std::endl;
	*/
					if( cm[ pIdxRS[i] ] ){
						va.x = pnt->points[j].normal_x; va.y = pnt->points[j].normal_y; va.z = pnt->points[j].normal_z;
						vb.x = pnt->points[ pIdxRS[i] ].normal_x; vb.y = pnt->points[ pIdxRS[i] ].normal_y; vb.z = pnt->points[ pIdxRS[i] ].normal_z;
						AkiInnerProduct( va, vb, &inner_product );
						if( 0<=inner_product ){
							//2013.10.09 曲率の条件を追加 
							if( fabs(l1-sqrt(pRSD[i]))<TH_L ){
								tmp_double3.x = pnt->points[ pIdxRS[i] ].x-pnt->points[j].x;
								tmp_double3.y = pnt->points[ pIdxRS[i] ].y-pnt->points[j].y;
								tmp_double3.z = pnt->points[ pIdxRS[i] ].z-pnt->points[j].z;
								iq1.push_back( tmp_double3 );
								iq1_len.push_back( sqrt( pRSD[i] ) );
								tmp_nrm.x = pnt->points[ pIdxRS[i] ].normal_x;
								tmp_nrm.y = pnt->points[ pIdxRS[i] ].normal_y;
								tmp_nrm.z = pnt->points[ pIdxRS[i] ].normal_z;
								iq1_n.push_back( tmp_nrm );
								//2013.10.01 インデクスの保存
								q1_idx.push_back( pIdxRS[i] );
							}
							//2013.10.09 曲率の条件を追加 
							if( fabs(l2-sqrt(pRSD[i]))<TH_L ){
								tmp_double3.x = pnt->points[ pIdxRS[i] ].x-pnt->points[j].x;
								tmp_double3.y = pnt->points[ pIdxRS[i] ].y-pnt->points[j].y;
								tmp_double3.z = pnt->points[ pIdxRS[i] ].z-pnt->points[j].z;
								iq2.push_back( tmp_double3 );
								iq2_len.push_back( sqrt( pRSD[i] ) );
								tmp_nrm.x = pnt->points[ pIdxRS[i] ].normal_x;
								tmp_nrm.y = pnt->points[ pIdxRS[i] ].normal_y;
								tmp_nrm.z = pnt->points[ pIdxRS[i] ].normal_z;
								iq2_n.push_back( tmp_nrm );
								//2013.10.01 インデクスの保存
								q2_idx.push_back( pIdxRS[i] );
							}
						}
					}
				}
	//			if( j==0 ) fprintf( SE,"n1=%d, n2=%d\n", (int)iq1.size(), (int)iq2.size() );
				//Step2
				for( size_t jj=0 ; jj<iq1.size() ; jj++ ){
					for( size_t ii=0 ; ii<iq2.size() ; ii++ ){
//						if( j==0 ) fprintf( SE,"ii=%d, jj=%d\n", ii, jj );
						inner_product = ( (iq1[jj].x*iq2[ii].x)+(iq1[jj].y*iq2[ii].y)+(iq1[jj].z*iq2[ii].z) )/(iq1_len[jj]*iq2_len[ii]);
						inner_product = (180.0*acos(inner_product))/M_PI;//Radian -> Degree
						nrm_inner_product = ( (iq1_n[jj].x*iq2_n[ii].x)+(iq1_n[jj].y*iq2_n[ii].y)+(iq1_n[jj].z*iq2_n[ii].z) );
//						fprintf( SE,"inner product:%lf\n\n",inner_product);
//						fprintf( SE,"iq1(%lf,%lf,%lf)\n",iq1[jj].x, iq1[jj].y, iq1[jj].z);
//						fprintf( SE,"iq1_len:%lf\n",iq1_len[jj]);
//						fprintf( SE,"iq2(%lf,%lf,%lf)\n",iq2[ii].x, iq2[ii].y, iq2[ii].z);
//						fprintf( SE,"iq2_len:%lf\n",iq2_len[ii]);
						if( (fabs(inner_product-theta)<TH_THETA) && (0<=nrm_inner_product) ){
							cand_vp.p.x = pnt->points[j].x; cand_vp.p.y = pnt->points[j].y; cand_vp.p.z = pnt->points[j].z;
							cand_vp.q1.x = iq1[jj].x+pnt->points[j].x; cand_vp.q1.y = iq1[jj].y+pnt->points[j].y; cand_vp.q1.z = iq1[jj].z+pnt->points[j].z;
							cand_vp.q2.x = iq2[ii].x+pnt->points[j].x; cand_vp.q2.y = iq2[ii].y+pnt->points[j].y; cand_vp.q2.z = iq2[ii].z+pnt->points[j].z;
							cand_vp.np.x = pnt->points[j].normal_x; cand_vp.np.y = pnt->points[j].normal_y; cand_vp.np.z = pnt->points[j].normal_z;
							cand_vp.nq1.x = iq1_n[jj].x; cand_vp.nq1.y = iq1_n[jj].y; cand_vp.nq1.z = iq1_n[jj].z;
							cand_vp.nq2.x = iq2_n[ii].x; cand_vp.nq2.y = iq2_n[ii].y; cand_vp.nq2.z = iq2_n[ii].z;
							//Calc v1xv2
							AkiCrossProduct( &iq1[jj], &iq2[ii], &v1xv2 );
							AkiInnerProduct( cand_vp.np, v1xv2, &ip_tmp );
							if( ip_tmp<0 ){ //Flipping of cross product of v1 and v2.
								v1xv2.x *= -1.0;
								v1xv2.y *= -1.0;
								v1xv2.z *= -1.0;
							}
							length = sqrt( (v1xv2.x*v1xv2.x)+(v1xv2.y*v1xv2.y)+(v1xv2.z*v1xv2.z) );
							cand_vp.nvp.x = v1xv2.x/length; cand_vp.nvp.y = v1xv2.y/length; cand_vp.nvp.z = v1xv2.z/length;
							AkiInnerProduct( cand_vp.np, cand_vp.nvp, &ip_p );
							AkiInnerProduct( cand_vp.nq1, cand_vp.nvp, &ip_q1 );
							AkiInnerProduct( cand_vp.nq2, cand_vp.nvp, &ip_q2 );
							cand_vp.ip_p = ip_p;
							cand_vp.ip_q1 = ip_q1;
							cand_vp.ip_q2= ip_q2;
							//2013.10.08 重心座標の登録
							cand_vp.vc.x = mc.x - cand_vp.p.x; cand_vp.vc.y = mc.y - cand_vp.p.y; cand_vp.vc.z = mc.z - cand_vp.p.z;
							//2013.10.01 インデクスの保存
							cand_vp.p_idx = j;
							cand_vp.q1_idx = q1_idx[jj];
							cand_vp.q2_idx = q2_idx[ii];
							sampled.push_back( cand_vp );
						}
					}
				}
			}

		}
//		fprintf( stderr,"number of extracted vp: %d\n", sampled.size() );
	}

}

void VPMarge( std::vector<vector_pair>& input, double th_dist, std::vector<vector_pair>& output ){

	int	input_length;
	std::vector<unsigned char> flag;
	input_length = (int)input.size();
	

	//20150507 秋月追記input_lengthが2以上のときのみ使える処理
	if( 2 <= input_length ){
		flag.resize( input_length );
		for( int i=0 ; i<input_length ; i++ ){
			flag[i] = 0;
		}
		
		double sq_th_dist;
		double dist_q1, dist_q2;

		sq_th_dist = th_dist*th_dist;
		for( int j=0 ; j<input_length ; j++ ){
			if( flag[j] == 0 ){
				flag[j] = 1;
				output.push_back( input[j] );
				for( int i=j+1 ; i<input_length ; i++ ){
					if( flag[i] == 0 ){
						dist_q1 = ((input[j].q1.x - input[i].q1.x)*(input[j].q1.x - input[i].q1.x))
								+ ((input[j].q1.y - input[i].q1.y)*(input[j].q1.y - input[i].q1.y))
								+ ((input[j].q1.z - input[i].q1.z)*(input[j].q1.z - input[i].q1.z));
						
						dist_q2 = ((input[j].q2.x - input[i].q2.x)*(input[j].q2.x - input[i].q2.x))
								+ ((input[j].q2.y - input[i].q2.y)*(input[j].q2.y - input[i].q2.y))
								+ ((input[j].q2.z - input[i].q2.z)*(input[j].q2.z - input[i].q2.z));

						if( (dist_q1<sq_th_dist) && (dist_q1<sq_th_dist) ){
							flag[i] = 1;
						}
					}
				}
			}
		}
	}else{
		//20150507 秋月追記 ベクトルペアが少なすぎるときはそのままコピー
		for( int j=0 ; j<input_length ; j++ ){
			output.push_back( input[j] );
		}
	}
}


void SamplingofVectorPairs2( const pcl::PointCloud<pcl::PointNormal>::Ptr pnt,
							double3 mc, char *cm, double l1, double l2, double theta, 
							double TH_L, double TH_THETA, double marge_dist,
							std::vector<vector_pair>& sampled ){

	std::vector<double3> iq1;
	std::vector<double3> iq2;
	std::vector<double> iq1_len;
	std::vector<double> iq2_len;
	std::vector<double3> iq1_n;
	std::vector<double3> iq2_n;
	double3 tmp_double3, tmp_nrm;
	vector_pair			cand_vp;
	double3				v1xv2, va, vb;
	double				length, inner_product, nrm_inner_product, ip_tmp;
	double				ip_p, ip_q1, ip_q2;
	//2013.10.01 インデクスの保存用変数の追加
	std::vector<int>	q1_idx;
	std::vector<int>	q2_idx;

	
	std::vector<vector_pair> cand_vps;
	std::vector<vector_pair> cand_vps_marged;


	pcl::KdTreeFLANN<pcl::PointNormal, flann::L2_Simple<float> > kdtree;
	kdtree.setInputCloud( pnt );
	std::vector<int> pIdxRS;
	std::vector<float> pRSD; //point radius squared distance
	float			radius;
	if( l1 < l2 ) radius = (float)l2;
	else		  radius = (float)l1;
	radius += (float)TH_L;

	//Making co-occurrence histogram
	for( size_t j=0 ; j<pnt->points.size() ; j++ ){
		pIdxRS.clear();
		pRSD.clear();
		//2013.10.09 曲率の条件を追加 
		if( cm[j] ){
			iq1.clear(); iq1_len.clear(); iq1_n.clear(); q1_idx.clear();
			iq2.clear(); iq2_len.clear(); iq2_n.clear(); q2_idx.clear();
			if ( kdtree.radiusSearch( pnt->points[j], radius, pIdxRS, pRSD) > 0 ){	
	/*		std::cout << "Seed( "  <<  pnt->points[ j ].x   
					<< ", " << pnt->points[ j ].y 
					<< ", " << pnt->points[ j ].z 
					<< ") " << std::endl;
	*/
				for ( size_t i = 1; i < pIdxRS.size (); i++){
	/*			std::cout << "	ID:"  <<   pIdxRS[i] 
							<< "( " << pnt->points[ pIdxRS[i] ].x 
							<< ", " << pnt->points[ pIdxRS[i] ].y 
							<< ", " << pnt->points[ pIdxRS[i] ].z 
							<< ", " << pnt->points[ pIdxRS[i] ].normal_x 
							<< ", " << pnt->points[ pIdxRS[i] ].normal_y 
							<< ", " << pnt->points[ pIdxRS[i] ].normal_z 
							<< ") (squared distance: " << pRSD[i] << ")" << std::endl;
	*/

					if( cm[ pIdxRS[i] ] ){
						va.x = pnt->points[j].normal_x; va.y = pnt->points[j].normal_y; va.z = pnt->points[j].normal_z;
						vb.x = pnt->points[ pIdxRS[i] ].normal_x; vb.y = pnt->points[ pIdxRS[i] ].normal_y; vb.z = pnt->points[ pIdxRS[i] ].normal_z;
						AkiInnerProduct( va, vb, &inner_product );
						if( 0<=inner_product ){
							//2013.10.09 曲率の条件を追加 
							if( fabs(l1-sqrt(pRSD[i]))<TH_L ){
								tmp_double3.x = pnt->points[ pIdxRS[i] ].x-pnt->points[j].x;
								tmp_double3.y = pnt->points[ pIdxRS[i] ].y-pnt->points[j].y;
								tmp_double3.z = pnt->points[ pIdxRS[i] ].z-pnt->points[j].z;
								iq1.push_back( tmp_double3 );
								iq1_len.push_back( sqrt( pRSD[i] ) );
								tmp_nrm.x = pnt->points[ pIdxRS[i] ].normal_x;
								tmp_nrm.y = pnt->points[ pIdxRS[i] ].normal_y;
								tmp_nrm.z = pnt->points[ pIdxRS[i] ].normal_z;
								iq1_n.push_back( tmp_nrm );
								//2013.10.01 インデクスの保存
								q1_idx.push_back( pIdxRS[i] );
							}
							//2013.10.09 曲率の条件を追加 
							if( fabs(l2-sqrt(pRSD[i]))<TH_L ){
								tmp_double3.x = pnt->points[ pIdxRS[i] ].x-pnt->points[j].x;
								tmp_double3.y = pnt->points[ pIdxRS[i] ].y-pnt->points[j].y;
								tmp_double3.z = pnt->points[ pIdxRS[i] ].z-pnt->points[j].z;
								iq2.push_back( tmp_double3 );
								iq2_len.push_back( sqrt( pRSD[i] ) );
								tmp_nrm.x = pnt->points[ pIdxRS[i] ].normal_x;
								tmp_nrm.y = pnt->points[ pIdxRS[i] ].normal_y;
								tmp_nrm.z = pnt->points[ pIdxRS[i] ].normal_z;
								iq2_n.push_back( tmp_nrm );
								//2013.10.01 インデクスの保存
								q2_idx.push_back( pIdxRS[i] );
							}
						}
					}
				}
	//			if( j==0 ) fprintf( SE,"n1=%d, n2=%d\n", (int)iq1.size(), (int)iq2.size() );
				//Step2
				cand_vps.clear();
				cand_vps_marged.clear();
				for( size_t jj=0 ; jj<iq1.size() ; jj++ ){
					for( size_t ii=0 ; ii<iq2.size() ; ii++ ){
//						if( j==0 ) fprintf( SE,"ii=%d, jj=%d\n", ii, jj );
						inner_product = ( (iq1[jj].x*iq2[ii].x)+(iq1[jj].y*iq2[ii].y)+(iq1[jj].z*iq2[ii].z) )/(iq1_len[jj]*iq2_len[ii]);
						inner_product = (180.0*acos(inner_product))/M_PI;//Radian -> Degree
						nrm_inner_product = ( (iq1_n[jj].x*iq2_n[ii].x)+(iq1_n[jj].y*iq2_n[ii].y)+(iq1_n[jj].z*iq2_n[ii].z) );
//						fprintf( SE,"inner product:%lf\n\n",inner_product);
//						fprintf( SE,"iq1(%lf,%lf,%lf)\n",iq1[jj].x, iq1[jj].y, iq1[jj].z);
//						fprintf( SE,"iq1_len:%lf\n",iq1_len[jj]);
//						fprintf( SE,"iq2(%lf,%lf,%lf)\n",iq2[ii].x, iq2[ii].y, iq2[ii].z);
//						fprintf( SE,"iq2_len:%lf\n",iq2_len[ii]);
						if( (fabs(inner_product-theta)<TH_THETA) && (0<=nrm_inner_product) ){
							cand_vp.p.x = pnt->points[j].x; cand_vp.p.y = pnt->points[j].y; cand_vp.p.z = pnt->points[j].z;
							cand_vp.q1.x = iq1[jj].x+pnt->points[j].x; cand_vp.q1.y = iq1[jj].y+pnt->points[j].y; cand_vp.q1.z = iq1[jj].z+pnt->points[j].z;
							cand_vp.q2.x = iq2[ii].x+pnt->points[j].x; cand_vp.q2.y = iq2[ii].y+pnt->points[j].y; cand_vp.q2.z = iq2[ii].z+pnt->points[j].z;
							cand_vp.np.x = pnt->points[j].normal_x; cand_vp.np.y = pnt->points[j].normal_y; cand_vp.np.z = pnt->points[j].normal_z;
							cand_vp.nq1.x = iq1_n[jj].x; cand_vp.nq1.y = iq1_n[jj].y; cand_vp.nq1.z = iq1_n[jj].z;
							cand_vp.nq2.x = iq2_n[ii].x; cand_vp.nq2.y = iq2_n[ii].y; cand_vp.nq2.z = iq2_n[ii].z;
							//Calc v1xv2
							AkiCrossProduct( &iq1[jj], &iq2[ii], &v1xv2 );
							AkiInnerProduct( cand_vp.np, v1xv2, &ip_tmp );
							if( ip_tmp<0 ){ //Flipping of cross product of v1 and v2.
								v1xv2.x *= -1.0;
								v1xv2.y *= -1.0;
								v1xv2.z *= -1.0;
							}
							length = sqrt( (v1xv2.x*v1xv2.x)+(v1xv2.y*v1xv2.y)+(v1xv2.z*v1xv2.z) );
							cand_vp.nvp.x = v1xv2.x/length; cand_vp.nvp.y = v1xv2.y/length; cand_vp.nvp.z = v1xv2.z/length;
							AkiInnerProduct( cand_vp.np, cand_vp.nvp, &ip_p );
							AkiInnerProduct( cand_vp.nq1, cand_vp.nvp, &ip_q1 );
							AkiInnerProduct( cand_vp.nq2, cand_vp.nvp, &ip_q2 );
							cand_vp.ip_p = ip_p;
							cand_vp.ip_q1 = ip_q1;
							cand_vp.ip_q2= ip_q2;
							//2013.10.08 重心座標の登録
							cand_vp.vc.x = mc.x - cand_vp.p.x; cand_vp.vc.y = mc.y - cand_vp.p.y; cand_vp.vc.z = mc.z - cand_vp.p.z;
							//2013.10.01 インデクスの保存
							cand_vp.p_idx = j;
							cand_vp.q1_idx = q1_idx[jj];
							cand_vp.q2_idx = q2_idx[ii];
							cand_vps.push_back( cand_vp );
						}
					}
				}
				//////ここに処理を書く
				VPMarge( cand_vps, marge_dist, cand_vps_marged );
				/////
				if( 0 < cand_vps_marged.size() ){

					for( size_t kk=0 ; kk<cand_vps_marged.size() ; kk++ ){
						sampled.push_back( cand_vps_marged[kk] );
					}

				}

			}

		}
//		fprintf( stderr,"number of extracted vp: %d\n", sampled.size() );
	}

}





void FlatRemoving( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
				   float	radius, double threshold,
				   const pcl::PointCloud<pcl::PointNormal>::Ptr reduced ){


	std::vector<double3> pnt;
	std::vector<double3> nrm;
	double				ip, sum_ip, ave_ip;
	double3				tmp_nrm, tmp_surround_nrm;
	double3				not_flat_pnt, not_flat_nrm;	

	pcl::KdTreeFLANN<pcl::PointNormal, flann::L2_Simple<float> > kdtree;
	kdtree.setInputCloud( cloud );
	std::vector<int> pIdxRS;
	std::vector<float> pRSD; //point radius squared distance

	for( size_t j=0 ; j<cloud->points.size() ; j++ ){
		tmp_nrm.x = cloud->points[ j ].normal_x; 
		tmp_nrm.y = cloud->points[ j ].normal_y;
		tmp_nrm.z = cloud->points[ j ].normal_z;
		sum_ip = 0.0;
		if ( kdtree.radiusSearch( cloud->points[j], radius, pIdxRS, pRSD) > 1 ){	
			for ( size_t i = 1; i < pIdxRS.size (); i++){
				tmp_surround_nrm.x = cloud->points[ pIdxRS[i] ].normal_x;
				tmp_surround_nrm.y = cloud->points[ pIdxRS[i] ].normal_y;
				tmp_surround_nrm.z = cloud->points[ pIdxRS[i] ].normal_z; 
				
				AkiInnerProduct( tmp_nrm, tmp_surround_nrm, &ip );
				sum_ip += ip;
			}
			ave_ip = (double)(sum_ip/(double)(pIdxRS.size()-1));
			if( ave_ip < threshold ){
				not_flat_pnt.x = cloud->points[j].x;	
				not_flat_pnt.y = cloud->points[j].y;	
				not_flat_pnt.z = cloud->points[j].z;	
				not_flat_nrm.x = cloud->points[j].normal_x;	
				not_flat_nrm.y = cloud->points[j].normal_y;	
				not_flat_nrm.z = cloud->points[j].normal_z;	

				pnt.push_back( not_flat_pnt );
				nrm.push_back( not_flat_nrm );
			}
		}
	}

	
	reduced->width = (int)pnt.size();
	reduced->height = 1;
	reduced->is_dense = false;
	reduced->points.resize((int)reduced->width);
	for( size_t j=0 ; j<pnt.size() ; j++ ){
		reduced->points[j].x = (float)pnt[j].x;	
		reduced->points[j].y = (float)pnt[j].y;	
		reduced->points[j].z = (float)pnt[j].z;	
		reduced->points[j].normal_x = (float)nrm[j].x;	
		reduced->points[j].normal_y = (float)nrm[j].y;	
		reduced->points[j].normal_z = (float)nrm[j].z;	
	}
}


// 点群の可視化．Matがポインタ渡しになっていることに注意．サイズは呼び出し元で定義しておくこと．
void PCDNormal2CVMat( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud, double pitch, double3 trans, int fill, cv::Mat *img ){

	int				imgX, imgY, imgZ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	cloud2->width = cloud->size();
	cloud2->height = 1;
	cloud2->is_dense = false;
	cloud2->points.resize(cloud2->width);	
	pcl::PointXYZ tmp;
	static int	n, Ndata;
	int			gap, swap;

	//点群のコピー
	for( size_t i=0 ; i<cloud->points.size() ; i++ ){
		cloud2->points[i].x = cloud->points[i].x;
		cloud2->points[i].y = cloud->points[i].y;
		cloud2->points[i].z = cloud->points[i].z;
	}

	//遠いもの順にソート
	Ndata = cloud2->points.size();
	gap = Ndata;
	swap = 1;
	while( gap>1 || swap ) {
		gap = (int)((double)gap/1.3) ;
		if(gap == 9 || gap == 10) gap = 11 ;
		swap = 0;
		for( n=0; n<(Ndata-gap) ; n++ ){
			if( cloud2->points[n+gap].z < cloud2->points[n].z ){
				tmp.x = cloud2->points[n].x; tmp.y = cloud2->points[n].y; tmp.z = cloud2->points[n].z;
				cloud2->points[n].x = cloud2->points[n+gap].x; cloud2->points[n].y = cloud2->points[n+gap].y; cloud2->points[n].z = cloud2->points[n+gap].z;
				cloud2->points[n+gap].x = tmp.x; cloud2->points[n+gap].y = tmp.y; cloud2->points[n+gap].z = tmp.z;
				swap = 1;
			}
		}
	}

	for( size_t i=0 ; i<cloud2->points.size() ; i++ ){
		imgX = (int)(((cloud2->points[i].x + trans.x)/pitch)+0.5);
		imgY = (int)(((cloud2->points[i].y + trans.y)/pitch)+0.5);
		imgZ = (int)(((cloud2->points[i].z + trans.z)/pitch)+0.5);
		if( (0<=imgX) && (0<=imgY) && (0<=imgZ) && (imgX<img->cols) && (imgY<img->rows) && (imgZ<256) ){
			img->at<uchar>( imgY, imgX ) = (unsigned char)imgZ;
			//ここから下は穴埋め処理(8近傍)
			if( fill ){
				if( imgX+1<img->cols ){
					img->at<uchar>( imgY, imgX+1 ) = (unsigned char)imgZ;
				}
				if( 0<imgX-1 ){
					img->at<uchar>( imgY, imgX-1 ) = (unsigned char)imgZ;
				}
				if( imgY+1<img->cols ){
					img->at<uchar>( imgY+1, imgX ) = (unsigned char)imgZ;
				}
				if( 0<imgY-1 ){
					img->at<uchar>( imgY-1, imgX ) = (unsigned char)imgZ;
				}
				//if( (imgX+1<img.rows) && (0<imgY-1) ){
				//	img.at<uchar>( imgY-1, imgX+1 ) = (unsigned char)imgZ;
				//}
				//if( (imgX+1<img.rows) && (imgY+1<img.cols) ){
				//	img.at<uchar>( imgY+1, imgX+1 ) = (unsigned char)imgZ;
				//}
				//if( (0<imgX-1) && (imgY+1<img.cols) ){
				//	img.at<uchar>( imgY+1, imgX-1 ) = (unsigned char)imgZ;
				//}
				//if( (0<imgX-1) && (0<imgY-1) ){
				//	img.at<uchar>( imgY-1, imgX-1 ) = (unsigned char)imgZ;
				//}
			}
		}
	}

}

void PCDXYZ2CVMat( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double pitch, double3 trans, int fill, cv::Mat *img ){

	int				imgX, imgY, imgZ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	cloud2->width = cloud->size();
	cloud2->height = 1;
	cloud2->is_dense = false;
	cloud2->points.resize(cloud2->width);	
	pcl::PointXYZ tmp;
	static int	n, Ndata;
	int			gap, swap;

	//点群のコピー
	for( size_t i=0 ; i<cloud->points.size() ; i++ ){
		cloud2->points[i].x = cloud->points[i].x;
		cloud2->points[i].y = cloud->points[i].y;
		cloud2->points[i].z = cloud->points[i].z;
	}

	//遠いもの順にソート
	Ndata = cloud2->points.size();
	gap = Ndata;
	swap = 1;
	while( gap>1 || swap ) {
		gap = (int)((double)gap/1.3) ;
		if(gap == 9 || gap == 10) gap = 11 ;
		swap = 0;
		for( n=0; n<(Ndata-gap) ; n++ ){
			if( cloud2->points[n+gap].z < cloud2->points[n].z ){
				tmp.x = cloud2->points[n].x; tmp.y = cloud2->points[n].y; tmp.z = cloud2->points[n].z;
				cloud2->points[n].x = cloud2->points[n+gap].x; cloud2->points[n].y = cloud2->points[n+gap].y; cloud2->points[n].z = cloud2->points[n+gap].z;
				cloud2->points[n+gap].x = tmp.x; cloud2->points[n+gap].y = tmp.y; cloud2->points[n+gap].z = tmp.z;
				swap = 1;
			}
		}
	}

	for( size_t i=0 ; i<cloud2->points.size() ; i++ ){
		imgX = (int)(((cloud2->points[i].x + trans.x)/pitch)+0.5);
		imgY = (int)(((cloud2->points[i].y + trans.y)/pitch)+0.5);
		imgZ = (int)(((cloud2->points[i].z + trans.z)/pitch)+0.5);
		if( (0<=imgX) && (0<=imgY) && (0<=imgZ) && (imgX<img->cols) && (imgY<img->rows) && (imgZ<256) ){
			img->at<uchar>( imgY, imgX ) = (unsigned char)imgZ;
			//ここから下は穴埋め処理(8近傍)
			if( fill ){
				if( imgX+1<img->cols ){
					img->at<uchar>( imgY, imgX+1 ) = (unsigned char)imgZ;
				}
				if( 0<imgX-1 ){
					img->at<uchar>( imgY, imgX-1 ) = (unsigned char)imgZ;
				}
				if( imgY+1<img->rows ){
					img->at<uchar>( imgY+1, imgX ) = (unsigned char)imgZ;
				}
				if( 0<imgY-1 ){
					img->at<uchar>( imgY-1, imgX ) = (unsigned char)imgZ;
				}
			}
		}
	}

}

void PCD2CVMatScoreMap( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double pitch, double3 trans, int fill, cv::Mat scene, cv::Mat img ){

	int				imgX, imgY, imgZ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	cloud2->width = cloud->size();
	cloud2->height = 1;
	cloud2->is_dense = false;
	cloud2->points.resize(cloud2->width);	
	pcl::PointXYZ tmp;
	static int	n, Ndata;
	int			gap, swap;

	std::vector<std::vector<double>> score_map;
	score_map.resize( (int)img.rows );
	for( size_t i=0 ; i<score_map.size() ; i++ ){
		score_map[i].resize( (int)img.cols );
	}
	for( int j=0 ; j<img.rows ; j++ ){
		for( int i=0 ; i<img.cols ; i++ ){
			score_map[j][i] = 0.0;
		}
	}

	//点群のコピー
	for( size_t i=0 ; i<cloud->points.size() ; i++ ){
		cloud2->points[i].x = cloud->points[i].x;
		cloud2->points[i].y = cloud->points[i].y;
		cloud2->points[i].z = cloud->points[i].z;
	}

	//遠いもの順にソート
	Ndata = cloud2->points.size();
	gap = Ndata;
	swap = 1;
	while( gap>1 || swap ) {
		gap = (int)((double)gap/1.3) ;
		if(gap == 9 || gap == 10) gap = 11 ;
		swap = 0;
		for( n=0; n<(Ndata-gap) ; n++ ){
			if( cloud2->points[n+gap].z < cloud2->points[n].z ){
				tmp.x = cloud2->points[n].x; tmp.y = cloud2->points[n].y; tmp.z = cloud2->points[n].z;
				cloud2->points[n].x = cloud2->points[n+gap].x; cloud2->points[n].y = cloud2->points[n+gap].y; cloud2->points[n].z = cloud2->points[n+gap].z;
				cloud2->points[n+gap].x = tmp.x; cloud2->points[n+gap].y = tmp.y; cloud2->points[n+gap].z = tmp.z;
				swap = 1;
			}
		}
	}

	for( size_t i=0 ; i<cloud2->points.size() ; i++ ){
		imgX = (int)(((cloud2->points[i].x + trans.x)/pitch)+0.5);
		imgY = (int)(((cloud2->points[i].y + trans.y)/pitch)+0.5);
		imgZ = (int)(((cloud2->points[i].z + trans.z)/pitch)+0.5);
		if( (0<=imgX) && (0<=imgY) && (0<=imgZ) && (imgX<img.rows) && (imgY<img.cols) && (imgZ<256) ){
			score_map[ imgY ][ imgX ] +=1.0;
		}
	}

	//最大値取得
	double	max;
	max = score_map[0][0];
	for( int j=0 ; j<img.rows ; j++ ){
		for( int i=0 ; i<img.cols ; i++ ){
			if( max < score_map[j][i] ){
				max = score_map[j][i];
			}
		}
	}
	//最大値で正規化	
	fprintf( stderr,"max = %.1lf\n", max );
	for( int j=0 ; j<img.rows ; j++ ){
		for( int i=0 ; i<img.cols ; i++ ){
			score_map[j][i] = score_map[j][i]/max;
		}
	}

	cv::Mat colormap, colormap2;
	colormap = cv::Mat::zeros( img.cols, img.rows, CV_8UC3 );
	cv::Vec3b bgr;
	int	r, g, b;
	cv::Vec3b bgr_si;
	for( int j=0 ; j<img.rows ; j++ ){
		for( int i=0 ; i<img.cols ; i++ ){
			//HSV変換
			hsv( score_map[j][i], &r, &g, &b );
			if( score_map[j][i] == 0.0 ){
				r = g = b = 0;
			}
			//スコアマップ生成
			bgr[0] = b; bgr[1] = g; bgr[2] = r;
			colormap.at<cv::Vec3b>( j, i ) = bgr;

			//シーンに重畳したスコアマップの生成
			bgr_si[0] = (b + scene.at<uchar>(j,i)) / 2;
			bgr_si[1] = (g + scene.at<uchar>(j,i)) / 2;
			bgr_si[2] = (r + scene.at<uchar>(j,i)) / 2;
			img.at<cv::Vec3b>( j, i ) = bgr_si;
		}
	}
	cv::flip( colormap, colormap2, 0 );
	cv::imwrite( "scoremapRGB.bmp", colormap2 );

}

void PCD2CVMatHypothesis( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,  const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_hyp,
							double pitch, double3 trans, cv::Mat img ){

	int				imgX, imgY, imgZ;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hyp2 (new pcl::PointCloud<pcl::PointXYZ>);
	cloud2->width = cloud->size();
	cloud2->height = 1;
	cloud2->is_dense = false;
	cloud2->points.resize(cloud2->width);	
	pcl::PointXYZ tmp;
	static int	n, Ndata;
	int			gap, swap;

	//点群のコピー
	for( size_t i=0 ; i<cloud->points.size() ; i++ ){
		cloud2->points[i].x = cloud->points[i].x;
		cloud2->points[i].y = cloud->points[i].y;
		cloud2->points[i].z = cloud->points[i].z;
	}

	//遠いもの順にソート
	Ndata = cloud2->points.size();
	gap = Ndata;
	swap = 1;
	while( gap>1 || swap ) {
		gap = (int)((double)gap/1.3) ;
		if(gap == 9 || gap == 10) gap = 11 ;
		swap = 0;
		for( n=0; n<(Ndata-gap) ; n++ ){
			if( cloud2->points[n+gap].z < cloud2->points[n].z ){
				tmp.x = cloud2->points[n].x; tmp.y = cloud2->points[n].y; tmp.z = cloud2->points[n].z;
				cloud2->points[n].x = cloud2->points[n+gap].x; cloud2->points[n].y = cloud2->points[n+gap].y; cloud2->points[n].z = cloud2->points[n+gap].z;
				cloud2->points[n+gap].x = tmp.x; cloud2->points[n+gap].y = tmp.y; cloud2->points[n+gap].z = tmp.z;
				swap = 1;
			}
		}
	}

	//点群のコピー
	cloud_hyp2->width = cloud_hyp->size();
	cloud_hyp2->height = 1;
	cloud_hyp2->is_dense = false;
	cloud_hyp2->points.resize(cloud_hyp2->width);
	for( size_t i=0 ; i<cloud_hyp->points.size() ; i++ ){
		cloud_hyp2->points[i].x = cloud_hyp->points[i].x;
		cloud_hyp2->points[i].y = cloud_hyp->points[i].y;
		cloud_hyp2->points[i].z = cloud_hyp->points[i].z;
	}

	//遠いもの順にソート
	Ndata = cloud_hyp2->points.size();
	gap = Ndata;
	swap = 1;
	while( gap>1 || swap ) {
		gap = (int)((double)gap/1.3) ;
		if(gap == 9 || gap == 10) gap = 11 ;
		swap = 0;
		for( n=0; n<(Ndata-gap) ; n++ ){
			if( cloud_hyp2->points[n+gap].z < cloud_hyp2->points[n].z ){
				tmp.x = cloud_hyp2->points[n].x; tmp.y = cloud_hyp2->points[n].y; tmp.z = cloud_hyp2->points[n].z;
				cloud_hyp2->points[n].x = cloud_hyp2->points[n+gap].x; cloud_hyp2->points[n].y = cloud_hyp2->points[n+gap].y; cloud_hyp2->points[n].z = cloud_hyp2->points[n+gap].z;
				cloud_hyp2->points[n+gap].x = tmp.x; cloud_hyp2->points[n+gap].y = tmp.y; cloud_hyp2->points[n+gap].z = tmp.z;
				swap = 1;
			}
		}
	}

	//シーンデータの描画
	cv::Vec3b bgr;
	for( int i=0 ; i<(int)cloud2->points.size() ; i++ ){
		imgX = (int)(((cloud2->points[i].x + trans.x)/pitch)+0.5);
		imgY = (int)(((cloud2->points[i].y + trans.y)/pitch)+0.5);
		imgZ = (int)(((cloud2->points[i].z + trans.z)/pitch)+0.5);
		if( (0<=imgX) && (0<=imgY) && (0<=imgZ) && (imgX<img.cols) && (imgY<img.rows) && (imgZ<256) ){
			bgr[0] = bgr[1] = bgr[2] = imgZ;
			img.at<cv::Vec3b>( imgY, imgX ) = bgr;
		}
	}

	for( int i=0 ; i<(int)cloud_hyp2->points.size() ; i++ ){
		imgX = (int)(((cloud_hyp2->points[i].x + trans.x)/pitch)+0.5);
		imgY = (int)(((cloud_hyp2->points[i].y + trans.y)/pitch)+0.5);
		imgZ = (int)(((cloud_hyp2->points[i].z + trans.z)/pitch)+0.5);
		if( (0<=imgX) && (0<=imgY) && (0<=imgZ) && (imgX<img.cols) && (imgY<img.cols) && (imgZ<256) ){
			bgr[0] = bgr[2] = 0;
			bgr[1] = imgZ;
			cv::circle( img, cv::Point(imgX, imgY), 2, cv::Scalar(0, imgZ, 0 ), -1, CV_AA, 0 );

		}
	}


}

void PCD2CVMatDrawCentroid( const double3 centroid, double pitch, double3 trans, cv::Mat scene, cv::Mat *dst ){

	int				imgX, imgY;
	cv::Vec3b bgr;
	fprintf( stderr,"size of centroid_img: %d x %d\n", scene.rows, scene.cols );
	fprintf( stderr,"centroid location( %.2lf, %.2lf, %.2lf)\n", centroid.x, centroid.y, centroid.z );
	for( int j=0 ; j<scene.rows ; j++ ){
		for( int i=0 ; i<scene.cols ; i++ ){
			bgr[0] = bgr[1] = bgr[2] = scene.at<uchar>( j, i );
			dst->at<cv::Vec3b>( j, i ) = bgr;
		}
	}
	
	if( (centroid.x != -1.0) && (centroid.y != -1.0) && (centroid.z != -1.0) ){
		imgX = (int)(((centroid.x + trans.x)/pitch)+0.5);
		imgY = (int)(((centroid.y + trans.y)/pitch)+0.5);
		cv::circle( *dst, cv::Point(imgX, imgY), 3, cv::Scalar(0,0,255), -1, CV_AA, 0 );
	}
}

void CloudTransformPntNrm( const pcl::PointCloud<pcl::PointNormal>::Ptr src, double R[][3], double3 trans, const pcl::PointCloud<pcl::PointNormal>::Ptr dst ){

	struct double3		tmp, tmp2;
	struct double3		nrm_tmp, nrm_tmp2;
	int					nPoints;

	if( src->points.size() != dst->points.size() ){
		fprintf( stderr,"Error: GeoTransByRT()\n");
		fprintf( stderr,"The size of src and dst is not equal.\n" );
		return;
	}
	nPoints = src->points.size();

	for( int i=0 ; i<nPoints ; i++ ){
		tmp.x = src->points[i].x;
		tmp.y = src->points[i].y;
		tmp.z = src->points[i].z;
		Aki3DMatrixx3DPoint( R, tmp, &tmp2 );
		dst->points[i].x = (float)tmp2.x;
		dst->points[i].y = (float)tmp2.y;
		dst->points[i].z = (float)tmp2.z;
		nrm_tmp.x = src->points[i].normal_x;
		nrm_tmp.y = src->points[i].normal_y;
		nrm_tmp.z = src->points[i].normal_z;
		Aki3DMatrixx3DPoint( R, nrm_tmp, &nrm_tmp2 );
		dst->points[i].normal_x = (float)nrm_tmp2.x;
		dst->points[i].normal_y = (float)nrm_tmp2.y;
		dst->points[i].normal_z = (float)nrm_tmp2.z;
	}
	for( int i=0 ; i<nPoints ; i++ ){
		dst->points[i].x += (float)trans.x;
		dst->points[i].y += (float)trans.y;
		dst->points[i].z += (float)trans.z;
	}

}

void CloudSubtraction( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bg, const double rad,
					  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::vector<int>& Idx ){

	// seed点周りの点群の取り出し
	std::vector<int> remove_flag;
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float> > tree;
	tree.setInputCloud( cloud );
	std::vector<int> pIdx;
	std::vector<float> pSd;

	pIdx.clear();
	pSd.clear();
	remove_flag.resize( cloud->points.size() );
	for( size_t i=0 ; i<cloud_bg->points.size() ; i++ ){
		if ( tree.radiusSearch( cloud_bg->points[i], rad, pIdx, pSd) > 0 ){
			for( size_t j=0 ; j<pIdx.size() ; j++ ){
				remove_flag[ pIdx[j] ] = 1;
			}
			pIdx.clear();
			pSd.clear();
		}
	}

	int cnt;
	cnt = 0;
	for( size_t i=0 ; i<remove_flag.size() ; i++ ){
		if( remove_flag[i] == 0 ){
			cloud_out->points.push_back( cloud->points[i] );
			Idx.push_back( i );
			cnt++;
		}
	}
	cloud_out->width = cnt;
	cloud_out->height = 1;

}

void CloudSubtractionInv( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_bg, const double rad,
					  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::vector<int>& Idx ){

	// seed点周りの点群の取り出し
	std::vector<int> remove_flag;
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float> > tree;
	tree.setInputCloud( cloud );
	std::vector<int> pIdx;
	std::vector<float> pSd;
	pcl::PointXYZ seed;

	pIdx.clear();
	pSd.clear();
	remove_flag.resize( cloud->points.size() );
	//20150427-秋月追記↓↓↓
	for( int i=0 ; i<remove_flag.size() ; i++ ){
		remove_flag[i] = 0;
	}
	//20150427-秋月追記↑↑↑
	for( size_t i=0 ; i<cloud_bg->points.size() ; i++ ){
		seed.x = cloud_bg->points[i].x;
		seed.y = cloud_bg->points[i].y;
		seed.z = cloud_bg->points[i].z;
		if ( tree.radiusSearch( seed, rad, pIdx, pSd) > 0 ){
			for( size_t j=0 ; j<pIdx.size() ; j++ ){
				remove_flag[ pIdx[j] ] = 1;
			}
			pIdx.clear();
			pSd.clear();
		}
	}

	int cnt;
	cnt = 0;
	for( size_t i=0 ; i<remove_flag.size() ; i++ ){
		if( remove_flag[i] == 1 ){
			cloud_out->points.push_back( cloud->points[i] );
			Idx.push_back( i );
			cnt++;
		}
	}
	cloud_out->width = cnt;
	cloud_out->height = 1;

}

// 点群のコピーをおこなう．保存するべきcloud_inにおける点のidはIdxに入っている．
void CopyPCD( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, pcl::PointCloud<int>& Idx,
				 pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out ){

	 cloud_out->width = Idx.size();
	 cloud_out->height = 1;
	 for( int i=0 ; i<Idx.size() ; i++ ){
		cloud_out->points.push_back( cloud_in->points[Idx[i]] );	
	 }
}
void CopyPCD( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<int>& Idx,
				 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out ){

	 cloud_out->width = Idx.size();
	 cloud_out->height = 1;
	 for( int i=0 ; i<Idx.size() ; i++ ){
		cloud_out->points.push_back( cloud_in->points[Idx[i]] );	
	 }
}

void CopyPCD( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<int>& Idx,
				 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out ){

	 cloud_out->width = Idx.size();
	 cloud_out->height = 1;
	 for( int i=0 ; i<Idx.size() ; i++ ){
		cloud_out->points.push_back( cloud_in->points[Idx[i]] );	
	 }
}


// 点群のコピーをおこなう．保存するべきcloud_inにおける点のidはIdxに入っている．
void CopyPCD( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, std::vector<int>& Idx,
				 pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out ){

	 cloud_out->width = Idx.size();
	 cloud_out->height = 1;
	 for( int i=0 ; i<Idx.size() ; i++ ){
		cloud_out->points.push_back( cloud_in->points[Idx[i]] );	
	 }
}

void CopyPCD( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, std::vector<int>& Idx,
				 pcl::PointCloud<pcl::PointNormal> *cloud_out ){

	 cloud_out->width = Idx.size();
	 cloud_out->height = 1;
	 for( int i=0 ; i<Idx.size() ; i++ ){
		cloud_out->points.push_back( cloud_in->points[Idx[i]] );	
	 }
}

void CopyPCD( const pcl::PointCloud<pcl::PointNormal> *cloud_in, std::vector<int>& Idx,
				 pcl::PointCloud<pcl::PointNormal> *cloud_out ){

	 cloud_out->width = Idx.size();
	 cloud_out->height = 1;
	 for( int i=0 ; i<Idx.size() ; i++ ){
		cloud_out->points.push_back( cloud_in->points[Idx[i]] );	
	 }
}

void CopyPCD( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::vector<int>& Idx,
				 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out ){

	 cloud_out->width = Idx.size();
	 cloud_out->height = 1;
	 for( int i=0 ; i<Idx.size() ; i++ ){
		cloud_out->points.push_back( cloud_in->points[Idx[i]] );	
	 }
}

void removeNaNNormalsFromPointCloud (const pcl::PointCloud<pcl::PointNormal> &cloud_in, 
									 pcl::PointCloud<pcl::PointNormal> &cloud_out,
                                     std::vector<int> &index)
{
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize (cloud_in.points.size ());
  }
  // Reserve enough space for the indices
  index.resize (cloud_in.points.size ());
  size_t j = 0;

  for (size_t i = 0; i < cloud_in.points.size (); ++i)
  {
    if (!pcl_isfinite (cloud_in.points[i].normal_x) || 
        !pcl_isfinite (cloud_in.points[i].normal_y) || 
        !pcl_isfinite (cloud_in.points[i].normal_z))
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    index[j] = static_cast<int>(i);
    j++;
  }
  if (j != cloud_in.points.size ())
  {
    // Resize to the correct size
    cloud_out.points.resize (j);
    index.resize (j);
  }

  cloud_out.height = 1;
  cloud_out.width  = static_cast<uint32_t>(j);
}

void CalcCenter( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, pcl::PointXYZ *center ){

	center->x = center->y = center->z = 0.0;
	for( size_t i=0 ; i<cloud_in->points.size() ; i++ ){
		center->x += cloud_in->points[i].x;
		center->y += cloud_in->points[i].y;
		center->z += cloud_in->points[i].z;
	}
	center->x /= (float)cloud_in->points.size();
	center->y /= (float)cloud_in->points.size();
	center->z /= (float)cloud_in->points.size();
}
void CalcCenter( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointXYZ *center ){

	center->x = center->y = center->z = 0.0;
	for( size_t i=0 ; i<cloud_in->points.size() ; i++ ){
		center->x += cloud_in->points[i].x;
		center->y += cloud_in->points[i].y;
		center->z += cloud_in->points[i].z;
	}
	center->x /= (float)cloud_in->points.size();
	center->y /= (float)cloud_in->points.size();
	center->z /= (float)cloud_in->points.size();
}

void BoundaryExtractor( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, const double rate, const double th_curvature, std::vector<int>& Idx ){

	std::vector<float>	score;

	score.resize( cloud_in->points.size() );
	for( int i=0 ; i<score.size() ; i++ ) score[i] = 0.0;

	int					nNear;
	float				radius, radius2;
	std::vector<int>	pIdxRS;
	std::vector<float>	pRSD;
	pcl::KdTreeFLANN<pcl::PointNormal, flann::L2_Simple<float> > kdtree;
	kdtree.setInputCloud( cloud_in );

	radius = 30.0;
	radius2 = radius * radius;
	for( int i=0 ; i<cloud_in->points.size() ; i++ ){
		pIdxRS.clear();
		pRSD.clear();
		nNear = kdtree.radiusSearch( cloud_in->points[i], radius, pIdxRS, pRSD );	
		for( int j=0 ; j<nNear ; j++ ){
			score[ pIdxRS[j] ] += (float)(1.0 - (pRSD[j] / radius2)); 
		}
	}

	float	max;
	max = score[0];
	for( int i=0 ; i<score.size() ; i++ ){
		if( max < score[i] ){
			max = score[i];
		}
	}
	fprintf( stderr,"max: %f\n", max );
	for( int i=0 ; i<score.size() ; i++ ){
		score[i] /= max;
	}

	std::vector<int> score_idx;
	score_idx.resize( score.size() );
	for( int i=0 ; i<score.size() ; i++ ) score_idx[i] = i;
	CombSort( score, score_idx );

	int		n_point;
	n_point = (int)(rate * (double)cloud_in->points.size());
	//輪郭部分の点群のIDの取得
	//スコアが下位の点に相当
	//for( int i=score.size()-n_point ; i<score.size() ; i++ ){
	//	Idx.push_back( score_idx[i] );
	//}
	
	//曲率の高い点のIDの取得
	for( int i=0 ; i<score.size()-n_point ; i++ ){
		if( th_curvature < cloud_in->points[ score_idx[i] ].curvature ){
			Idx.push_back( score_idx[i] );
		}
	}

	pcl::PointCloud<pcl::PointXYZHSV>::Ptr bdmap (new pcl::PointCloud<pcl::PointXYZHSV> );
	bdmap->width = cloud_in->width;
	bdmap->height = cloud_in->height;
	bdmap->points.resize( bdmap->width * bdmap->height );
	for( int i=0 ; i<bdmap->points.size() ; i++ ){
		bdmap->points[i].x = cloud_in->points[i].x;
		bdmap->points[i].y = cloud_in->points[i].y;
		bdmap->points[i].z = cloud_in->points[i].z;
		bdmap->points[i].h = 0.6*( 1.0 - (score[i] / max) );
		bdmap->points[i].s = 1.0;
		bdmap->points[i].v = 1.0;
	}
#if OUTPUT2
	pcl::io::savePCDFileASCII( "boundaryness.pcd", *bdmap );
#endif
}

// 点群をPCLのフォーマットに変換
void Pnt2PCDXYZ( const double *pnt, const int n_points, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& Idx ){

	pcl::PointXYZ tmp_pnt;
	int		cnt;
	cnt = 0;
	for( int i=0 ; i<n_points ; i++ ){
		tmp_pnt.x = (float)pnt[3*i];
		tmp_pnt.y = (float)pnt[(3*i)+1];
		tmp_pnt.z = (float)pnt[(3*i)+2];
		if( tmp_pnt.z != -1000.0 ){
			cloud->points.push_back( tmp_pnt );
			Idx.push_back( i );
			cnt++;
		}
	}
	cloud->width = cnt;
	cloud->height = 1;

}

void Pnt2PCDXYZ( const double *pnt, cv::Mat img, const int n_points, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& Idx ){

	pcl::PointXYZ tmp_pnt;
	int		cnt;
	cnt = 0;
	for( int i=0 ; i<n_points ; i++ ){
		tmp_pnt.x = (float)pnt[3*i];
		tmp_pnt.y = (float)pnt[(3*i)+1];
		tmp_pnt.z = (float)pnt[(3*i)+2];
		if( (tmp_pnt.z != -1000.0) && (img.at<uchar>( (i/WIDTH), (i%WIDTH) ) != 0 ) ){
			cloud->points.push_back( tmp_pnt );
			Idx.push_back( i );
			cnt++;
		}
	}
	cloud->width = cnt;
	cloud->height = 1;

}

void Pnt2PCDXYZ_Edge( const double *pnt, cv::Mat img, const int n_points, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<int>& Idx ){

	pcl::PointXYZ tmp_pnt;
	int		cnt;
	int		kernel, kernel_half;
	cnt = 0;

#if OUTPUT3
	cv::Mat mask = cv::Mat::zeros( img.rows, img.cols, CV_8U );
#endif

	kernel = 5;
	kernel_half = kernel/2;

	for( int j=kernel_half ; j<HEIGHT-kernel_half ; j++ ){
		for( int i=kernel_half ; i<WIDTH-kernel_half ; i++ ){

			double	center;
			if( (pnt[(3*((WIDTH*j)+i))+2] != -1000.0) && (img.at<uchar>( j, i ) != 0 ) ){
				center = pnt[(3*((WIDTH*j)+i))+2];


//				fprintf( SE,"Center(%04d, %04d) %.3lf\n", i, j, center );


				double	sum, ave, local_num;
				sum = ave = local_num = 0.0;
				int II, JJ;
				for( int jj=-kernel_half; jj<=kernel_half ; jj++ ){ 
					for( int ii=-kernel_half; ii<=kernel_half ; ii++ ){ 

						II = i + ii;
						JJ = j + jj;
						if( (pnt[(3*((WIDTH*JJ)+II))+2] != -1000.0) && (img.at<uchar>( j+jj, i+ii ) != 0) ){
							sum += fabs( center - pnt[(3*((WIDTH*JJ)+II))+2] );
							local_num += 1.0;


//							fprintf( SE," in pnt(%04d, %04d) %.3lf\n", i+ii, j+jj, pnt[(3*((WIDTH*JJ)+II))+2] );
//							fprintf( SE," center - pnt = %.3lf\n", fabs( center - pnt[(3*((WIDTH*JJ)+II))+2] ) );

						}else{
//							fprintf( SE,"   out pnt = %.3lf\n", pnt[(3*((WIDTH*JJ)+II))+2] );

						}
					}
				}
				if( 1.0 < local_num ){
					ave = sum / (local_num-1.0);


//					fprintf( SE,"      ave = %.3lf\n", ave );



					if( ave < 2.0 ){
						tmp_pnt.x = (float)pnt[(3*((WIDTH*j)+i))];
						tmp_pnt.y = (float)pnt[(3*((WIDTH*j)+i))+1];
						tmp_pnt.z = (float)pnt[(3*((WIDTH*j)+i))+2];
						cloud->points.push_back( tmp_pnt );
						Idx.push_back( i );
						cnt++;

						mask.at<uchar>( j, i ) = 255;

//						fprintf( SE,"      Save\n");

					}else{
//						fprintf( SE,"      outlier\n");

					}
				}else{


//					fprintf( SE,"      # of neighbor is too low.\n" );

				}

			}


		}
	}

	cloud->width = cnt;
	cloud->height = 1;

#if OUTPUT2
	cv::imshow( "mask 孤立点除去", mask );
	cv::imwrite( "mask.bmp", mask );
	cv::waitKey(0);
#endif
}



bool CheckSavePCD( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ){

	if( (cloud->points.size() == cloud->width*cloud->height) && (cloud->points.size() != 0 ) ){
		return true;
	}else{
		fprintf( stderr,"!!Error point cloud can not be save.\n");
		fprintf( stderr," cloud->points.size() = %d\n", cloud->points.size() );
		fprintf( stderr," cloud->width * cloud->height = %d\n", cloud->width * cloud->height );
		return false;
	}
}
bool CheckSavePCD( const pcl::PointCloud<pcl::PointNormal>::Ptr cloud ){

	if( (cloud->points.size() == cloud->width*cloud->height) && (cloud->points.size() != 0 ) ){
		return true;
	}else{
		fprintf( stderr,"!!Error point cloud can not be save.\n");
		fprintf( stderr," cloud->points.size() = %d\n", cloud->points.size() );
		fprintf( stderr," cloud->width = %d\n", cloud->width );
		fprintf( stderr," cloud->height = %d\n", cloud->height );
		return false;
	}
}
bool CheckSavePCD( const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ){

	if( (cloud->points.size() == cloud->width*cloud->height) && (cloud->points.size() != 0 ) ){
		return true;
	}else{
		fprintf( stderr,"!!Error point cloud can not be save.\n");
		fprintf( stderr," cloud->points.size() = %d\n", cloud->points.size() );
		fprintf( stderr," cloud->width * cloud->height = %d\n", cloud->width * cloud->height );
		return false;
	}
}

// 20150425-Akizuki
// dist_bin_front      ビンの前面までの距離
// camera_inclination  カメラの傾き
// bin_width		   ビンの横幅
void BinRemove( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double dist_bin_front, const double camera_inclination, const double bin_width,
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::vector<int>& Idx ){

	double	plane[4];						//ビンの前面を定義するための平面方程式
	double3	plane_abc, rot_plane_abc;		//平面方程式のabcの部分
	double	rot[3][3];
	double	dist_bin;
	double	bin_width_half;		//ビンの横幅の半分
	double	deg2rad, rad2deg;

	deg2rad = M_PI/180.0;
	rad2deg = 180.0/M_PI;

	bin_width_half = bin_width / 2.0;
	dist_bin = dist_bin_front / cos( camera_inclination * deg2rad ); //ビン前面とカメラ中心との距離
	plane[3] = dist_bin;
	plane_abc.x = 0.0;
	plane_abc.y = 0.0;
	plane_abc.z = 1.0;

	// 平面方程式を回転させる行列の設定
	rot[0][0] = 1.0; rot[0][1] = 0.0;								  rot[0][2] = 0.0;                   
	rot[1][0] = 0.0; rot[1][1] = cos( -camera_inclination*deg2rad );  rot[1][2] = -sin( -camera_inclination*deg2rad ); 
	rot[2][0] = 0.0; rot[2][1] = sin( -camera_inclination*deg2rad );  rot[2][2] = cos( -camera_inclination*deg2rad );  

	// 平面方程式を回転
	Aki3DMatrixx3DPoint( rot, plane_abc, &rot_plane_abc );
	plane[0] = rot_plane_abc.x;
	plane[1] = rot_plane_abc.y;
	plane[2] = rot_plane_abc.z;

	fprintf( SE,"Plane  param:\n  a = %.3lf, b = %.3lf, c = %.3lf, d = %.3lf\n", plane[0], plane[1], plane[2], plane[3] );
	int	cnt;
	double	dist_p2plane;
	cnt = 0;
	for( size_t i=0 ; i<cloud->points.size() ; i++ ){
		if( (-bin_width_half <cloud->points[i].x) && (cloud->points[i].x < bin_width_half) ){
			double3	v_plane2p;
			v_plane2p.x = cloud->points[i].x - 0.0;
			v_plane2p.y = cloud->points[i].y - 0.0;
			v_plane2p.z = cloud->points[i].z - dist_bin_front;
			double	ip;
			AkiInnerProduct( v_plane2p, rot_plane_abc, &ip );
			if( ip < 0 ){
				cloud_out->points.push_back( cloud->points[i] );
				Idx.push_back( i );
				cnt++;
			}
		}
	}
	cloud_out->width = cnt;
	cloud_out->height = 1;
}

// 20150425-Akizuki
// bin_width		   ビンの横幅
void BinRemove2( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double bin_width, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::vector<int>& Idx ){

	double	bin_width_half;		//ビンの横幅の半分

	bin_width_half = bin_width / 2.0;

	int	cnt;
	double	dist_p2plane;
	cnt = 0;
	for( size_t i=0 ; i<cloud->points.size() ; i++ ){
		if( (-bin_width_half <cloud->points[i].x) && (cloud->points[i].x < bin_width_half) ){
			cloud_out->points.push_back( cloud->points[i] );
			Idx.push_back( i );
			cnt++;
		}
	}
	cloud_out->width = cnt;
	cloud_out->height = 1;

}

// 20150526-秋月
// ビンの横幅使った壁の除去と，ノイズデータの削除
// bin_width		   ビンの横幅
void BinAndNoiseRemove( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double bin_width, cv::Mat im_depth, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::vector<int>& Idx ){


	// 距離画像ベースのセグメンテーション
	//画像構造体
	cv::Mat	im_orgMedian;  //平滑化した入力画像
	cv::Mat im_edgeMask;	//エッジマスク

	//パラメータ
	int m_nPrmCannyEdge;
	int m_nPrmMedFilter;
	int	m_nPrmMinSegment;

	m_nPrmCannyEdge		=	15;		// キャニーエッジ
	m_nPrmMedFilter		=	9;		// メディアンフィルタ
	m_nPrmMinSegment	=   500;

	if(dbgflg == 1)
	{
		cv::imshow( "Sub  入力画像", im_depth );
		cv::imwrite( "segclass_Sub.bmp", im_depth );
		cv::waitKey(0);
	}


	// (004) 入力距離画像を処理用メモリへコピー
	im_orgMedian	=	im_depth.clone();
	// (005) 処理用画像に２回メディアンフィルタかけて平滑化
	for(int i = 0; i < 2; i++)
	{
		medianBlur(im_orgMedian, im_orgMedian, m_nPrmMedFilter);
	}

	if(dbgflg == 1)
	{
		cv::imshow( "im_orgMedian  平滑化", im_orgMedian );
		cv::imwrite( "segclass_im_orgMedian.bmp", im_orgMedian );
		cv::waitKey(0);
	}

	// (006) 平滑化された画像を正規化
	cv::Mat im_tmp1;
	normalize(im_orgMedian, im_tmp1, 255, 0, 1, -1);

	if(dbgflg == 1)
	{
		cv::imshow( "im_tmp 正規化", im_tmp1 );
		cv::waitKey(0);
	}

	// (014) エッジ抽出(Canny) im_rMed2 -(エッジ検出)-> im_tmp1
	
	cv::Canny( im_tmp1, im_tmp1, 0, m_nPrmCannyEdge, 3);

	if(dbgflg == 1)
	{
		cv::imshow( "im_tmp エッジ抽出", im_tmp1 );
		cv::imwrite( "segclass_edge.bmp", im_tmp1 );
		cv::waitKey(0);
	}

	// (015) エッジ膨張 im_tmp1 -> im_tmp1
	cv::Mat C	=	cv::Mat::ones(3, 3, CV_32FC1);
	cv::dilate(im_tmp1, im_tmp1, C);

	if(dbgflg == 1)
	{
		cv::imshow( "im_tmp エッジ膨張", im_tmp1 );
		cv::imwrite( "segclass_dilated_edge.bmp", im_tmp1 );
		cv::waitKey(0);
	}


	// (016) エッジマスク
	im_edgeMask	=	(im_tmp1==0) - (im_orgMedian==0);

	if(dbgflg == 1)
	{
		cv::imshow( "im_edgeMask エッジマスク", im_edgeMask );
		cv::imwrite( "segclass_im_edgeMask.bmp", im_edgeMask );
		cv::waitKey(0);
	}

	//収縮
	//cv::erode( im_edgeMask, im_edgeMask, C );
	//cv::erode( im_edgeMask, im_edgeMask, C );

	//if(dbgflg == 1)
	//{
	//	cv::imshow( "im_edgeMask 収縮", im_edgeMask );
	//	cv::waitKey(0);
	//}

	// (017) ラべリング
	LabelingBS	lab;
	cv::Mat iml2(HEIGHT, WIDTH, CV_16UC1);
	lab.Exec(					// ラべリング実行関数
		im_edgeMask.data,		// 入力画像（エッジマスク）
		(short *)&iml2.data[0],	// 出力画像（ラべリング画像）
		WIDTH,						// 画像Xサイズ
		HEIGHT,						// 画像Yサイズ
		true, 
		int(m_nPrmMinSegment));		// ワーク最小サイズ（ピクセル）m_nPrmMinSegment


	cv::Mat iml;
	resize(iml2, iml, cv::Size(), 0.5, 0.5, CV_INTER_NN);
	cv::normalize(iml,iml,255*256,0,1,-1);	//正規化


	if(dbgflg == 1)
	{
		cv::imshow( "iml セグメンテーション結果", iml );
		cv::imwrite( "segclass_iml2.bmp", iml2 );
		cv::imwrite( "segclass_iml.bmp", iml );
		cv::waitKey(0);
	}










	double	bin_width_half;		//ビンの横幅の半分

	bin_width_half = bin_width / 2.0;

	int	cnt;
	double	dist_p2plane;
	cnt = 0;
	for( size_t i=0 ; i<cloud->points.size() ; i++ ){
		if( (-bin_width_half <cloud->points[i].x) && (cloud->points[i].x < bin_width_half) ){
			cloud_out->points.push_back( cloud->points[i] );
			Idx.push_back( i );
			cnt++;
		}
	}
	cloud_out->width = cnt;
	cloud_out->height = 1;

}




void PCDXYZ2CVMat_Mask( const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double pitch, double3 trans, int fill, cv::Mat *img ){

	int				imgX, imgY;

	for( size_t i=0 ; i<cloud->points.size() ; i++ ){
		imgX = (int)(((cloud->points[i].x + trans.x)/pitch)+0.5);
		imgY = (int)(((cloud->points[i].y + trans.y)/pitch)+0.5);

		if( (0<=imgX) && (0<=imgY) && (imgX<img->cols) && (imgY<img->rows) ){
			img->at<uchar>( imgY, imgX ) = 255;
			//ここから下は穴埋め処理(8近傍)
			if( fill ){
				if( imgX+1<img->cols ){
					img->at<uchar>( imgY, imgX+1 ) = 255;
				}
				if( 0<imgX-1 ){
					img->at<uchar>( imgY, imgX-1 ) = 255;
				}
				if( imgY+1<img->rows ){
					img->at<uchar>( imgY+1, imgX ) = 255;
				}
				if( 0<imgY-1 ){
					img->at<uchar>( imgY-1, imgX ) = 255;
				}
			}
		}
	}

}

void SegmentationNoiseReduction( cv::Mat im_in, cv::Mat im_out ){

		// 距離画像ベースのセグメンテーション
	//画像構造体
	cv::Mat	im_orgMedian;  //平滑化した入力画像
	cv::Mat im_edgeMask;	//エッジマスク

	//パラメータ
	int m_nPrmCannyEdge;
	int m_nPrmMedFilter;
	int	m_nPrmMinSegment;

	m_nPrmCannyEdge		=	15;		// キャニーエッジ
	m_nPrmMedFilter		=	9;		// メディアンフィルタ
	m_nPrmMinSegment	=   500;

	if(dbgflg == 1)
	{
		cv::imshow( "Sub  入力画像", im_in );
		cv::imwrite( "segclass_Sub.bmp", im_in );
		cv::waitKey(0);
	}


	// (004) 入力距離画像を処理用メモリへコピー
	im_orgMedian	=	im_in.clone();
	// (005) 処理用画像に２回メディアンフィルタかけて平滑化
	for(int i = 0; i < 2; i++)
	{
		medianBlur(im_orgMedian, im_orgMedian, m_nPrmMedFilter);
	}

	if(dbgflg == 1)
	{
		cv::imshow( "im_orgMedian  平滑化", im_orgMedian );
		cv::imwrite( "segclass_im_orgMedian.bmp", im_orgMedian );
		cv::waitKey(0);
	}

	// (006) 平滑化された画像を正規化
	cv::Mat im_tmp1;
	normalize(im_orgMedian, im_tmp1, 255, 0, 1, -1);

	if(dbgflg == 1)
	{
		cv::imshow( "im_tmp 正規化", im_tmp1 );
		cv::waitKey(0);
	}

	// (014) エッジ抽出(Canny) im_rMed2 -(エッジ検出)-> im_tmp1
	
	cv::Canny( im_tmp1, im_tmp1, 0, m_nPrmCannyEdge, 3);

	if(dbgflg == 1)
	{
		cv::imshow( "im_tmp エッジ抽出", im_tmp1 );
		cv::imwrite( "segclass_edge.bmp", im_tmp1 );
		cv::waitKey(0);
	}

	// (015) エッジ膨張 im_tmp1 -> im_tmp1
	cv::Mat C	=	cv::Mat::ones(3, 3, CV_32FC1);
	cv::dilate(im_tmp1, im_tmp1, C);

	if(dbgflg == 1)
	{
		cv::imshow( "im_tmp エッジ膨張", im_tmp1 );
		cv::imwrite( "segclass_dilated_edge.bmp", im_tmp1 );
		cv::waitKey(0);
	}


	// (016) エッジマスク
	im_edgeMask	=	(im_tmp1==0) - (im_orgMedian==0);

	if(dbgflg == 1)
	{
		cv::imshow( "im_edgeMask エッジマスク", im_edgeMask );
		cv::imwrite( "segclass_im_edgeMask.bmp", im_edgeMask );
		cv::waitKey(0);
	}

	//収縮
	//cv::erode( im_edgeMask, im_edgeMask, C );
	//cv::erode( im_edgeMask, im_edgeMask, C );

	//if(dbgflg == 1)
	//{
	//	cv::imshow( "im_edgeMask 収縮", im_edgeMask );
	//	cv::waitKey(0);
	//}

	// (017) ラべリング
	LabelingBS	lab;
	cv::Mat iml2(HEIGHT, WIDTH, CV_16UC1);
	lab.Exec(					// ラべリング実行関数
		im_edgeMask.data,		// 入力画像（エッジマスク）
		(short *)&iml2.data[0],	// 出力画像（ラべリング画像）
		WIDTH,						// 画像Xサイズ
		HEIGHT,						// 画像Yサイズ
		true, 
		int(m_nPrmMinSegment));		// ワーク最小サイズ（ピクセル）m_nPrmMinSegment


	for( int j=0 ; j<HEIGHT ; j++ ){
		for( int i=0 ; i<WIDTH ; i++ ){
			if( iml2.at<short>( j, i) != 0 ){
				im_out.at<uchar>( j, i ) = 255;
			}
		}
	}


		if(dbgflg == 1)
	{
		cv::imshow( "im_out セグメンテーション結果2", im_out );
		cv::imwrite( "segclass_im_out.bmp", im_out );
		cv::waitKey(0);
	}
}