#pragma once
//// 大学側認識処理
//// 入力データ
////	**binIdx	ビン内の物体IDデータ
////	*binNum		ビン内の物体数
////	*itemIdx	認識対象物体のID
////	orderBin	オーダーのあったビンのID
////	*pnt		ポイントクラウド		
////　	*depth		距離画像
////	*color		RGB画像
////    backgroudID 視点情報．奇数なら斜め，偶数なら正面視点を想定します．
////
//// 出力データ
////    識別対象物に関する情報
////	work_i		認識対象物上の点のi座標
////	work_j		認識対象物上の点のj座標
////	work_score	点（ work_i, work_j ）の信頼度
////	cluster		認識結果のセグメントが格納されます．1280x960の1次元配列です．
////				0, 255の二値画像です．
////    識別対象物'以外'に関する情報
////	nt_itemIdx	認識対象物以外のID
////	nt_i		認識対象物以外の点のi座標配列
////	nt_j		認識対象物以外の点のj座標配列
////	nt_score	点（ nt_i, nt_j ）の信頼度
////	nt_cluster	認識対象物以外のセグメントが格納されます．1280x960の1次元配列です．
////				0, 255の二値画像です．
////　　信頼度マップ
////    c_map       各セグメントの信頼度が格納されます．1280x960の1次元配列です．値の範囲は[1,100]です．
////				ただし，ターゲットアイテムの得点は+100されており，最大値が200になります．
////    IDマップ	1280x960の位置次元配列に，アイテムIDが振ってあります．
////		

//bool RecgAPC2016( int **binIdx, int *binNum, int *itemIdx, int orderBin, double *pnt, unsigned char *depth, unsigned char *color, int backgroundID,
//				 std::vector<int>& work_i,  std::vector<int>& work_j,  std::vector<double>& work_score, unsigned char *cluster, 
//				 std::vector<int>& nt_itemIdx, std::vector<int>& nt_i, std::vector<int>& nt_j, std::vector<double>& nt_score, unsigned char *nt_cluster,
//				 unsigned char *cmap, unsigned char *id_map );
bool RecgAPC2016( int *nItemIndices, int nItemNum, int nTargetItemIdx, int nTargetBinIdx, double *pnt, unsigned char *depth, unsigned char *color, int backgroundID,
				 std::vector<int>& work_i,  std::vector<int>& work_j,  std::vector<double>& work_score, unsigned char *cluster, 
				 std::vector<int>& nt_itemIdx, std::vector<int>& nt_i, std::vector<int>& nt_j, std::vector<double>& nt_score, unsigned char *nt_cluster,
				 unsigned char *c_map, unsigned char *id_map);


// 降順ソート
bool CombSort( const std::vector<double>& values, std::vector<int>& Idx );