#pragma once
// 大学側認識処理
// 出力データの説明．
// fla:     認識したアルゴリズムの番号が返却されます．未使用？
// workPos: 物体位置です．画像座標の ( workPos[0], workPos[1] ) 
// cluster: 物体に属する距離データが格納されます．1280x960の1次元配列です．
//          距離データのない部分の画素値は0です．
bool Recg( int **binIdx, int *binNum, int *itemIdx, double *pnt, unsigned char *depth, unsigned char *color, int *flag, 
		   double *workPos, unsigned char *cluster );
// APC 20140414 秋月追記　↑↑↑