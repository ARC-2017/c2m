#ifndef INCLUDE_algorithm_table_h_
#define INCLUDE_algorithm_table_h_

//======================================================//
// アイテムごとに得意なアルゴリズムで認識を実行する．
// このヘッダのテーブルを参照することで，アルゴリズムが
// 決定される．
//======================================================//

#include "common.h"

// [ビンの種類][アイテムID]の順番．
// アイテムID=0は使われていないので，
// 配列は26個用意し，1-25番を利用します．
									   /*Single Bin*/ /*Double Bin*/ /*Multi Bin*/
unsigned char method[26][3] = { /* 0*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_SIMPLE}, /*とりあえずシンプルを入れておく*/
								/* 1*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_SIMPLE},
								/* 2*/{METHOD_WHF   , METHOD_WHF   , METHOD_WHF},
								/* 3*/{METHOD_WHF   , METHOD_WHF   , METHOD_WHF},
								/* 4*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_SIMPLE},  
								/* 5*/{METHOD_WHF   , METHOD_WHF   , METHOD_SIMPLE},
								/* 6*/{METHOD_VPM   , METHOD_VPM   , METHOD_VPM},
								/* 7*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_SIMPLE},
								/* 8*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_WHF},
								/* 9*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_SIMPLE},
								/*10*/{METHOD_WHF   , METHOD_WHF   , METHOD_WHF},
								/*11*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_SIMPLE},
								/*12*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_SIMPLE},
								/*13*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_SIMPLE},
								/*14*/{METHOD_WHF   , METHOD_WHF   , METHOD_WHF},
								/*15*/{METHOD_WHF   , METHOD_WHF   , METHOD_SIMPLE},
								/*16*/{METHOD_WHF   , METHOD_WHF   , METHOD_WHF},
								/*17*/{METHOD_WHF   , METHOD_WHF   , METHOD_WHF},
								/*18*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_WHF},
								/*19*/{METHOD_VPM   , METHOD_VPM   , METHOD_SIMPLE},
								/*20*/{METHOD_VPM   , METHOD_VPM   , METHOD_VPM},
								/*21*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_SIMPLE},
								/*22*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_SIMPLE},
								/*23*/{METHOD_WHF   , METHOD_WHF   , METHOD_SIMPLE},
								/*24*/{METHOD_VPM   , METHOD_VPM   , METHOD_SIMPLE},
								/*25*/{METHOD_SIMPLE, METHOD_SIMPLE, METHOD_SIMPLE}
};


// C-FASTが動くかどうかを決定する．
unsigned char C_FAST_SW[26] = { 
				/* 0*/ 		OFF,
				/* 1*/ ON,
				/* 2*/ ON,
				/* 3*/ ON,
				/* 4*/ ON,
				/* 5*/ 		OFF,
				/* 6*/ OFF,
				/* 7*/ 		OFF,
				/* 8*/ ON,
				/* 9*/ ON,
				/*10*/ 		OFF,
				/*11*/ ON,
				/*12*/ ON,
				/*13*/ ON,
				/*14*/ ON,
				/*15*/ ON,
				/*16*/ ON,
				/*17*/		OFF,
				/*18*/ ON,
				/*19*/ ON,
				/*20*/ 		OFF,
				/*21*/ 		OFF,
				/*22*/ ON,
				/*23*/ ON,
				/*24*/ 		OFF,
				/*25*/ ON
};

#endif
