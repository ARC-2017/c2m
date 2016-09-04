#define PARAM_DIR "C:\\Temp\\RecgAPCcnn_params"	/* CNNの学習パラメータのディレクトリ */

#define NUM_OF_CONV_LAYER	5  /* 畳み込み層の数 */
#define NUM_OF_FC_LAYER		3  /* 全結合層の数 */
#define NUM_OF_BAT_NORM		2  /* バッチ正規化層の数 */

#define PATCH_SIZE			165
#define CNN_PATCH_SIZE		227

/* 畳み込み層の重みフィルタ, バイアス, 特徴マップのクラス */
class CONVOLUTION_LAYER{
public:

	/* 重みフィルタ */
	std::vector< std::vector<cv::Mat> > conv_w;
	
	/* バイアス */
	std::vector<float> conv_b;
	
	/* 出力特徴マップ */
	std::vector<cv::Mat> feature_map;

};

/* 全結合層の結合重み, バイアス, 応答値ユニットのクラス */
class FULL_CONECTION_LAYER{
public:

	/* 結合重み */
	std::vector<cv::Mat> fc_w;
	
	/* バイアス */
	std::vector<float> fc_b;
	
	/* 出力応答値ユニット */
	cv::Mat response_unit;

};

/* バッチ正規化のγ係数とβ係数 */
class BATCH_NORMALIZATION_LAYER{
public:

	/* γ係数 */
	std::vector<float> bn_g;

	/* β係数 */
	std::vector<float> bn_b;

	/* 平均値 */
	std::vector<float> bn_mean;

	/* 分散値 */
	std::vector<float> bn_var;

};