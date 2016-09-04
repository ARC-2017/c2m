#define PARAM_DIR "C:\\Temp\\RecgAPCcnn_params"	/* CNN�̊w�K�p�����[�^�̃f�B���N�g�� */

#define NUM_OF_CONV_LAYER	5  /* ��ݍ��ݑw�̐� */
#define NUM_OF_FC_LAYER		3  /* �S�����w�̐� */
#define NUM_OF_BAT_NORM		2  /* �o�b�`���K���w�̐� */

#define PATCH_SIZE			165
#define CNN_PATCH_SIZE		227

/* ��ݍ��ݑw�̏d�݃t�B���^, �o�C�A�X, �����}�b�v�̃N���X */
class CONVOLUTION_LAYER{
public:

	/* �d�݃t�B���^ */
	std::vector< std::vector<cv::Mat> > conv_w;
	
	/* �o�C�A�X */
	std::vector<float> conv_b;
	
	/* �o�͓����}�b�v */
	std::vector<cv::Mat> feature_map;

};

/* �S�����w�̌����d��, �o�C�A�X, �����l���j�b�g�̃N���X */
class FULL_CONECTION_LAYER{
public:

	/* �����d�� */
	std::vector<cv::Mat> fc_w;
	
	/* �o�C�A�X */
	std::vector<float> fc_b;
	
	/* �o�͉����l���j�b�g */
	cv::Mat response_unit;

};

/* �o�b�`���K���̃��W���ƃ��W�� */
class BATCH_NORMALIZATION_LAYER{
public:

	/* ���W�� */
	std::vector<float> bn_g;

	/* ���W�� */
	std::vector<float> bn_b;

	/* ���ϒl */
	std::vector<float> bn_mean;

	/* ���U�l */
	std::vector<float> bn_var;

};