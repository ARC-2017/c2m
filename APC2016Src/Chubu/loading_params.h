
bool loadingConvolutionParams(char conv_w_path[256], char conv_b_path[256], std::vector< std::vector<cv::Mat> > &conv_w, std::vector<float> &bias);

bool loadingFullConectionParams(char fc_w_path[256], char fc_b_path[256], std::vector<cv::Mat> &fc_w, std::vector<float> &bias);

bool loadingBatchNormalizationParams(char bn_g_path[256], char bn_b_path[256], char bn_m_path[256], char bn_v_path[256], std::vector<float> &gamma, std::vector<float> &beta, std::vector<float> &mean, std::vector<float> &var);