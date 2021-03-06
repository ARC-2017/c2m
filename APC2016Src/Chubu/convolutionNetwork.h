
/* 畳み込み処理関数 */
bool convolutionLayer(std::vector<cv::Mat> input_map, std::vector<cv::Mat> &output_map, std::vector< std::vector<cv::Mat> > conv_w, std::vector<float> bias, int stride, int pad);

/* Batch Normalization関数 */
bool batchNormalization(std::vector<cv::Mat> &feature_map, std::vector<float> gamma, std::vector<float> beta, std::vector<float> mean, std::vector<float> var);

/* ReLU関数 */
bool convReLU(std::vector<cv::Mat> &feature_map);

/* Maxpooling関数 */
bool maxPooling(std::vector<cv::Mat> &feature_map, int pooling_stride, int pooling_size);

