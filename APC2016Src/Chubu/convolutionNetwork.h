
/* ô‚İ‚İˆ—ŠÖ” */
bool convolutionLayer(std::vector<cv::Mat> input_map, std::vector<cv::Mat> &output_map, std::vector< std::vector<cv::Mat> > conv_w, std::vector<float> bias, int stride, int pad);

/* Batch NormalizationŠÖ” */
bool batchNormalization(std::vector<cv::Mat> &feature_map, std::vector<float> gamma, std::vector<float> beta, std::vector<float> mean, std::vector<float> var);

/* ReLUŠÖ” */
bool convReLU(std::vector<cv::Mat> &feature_map);

/* MaxpoolingŠÖ” */
bool maxPooling(std::vector<cv::Mat> &feature_map, int pooling_stride, int pooling_size);

