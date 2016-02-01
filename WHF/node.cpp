#include "Node.h"

/*!
@brief デフォルトコンストラクタ
@return void
*/
Node::Node( void ){

	Centroid = cv::Point(0, 0);
	threshold = 0.0;
	depth = 0;
	index = 0;
	nodeType = NULL;
	right = NULL;
	left = NULL;
	return;
}