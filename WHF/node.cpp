#include "Node.h"

/*!
@brief �f�t�H���g�R���X�g���N�^
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