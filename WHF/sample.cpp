#include "sample.h"
using namespace std;


//学習サンプルのデータ入力
bool instance::insert( int _BagId, Mat _patch, int _label, int _subLabel, int _plane, Point _offset, Point _cutP, dot_feature* _dot, int _angle, int _Id, int _instanceId, double _pi, double _pij, double _weight ){

	BagId			= _BagId;
	instanceId	= _instanceId;
	patch			= _patch;
	label			= _label;
	subLabel		= _subLabel;
	plane			= _plane;
	offset		= _offset;
	cutP			= _cutP;
	angle			= _angle;
	Id				= _Id;
	pi				= _pi;
	pij			= _pij;
	weight		= _weight;

	if( label == 4 ){
		printf( "BagId:%d label:%d angle:%d Id:%d\n", BagId, label, angle, Id );
		cv::imshow( "パッチ", patch );
		cv::waitKey();
	}

	for(int cell = 0; cell < CELL_NUM; cell++){

		dot[cell].feature0_180 = _dot[cell].feature0_180;
	}

	return true;
}

//テストサンプルのデータ入力
bool testSample::insert( Mat _patch, Point _point, dot_feature* _dot ){
	patch = _patch;
	point = _point;

	for(int cell = 0; cell < CELL_NUM; cell++){

		dot[cell].feature0_180 = _dot[cell].feature0_180;
	}

	return true;
}

/*!
@brief コンソールへの出力
@return true
*/
bool testSample::console_out( void ){
	
	string filename = "patchImage";
	namedWindow( filename );
	imshow( filename, patch );
	if(waitKey( 30 ) == 27){
		destroyWindow(filename);
		return true;
	}
	destroyWindow( filename );
	cout << "Point = " << point << endl;
	
	return true;
}


testSample::testSample(void)
{
}

testSample::~testSample(void)
{
}