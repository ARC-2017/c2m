#define _CRT_SECURE_NO_WARNINGS
#include "Decisiontree.h"

static const string output = OUTPUT_PATH;
static const double LOG10_2 = 0.301;
using namespace std;

ofstream ofs1( "./Weight_TXT/weight.txt" );

int face2[] = {cv::FONT_HERSHEY_SIMPLEX, cv::FONT_HERSHEY_PLAIN, cv::FONT_HERSHEY_DUPLEX, cv::FONT_HERSHEY_COMPLEX, 
              cv::FONT_HERSHEY_TRIPLEX, cv::FONT_HERSHEY_COMPLEX_SMALL, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 
              cv::FONT_HERSHEY_SCRIPT_COMPLEX, cv::FONT_ITALIC};


//分岐関数
inline bool Decisiontree::splitFunction(const instance* &samples, const instance* &Tsamples, float &th){

	unsigned int comparison0_180[CELL_NUM];
	unsigned int comparison180_360[CELL_NUM];
	float sum = CELL_NUM;
	float distance = 0.0;

	int Sample0_180;
	int SampleAndTemplate0_180;
	int Template0_180;


	for(int cell = 0; cell < CELL_NUM; cell++){

		//サンプルとテンプレートのDOTをAND演算
		comparison0_180[cell] = samples->dot[cell].feature0_180 & Tsamples->dot[cell].feature0_180;
		comparison180_360[cell] = samples->dot[cell].feature180_360 & Tsamples->dot[cell].feature180_360;

		Sample0_180 = (int)BITS_COUNT_TABLE[samples->dot[cell].feature0_180];
		Template0_180 = (int)BITS_COUNT_TABLE[Tsamples->dot[cell].feature0_180];
		SampleAndTemplate0_180 = (int)BITS_COUNT_TABLE[comparison0_180[cell]];

		//両者の特徴量の値が0の場合は除外
		if( Sample0_180 == 0 && Template0_180 == 0 ){

			sum--;
		}

		if( Template0_180 > 0 ){

			distance += ( SampleAndTemplate0_180 ) / (float) Template0_180;
		}
	}

	if(sum == 0.0){

		distance = 0.0;
	}else{
	
		distance = distance / sum;
	}

	//分岐条件
	if(distance < th ){

		return true;
	}else{

		//return true;
		return false;
	}
}

//分岐関数の作成
Node * Decisiontree::createNode(  
									 Node *node, 
									 const vector<const instance *> &samples, 
									 const vector<const instance *> &Tsamples,
									 vector<double> &il,
									 int depth,
									 int index
									 )
{

	if(samples.empty()  || Tsamples.empty()){

		printf("サンプル数：%d テンプレート数：%d", samples.size(), Tsamples.size());
		return NULL;
	}


	// 分割結果の保持用バッファ
	std::vector<const instance *> lSamples;
	std::vector<const instance *> rSamples;
	std::vector<const instance *> TemplateCandidate;

	// 必要になり得るサイズの最大値分のバッファを確保しておく
	lSamples.reserve(samples.size());
	rSamples.reserve(samples.size());
	node->lSamples.reserve(samples.size());
	node->rSamples.reserve(samples.size());

	//サンプルのDOT特徴保持用の配列
	unsigned int bestFeature0_180[CELL_NUM];
	unsigned int bestFeature180_360[CELL_NUM];
	unsigned int bestFeatureOri[CELL_NUM];
	
	for( int i = 0; i < samples.size(); i++ ){

		if( samples[ i ]->label == 1 ){

			TemplateCandidate.push_back( samples[ i ] );
		}
	}

	if( TemplateCandidate.size() == 0 ){

		node->nodeType = false;
		node->weightDistribution.resize(numClass, 0.0);
		node->weightSubDistribution.resize( numClass, 0.0 );

		return node;
	}


	// 評価値を最小化する分割結果を探索する
	double bestGain = DBL_MAX;				
	float bestThreshold = 0;
	int bestTrand_id = 0;
	std::vector<const instance *> bestlSamples;
	std::vector<const instance *> bestrSamples;
	const instance* bestTemplate;
	const instance* Template;
	bestlSamples.reserve(samples.size());
	bestrSamples.reserve(samples.size());


	//ノードに保存するテンプレートをポジティブサンプルからランダムに選択
	for(int ft=0; ft<featureTests; ++ft){

		int Trand_id = 
			(int)(0+(float)rand()/(float)(RAND_MAX) * ((TemplateCandidate.size() - 1) - 0));

		if( TemplateCandidate.size() < Trand_id ){
			printf("要素数：%d 番号：%d\n", TemplateCandidate.size(), Trand_id);
		}
		Template = TemplateCandidate[Trand_id];

		for(int tt=0; tt<thresholdTest; ++tt){

			// 分岐につかうしきい値を特徴量の最大・最小の間でランダムに決定
			float rand_threshold = float(0.0+(float)rand()/(float)(RAND_MAX)
			                       *((float)1.0 - 0.0 ));

			//
			// 上記のランダムなしきい値で分割を試す
			//
			lSamples.clear();
			rSamples.clear();

			//サンプルを分割
			for(unsigned int s=0; s<(int)samples.size(); ++s){

				const instance* tmp;
				tmp = samples[s];
				if(splitFunction(tmp, Template, rand_threshold)){

					//	左子ノードのサンプルに追加
					lSamples.push_back(samples[s]);
				}else{

					//	右の子ノードにサンプルを追加
					rSamples.push_back(samples[s]);
				}
			}

			//
			//gainSwitch により評価関数の選択
			//
			double gain;
			if( depth % 2 == 0 && (int)samples.size() > SPLIT_SELECT_MIN ){

				// 情報利得の算出
				gain = computeInformationGain(lSamples, rSamples, il);
			}else{

				// 分散の算出
				gain = computeVariance(lSamples, rSamples);
			}


			// 評価値の最小値を探索
			if(gain < bestGain){

				bestGain = gain;
				bestThreshold = rand_threshold;
				bestTemplate = Template;
				bestTrand_id = Trand_id;
				bestlSamples = lSamples;
				bestrSamples = rSamples;
				for(int c = 0; c < CELL_NUM; c++){

					bestFeature0_180[c] = Template->dot[c].feature0_180;
					bestFeature180_360[c] = Template->dot[c].feature180_360;
					bestFeatureOri[c] = Template->dot[c].featureOri;
				}
			}
		} //ttループend
	}	//ftループend

	// 最良だったパラメータをノードに保存
	node->threshold = bestThreshold;
	node->lSamples = bestlSamples;
	node->rSamples = bestrSamples;
	node->Template = bestTemplate;
	node->depth = depth;
	node->index = index;
	for(int c = 0; c < CELL_NUM; c++){
		node->feature0_180[c] = bestFeature0_180[c];
		node->feature180_360[c] = bestFeature180_360[c];
		node->featureOri[c] = bestFeatureOri[c];
	}

	//ノードのパッチを保存
	for( int s = 0; s < samples.size(); s++ ){

		node->allSamples.push_back(const_cast<instance*>( samples[s] ));

		//ポジティブパッチのみを保存
		if( samples[s]->label == POS_LABEL ){

			node->samples.push_back( samples[s] );
		}
	}
	
	//末端ノードに到達したか判定
	if(DBL_MIN > bestGain || maxDepth < depth || bestlSamples.empty() || bestrSamples.empty() || samples.size() < SAMPLE_MIN){
		
		node->nodeType = false;
		node->weightDistribution.resize(numClass, 0.0);
		node->weightSubDistribution.resize( numClass, 0.0 );

		if( samples.size() == 0 ){

			printf("no sample\n");
			return NULL;
		}
	//分岐ノード
	}else{

		node->nodeType = true;
	}

	return node;
}

//決定木の作成を行う関数
Node * Decisiontree::buildVerWeight( const vector<const instance *> &samples, 
									 const vector<const instance *> &Tsamples,
									 vector<double> &il, 
									 vector<double> &il_sub, 
									 int depth, 
									 int index
									 )
{


	int pos_n = 0;
	for( int s = 0; s < samples.size(); s++ ){

		if( samples[ s ]->label == POS_LABEL ){

			pos_n++;
		}
	}
	cout << "サンプルの総数：" << samples.size() << " ポジティブサンプル数：" << pos_n << endl;

	if(samples.empty()  || Tsamples.empty()){

		return NULL;
	}
	
	//ノード保存用のキュー
	queue<Node *> splitNode;
	queue<Node *> leafNode;

	Node *node = new Node();
	node->nodeType = true;
	Node *root;
	root = node;

	//ルートノードを作成
	node = createNode( node, samples, Tsamples, il, 0, 0 );
	splitNode.push( node );

	int nowDepth = 0;
	int nodeNum = 0;
	//終了条件(学習の終了)に達するまで無限ループ
	printf("\n");
	while(true){
		cout << "深さ：" << nowDepth << " ノード：" << nodeNum << "\r";

		// 学習終了
		if( splitNode.size() == NULL ){

			//重み更新関数
//			weightUpdate( splitNode, leafNode, negRate, sub );
//			weightNormalization( samples );

			break;
		}

		//splitNodeの先頭要素を取り出す
		node = splitNode.front();

		//階層の切り替わりを判定
		if( nowDepth != node->depth ){

			//重み更新関数
			weightUpdate( splitNode, leafNode );
			weightNormalization( samples );

			nowDepth++;
			nodeNum = 0;
		}
		
		//左の子ノードのポインタが無い場合，ノードを生成
		if( node->left == NULL ){

			//printf("左の子ノード作成中...\n");
			node->left = new Node();
			node->left = createNode( node->left, node->lSamples, Tsamples, il, node->depth+1, node->index+1 );

			//nodeTypeに応じたキューにnode->leftをスタック
			if( node->left->nodeType == true ){

				splitNode.push( node->left );
			}else{

				leafNode.push( node->left );
			}
			
		//右の子ノードのポインタが無い場合，ノードを生成
		}else if( node->right == NULL ){

			//printf("右の子ノード作成中...\n");
			node->right = new Node();
			node->right = createNode( node->right, node->rSamples, Tsamples, il, node->depth+1, node->index+2 );

			//nodeTypeに応じたキューにnode->leftをスタック
			if( node->right->nodeType == true ){

				splitNode.push( node->right );
			}else{

				leafNode.push( node->right );

			}
		}else if( node->left != NULL && node->right != NULL ){

			//不要となったサンプルを削除
			if( node->nodeType == true ){
				node->allSamples.clear();
				node->samples.clear();
				node->lSamples.clear();
				node->rSamples.clear();
			}

			//左右の子ノードを作成したので親ノードをキューから削除
			splitNode.pop();
		}
		nodeNum++;
	}
	return root;
}

//重みの正規化
inline bool Decisiontree::weightNormalization( const vector<const instance *> &samples ){
	
	double tatal = 0.0;
	double tatal_sub = 0.0;
	double max = DBL_MIN;
	double min = DBL_MAX;
	double max_sub = DBL_MIN;
	double min_sub = DBL_MAX;

	//重みの総和を算出
	for( int s = 0; s < samples.size(); s++ ){

		tatal += samples[s]->weight;
		if( samples[s]->label == POS_LABEL ){

			tatal_sub += samples[s]->weight_sub;

			if( max_sub < samples[s]->weight_sub ){

				max_sub = samples[s]->weight_sub;
			}
			if( min_sub > samples[s]->weight_sub ){

				min_sub = samples[s]->weight_sub;
			}
		}

		if( max < samples[s]->weight ){

			max = samples[s]->weight;
		}
		if( min > samples[s]->weight ){

			min = samples[s]->weight;
		}
	}

	//正規化
	for( int s = 0; s < samples.size(); s++ ){

		if( ( max - min ) != 0.0 ){

			double norm = 1.0 * (  ( ( samples[s]->weight - min ) + 0.01 ) / ( max - min ) );
			const_cast<instance*>(samples[s])->weight = norm;

		}


		if( samples[s]->label == POS_LABEL ){

			if( ( max_sub - min_sub ) != 0.0 ){
	
				double norm_sub = 1.0 * ( ( ( samples[s]->weight_sub - min_sub ) + 0.01 ) / ( max_sub - min_sub ) );
				const_cast<instance*>(samples[s])->weight_sub = norm_sub;

			}
		}
	}


	return true;
}


//重み更新
inline bool Decisiontree::weightUpdate( queue<Node *> &splitNode, queue<Node *> &leafNode ){

	Node *tmpNode;

	//分岐ノードに対する処理
	size_t splitNodeNum = splitNode.size();
	if( splitNode.size() != NULL ){
		 
		ofs1 << "\n" << endl;
		for( int i = 0; i < splitNodeNum; i++ ){

			tmpNode = splitNode.front();

			//重みつき確率を算出
			vector<double> dist(numClass, 0.0);
			for(unsigned int s=0; s < tmpNode->allSamples.size(); s++){

				dist[tmpNode->allSamples[s]->label] += tmpNode->allSamples[s]->weight;
			}
			double total = 0.0;

			for( int i = 0; i < numClass; i++ ){

				total = total + dist[ i ];
			}
			for( int i = 0; i < numClass; i++ ){

				if( dist[ i ] != 0.0 ){
					dist[ i ] = dist[ i ] / total;
				}
			}
			//重みつき確率を算出(サブクラス)
			vector<double> dist_sub(numClass, 0.0);
			for(unsigned int s=0; s < tmpNode->allSamples.size(); s++){

				if( tmpNode->allSamples[ s ]->label == POS_LABEL ){

					dist_sub[tmpNode->allSamples[s]->subLabel] += tmpNode->allSamples[s]->weight_sub;
				}
			}
			total = 0.0;

			for( int i = 0; i < numClass; i++ ){

				total = total + dist_sub[ i ];
			}
			for( int i = 0; i < numClass; i++ ){

				if( dist_sub[ i ] != 0.0 ){
					dist_sub[ i ] = dist_sub[ i ] / total;
				}
			}

			//重み更新
			double ep = 0.001;
			int pos_cnt = 0;
			for(unsigned int s=0; s < tmpNode->allSamples.size(); s++){

				double F = 1.0 / 2.0 * log( ( dist[ POS_LABEL ] + ep ) / ( dist[ NEG_LABEL ] + ep ) );
				double tmpW = tmpNode->allSamples[s]->weight;
				int label = -1;
				if( tmpNode->allSamples[s]->label == POS_LABEL ){

					label = 1;
				}
				const_cast<instance*>(tmpNode->allSamples[s])->weight = tmpW * exp( label * F );

				int sub_label = -1;
				if( tmpNode->allSamples[s]->label == POS_LABEL  ){

					sub_label = 1;
				}
				if( tmpNode->allSamples[s]->label == POS_LABEL ){
						
					double F_sub = 1.0 / 2.0 * log( ( dist_sub[ POS_LABEL ] + ep ) / ( dist_sub[ NEG_LABEL ] + ep ) );
					tmpW = tmpNode->allSamples[s]->weight_sub;
					const_cast<instance*>(tmpNode->allSamples[s])->weight_sub = tmpW * exp( sub_label * F_sub );
				}
			}

			splitNode.push( tmpNode );
			splitNode.pop();
		}
	}

	//末端ノードに対する処理
	size_t leafNodeNum = leafNode.size();
	if( leafNode.size() != NULL ){
		ofs1 << "\n" << endl;
		for( int i = 0; i < leafNodeNum; i++ ){

			tmpNode = leafNode.front();

			//重みつき確率を算出
			vector<double> dist(numClass, 0.0);
			for(unsigned int s=0; s < tmpNode->allSamples.size(); s++){

				dist[tmpNode->allSamples[s]->label] += tmpNode->allSamples[s]->weight;
			}
			double total = 0.0;

			for( int i = 0; i < numClass; i++ ){

				total = total + dist[ i ];
			}
			for( int i = 0; i < numClass; i++ ){

				if( dist[ i ] != 0.0 ){
					dist[ i ] = dist[ i ] / total;
				}
			}

			//重みつき確率を算出(サブクラス)
			vector<double> dist_sub(numClass, 0.0);
			for(unsigned int s=0; s < tmpNode->allSamples.size(); s++){

				if( tmpNode->allSamples[ s ]->label == POS_LABEL ){

					dist_sub[tmpNode->allSamples[s]->subLabel] += tmpNode->allSamples[s]->weight_sub;
				}
			}
			total = 0.0;

			for( int i = 0; i < numClass; i++ ){

				total = total + dist_sub[ i ];
			}
			for( int i = 0; i < numClass; i++ ){

				if( dist_sub[ i ] != 0.0 ){
					dist_sub[ i ] = dist_sub[ i ] / total;
				}
			}

			//重み更新
			double ep = 0.001;
			int pos_cnt = 0;
			for(unsigned int s=0; s < tmpNode->allSamples.size(); s++){

				double F = 1.0 / 2.0 * log( ( dist[ POS_LABEL ] + ep ) / ( dist[ NEG_LABEL ] + ep ) );
				double tmpW = tmpNode->allSamples[s]->weight;
				int label = -1;
				if( tmpNode->allSamples[s]->label == POS_LABEL ){

					label = 1;
				}
				const_cast<instance*>(tmpNode->allSamples[s])->weight = tmpW * exp( label * F );
			}

			leafNode.push( tmpNode );
			leafNode.pop();

		}
	}



	return true;
}

//情報利得の計算
inline double Decisiontree::computeInformationGain( std::vector<const instance *> &lSamples, std::vector<const instance *> &rSamples, vector<double> &il ){
	vector<double> ldist(numClass, 0.0);
	vector<double> rdist(numClass, 0.0);

	// 左の確率密度関数を作成
	for(unsigned int s=0; s<lSamples.size(); s++){

		ldist[lSamples[s]->label] += lSamples[s]->weight;
	}

	// 右の確率密度関数を作成
	for(unsigned int s=0; s<rSamples.size(); s++){

		rdist[rSamples[s]->label] += rSamples[s]->weight;
	}

	// 左右のヒストグラムを正規化
	double leftTotal = 0.0, rightTotal = 0.0;
	
	for( int i = 0; i < numClass; i++ ){
		leftTotal = leftTotal + ldist[ i ];
		rightTotal = rightTotal + rdist[ i ];
	}

	for( int i = 0; i < numClass; i++ ){
		if( ldist[ i ] != 0.0 ){
			ldist[ i ] = ldist[ i ] / leftTotal;
		}
		if( rdist[ i ] != 0.0 ){
			rdist[ i ] = rdist[ i ] / rightTotal;
		}
	}

	double lentropy = 0.0;
	double rentropy = 0.0;

	for( int i = 0; i < numClass; i++ ){

		// 左のエントロピー
		if( ldist[ i ] != 0.0 ){
			lentropy = (double)lentropy + ldist[ i ] * (double)log10( ldist[ i ] );
		}

		// 右のエントロピー
		if( rdist[ i ] != 0.0 ){
			rentropy = (double)rentropy + rdist[ i ] * (double)log10( rdist[ i ] );
		}
	}
		
	// 情報利得の値を返却
	return (double)lSamples.size()  * -lentropy + (double)rSamples.size() * -rentropy;
}

//サブクラスの情報利得を算出
inline double Decisiontree::computeInformationGainSub( std::vector<const instance *> &lSamples, std::vector<const instance *> &rSamples, vector<double> &il ){
	vector<double> ldist(numClass, 0.0);
	vector<double> rdist(numClass, 0.0);

	// 左の確率密度関数を作成
	for(unsigned int s=0; s<lSamples.size(); s++){

		ldist[lSamples[s]->subLabel] += lSamples[s]->weight_sub;
	}

	// 右の確率密度関数を作成
	for(unsigned int s=0; s<rSamples.size(); s++){

		rdist[rSamples[s]->subLabel] += rSamples[s]->weight_sub;
	}
	double leftTotal = 0.0, rightTotal = 0.0;
	
	for( int i = 0; i < numClass; i++ ){
		leftTotal = leftTotal + ldist[ i ];
		rightTotal = rightTotal + rdist[ i ];
	}

	for( int i = 0; i < numClass; i++ ){
		if( ldist[ i ] != 0.0 ){
			ldist[ i ] = ldist[ i ] / leftTotal;
		}
		if( rdist[ i ] != 0.0 ){
			rdist[ i ] = rdist[ i ] / rightTotal;
		}
	}

	double lentropy = 0.0;
	double rentropy = 0.0;

	for( int i = 0; i < numClass; i++ ){
		// 左のエントロピー
		if( ldist[ i ] != 0.0 ){
			lentropy = (double)lentropy + ldist[ i ] * (double)log10( ldist[ i ] );
		}
		// 右のエントロピー
		if( rdist[ i ] != 0.0 ){
			rentropy = (double)rentropy + rdist[ i ] * (double)log10( rdist[ i ] );
		}
	}
	
	// 情報利得の値を返却
	return (double)lSamples.size()  * -lentropy + (double)rSamples.size() * -rentropy;
}

//分散の算出
inline double Decisiontree::computeVariance( std::vector<const instance *> &lSamples, std::vector<const instance *> &rSamples ){
	double tmpX, tmpY;
	double leftVariance = 0.0;
	double rightVariance = 0.0;
	Point2d leftCentroid = ( 0, 0 );
	Point2d rightCentroid = ( 0, 0 );

	// left
	if( lSamples.size() == 0 ){
		return DBL_MAX;
	}else{
	//左のサンプルのセントロイドの算出
		int lcount = 0;
		for(unsigned  int i = 0; i < lSamples.size(); i++ ){
			if(lSamples[i]->label == POS_LABEL){
				leftCentroid.x += lSamples[ i ]->offset.x;
				leftCentroid.y += lSamples[ i ]->offset.y;
				lcount++;
			}
		}
		leftCentroid.x = (double)(leftCentroid.x / lcount);
		leftCentroid.y = (double)(leftCentroid.y / lcount);
	}

	//right
	if( rSamples.size() == 0 ){
		return DBL_MAX;
	}else{
		//右のサンプルのセントロイドの算出
		int rcount = 0;
		for(unsigned int i = 0; i < rSamples.size(); i++ ){
			if(rSamples[i]->label == POS_LABEL){
				rightCentroid.x += rSamples[ i ]->offset.x;
				rightCentroid.y += rSamples[ i ]->offset.y;
				rcount ++;
			}
		}
		rightCentroid.x = (double)(rightCentroid.x / rcount);
		rightCentroid.y = (double)(rightCentroid.y / rcount);
	}

	//左の分散の算出
	int lcount = 0;
	for(unsigned int i = 0; i < lSamples.size(); i++ ){
		if(lSamples[i]->label == POS_LABEL){
			tmpX = leftCentroid.x - lSamples[ i ]->offset.x;
			leftVariance += tmpX * tmpX;
			tmpY = leftCentroid.y - lSamples[ i ]->offset.y;
			leftVariance += tmpY * tmpY;
			lcount ++;
		}
	}
	if( lcount == 0 ){
		leftVariance = 0;
	}else{
		leftVariance = leftVariance / (double)lcount;
	}


	//右の分散の算出
	int rcount = 0;
	for(unsigned int i = 0; i < rSamples.size(); i++ ){
		if(rSamples[i]->label == POS_LABEL){
			tmpX = rightCentroid.x - rSamples[ i ]->offset.x;
			rightVariance += tmpX * tmpX;
			tmpY = rightCentroid.y - rSamples[ i ]->offset.y;
			rightVariance += tmpY * tmpY;
			rcount ++;
		}
	}
	if( rcount == 0 ){
		rightVariance = 0.0;
	}else{
		rightVariance = rightVariance / (double)rcount;
	}

	double Variance = leftVariance + rightVariance;
	return  Variance;
}

// サンプルをトラバーサルする関数
inline bool Decisiontree::traversalSample(Node *node, const instance* &samp, vector<double> &il){

	if(node->nodeType == true){

		if( splitFunction(samp, node->Template, node->threshold) ){

			return traversalSample( node->left, samp, il );
		}else{

			return traversalSample( node->right, samp, il );
		}
	}else{

		node->weightDistribution[samp->label] += samp->weight;
		node->weightSubDistribution[samp->subLabel] += samp->weight_sub;

		//ポジティブサンプルのみ末端ノードに保存
		if(samp->label == POS_LABEL){
			node->samples.push_back(samp);
		}
	}

	return true;
}


///末端ノードのオフセットおよび角度のリストを作成
inline bool Decisiontree::normalization( Node* node){

	//printf("アドレス:%d\n", node);
	if(node->nodeType == true){
		// 分岐ノード
		//printf("分岐ノード(深さ：%d インデックス：%d しきい値：%d ID：%d) サンプル数：%d\n", node->depth, node->index, node->threshold, node->Trand_id, node->samples.size());
		normalization(node->left);
		normalization(node->right);
		
	}else{
		// 末端ノード
		int min = INT_MAX, max = INT_MIN;
		for( int s = 0; s < (int)node->samples.size(); s++){


			if( min > node->samples[ s ]->angle ){

				min = node->samples[ s ]->angle;
			}
			if( max < node->samples[ s ]->angle ){

				max = node->samples[ s ]->angle;
			}
		}

		// 確率密度関数の正規化
		double total = 0.0;
		for( int i = 0; i < numClass; i++ ){
			total = total + node->weightDistribution[ i ];
		}
		for( int i = 0; i < numClass; i++ ){
			node->weightDistribution[ i ] = node->weightDistribution[i] / total;
		}

		// 確率密度関数の正規化(サブクラス)
		double total_sub = 0.0;
		for( int i = 0; i < numClass; i++ ){

			total_sub = total_sub + node->weightSubDistribution[ i ];
		}
		for( int i = 0; i < numClass; i++ ){

			if( total_sub == 0 ){

				node->weightSubDistribution[ i ] = 0.0;
			}else{

				node->weightSubDistribution[ i ] = node->weightSubDistribution[i] / total_sub;
			}
		}
	}
	return true;
}

//ノードの情報を保存
inline bool Decisiontree::saveNode( Node *node, ofstream & ofs ){
	
	if(node == NULL){
		return false;
	}

	// このノードの情報
	ofs << node->depth << " "
		<< node->index << " "
		<< node->nodeType << " "
		<< std::endl;

	if(ofs.fail()){

		return false;
	}

	if(node->nodeType == true){
		ofs <<  node->threshold << endl;

		for(int c = 0; c < CELL_NUM; c++){

			ofs << node->feature0_180[c]
				<< std::endl;
			//ofs << node->feature0_180[c] << " "
			//	<< node->feature180_360[c]
			//	<< std::endl;
		}

		// 分岐ノードなので再帰処理で子ノードを書き込む
		if(saveNode(node->left, ofs) == false
		|| saveNode(node->right, ofs) == false){
			return false;
		}
	}
	else{

		int PosNum = 0, NegNum = 0;
		int Sub1 = 0, Sub2 = 0;
		for( int s = 0; s < node->allSamples.size(); s++ ){

			if( node->allSamples[s]->label == POS_LABEL ){

				PosNum++;
			}else{

				NegNum++;
			}
		}

		////末端ノードに保存されているセントロイド数とネガティブサンプル数の書き出し
//		ofs << node->samples.size() << " " << NegNum << endl;
		ofs << node->samples.size() << endl;

		// 末端ノードなのでクラス確率書き出し
		for( int i =  0; i < numClass; i++ ){
			
			ofs << node->weightDistribution[ i ] << endl;
		}
		//for( int i = 0; i < numClass; i++ ){

		//	ofs << 0.0 << endl;
		//}

		//ポジティブサンプルの情報を書き出し
		for( int s = 0; s < (int)node->samples.size(); s++){
			ofs << node->samples[s]->offset.x << " " <<
					 node->samples[s]->offset.y		 << " " <<
					 node->samples[s]->angle 	 	 << " " << 
//					 node->samples[s]->plane		 	 << " " << 
//					 node->samples[s]->Id				 << " " << 
//					 node->samples[s]->subLabel		 << " " << 
					 node->samples[s]->weight		 << " " << 
//					 0.0		 << " " << 
					 node->samples[s]->cutP.x		 << " " << 
					 node->samples[s]->cutP.y   << endl;
			//for(int c = 0; c < CELL_NUM; c++){
			//	ofs << node->samples[s]->dot[c].feature0_180 << " "
			//		 << node->samples[s]->dot[c].feature180_360
			//		 << std::endl;
			//}
		}

		////ネガティブサンプルの情報を書き出し
		//for( int s = 0; s < (int)node->allSamples.size(); s++){

		//	if( node->allSamples[ s ]->label == NEG_LABEL ){
		//	
		//		for(int c = 0; c < CELL_NUM; c++){
		//			ofs << node->allSamples[s]->dot[c].feature0_180 << " "
		//				 << node->allSamples[s]->dot[c].feature180_360
		//				 << std::endl;
		//		}
		//	}
		//}

		if(ofs.fail()){
			return false;
		}
	}
	return true;
}


//
//決定木を走査する関数
//
bool Decisiontree::Traversal(vector<const instance *> samples, vector<double> &il){
	Node *node = root;
	printf("トラバーサル開始 サンプル数:%d\n", samples.size());

	for(int i=0; i<samples.size(); i++){

		traversalSample(root, samples[i], il);
	}

	normalization(root);

	return true;
}

//決定木の保存を実行
bool Decisiontree::save( void ){
	char filename[ 256 ];
	sprintf( filename, "%sForest%d.dat", output.c_str(),  treeIndex );
	ofstream ofs( filename );
	saveNode( root, ofs );
	return true;
}

//決定木の初期化
bool Decisiontree::Initilizer( int _numTrees, int _maxDepth, int _featureTests, int _thresholdTest, int _numClass, int _treeIndex ){
	numTrees = _numTrees;
	maxDepth = _maxDepth;
	featureTests = _featureTests;
	thresholdTest = _thresholdTest;
	numClass = _numClass;
	treeIndex = _treeIndex;
	nodeNum = 0;
	return true;
}

//コンストラクタ
Decisiontree::Decisiontree( int _numTrees, int _maxDepth, int _featureTests, int _thresholdTest, int _numClass ){
	numTrees = _numTrees;
	maxDepth = _maxDepth;
	featureTests = _featureTests;
	thresholdTest = _thresholdTest;
	numClass = _numClass;
	nodeNum = 0;
}

//デフォルトコンストラクタ
Decisiontree::Decisiontree(void){
	numTrees = 5;
	maxDepth = 10;
	featureTests = 400;
}

//デストラクタ
Decisiontree::~Decisiontree(void)
{
}

//学習を実行する関数
bool Decisiontree::LearnVerWeight( const vector<const instance *> &samples, const vector<const instance *> &Tsamples, vector<double> &il, vector<double> &il_sub ){

	root = buildVerWeight( samples, Tsamples, il, il_sub, 0, 0 );

	return true;
}
