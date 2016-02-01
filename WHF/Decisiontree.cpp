#define _CRT_SECURE_NO_WARNINGS
#include "Decisiontree.h"

static const string output = OUTPUT_PATH;
static const double LOG10_2 = 0.301;
using namespace std;

ofstream ofs1( "./Weight_TXT/weight.txt" );

int face2[] = {cv::FONT_HERSHEY_SIMPLEX, cv::FONT_HERSHEY_PLAIN, cv::FONT_HERSHEY_DUPLEX, cv::FONT_HERSHEY_COMPLEX, 
              cv::FONT_HERSHEY_TRIPLEX, cv::FONT_HERSHEY_COMPLEX_SMALL, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 
              cv::FONT_HERSHEY_SCRIPT_COMPLEX, cv::FONT_ITALIC};


//����֐�
inline bool Decisiontree::splitFunction(const instance* &samples, const instance* &Tsamples, float &th){

	unsigned int comparison0_180[CELL_NUM];
	unsigned int comparison180_360[CELL_NUM];
	float sum = CELL_NUM;
	float distance = 0.0;

	int Sample0_180;
	int SampleAndTemplate0_180;
	int Template0_180;


	for(int cell = 0; cell < CELL_NUM; cell++){

		//�T���v���ƃe���v���[�g��DOT��AND���Z
		comparison0_180[cell] = samples->dot[cell].feature0_180 & Tsamples->dot[cell].feature0_180;
		comparison180_360[cell] = samples->dot[cell].feature180_360 & Tsamples->dot[cell].feature180_360;

		Sample0_180 = (int)BITS_COUNT_TABLE[samples->dot[cell].feature0_180];
		Template0_180 = (int)BITS_COUNT_TABLE[Tsamples->dot[cell].feature0_180];
		SampleAndTemplate0_180 = (int)BITS_COUNT_TABLE[comparison0_180[cell]];

		//���҂̓����ʂ̒l��0�̏ꍇ�͏��O
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

	//�������
	if(distance < th ){

		return true;
	}else{

		//return true;
		return false;
	}
}

//����֐��̍쐬
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

		printf("�T���v�����F%d �e���v���[�g���F%d", samples.size(), Tsamples.size());
		return NULL;
	}


	// �������ʂ̕ێ��p�o�b�t�@
	std::vector<const instance *> lSamples;
	std::vector<const instance *> rSamples;
	std::vector<const instance *> TemplateCandidate;

	// �K�v�ɂȂ蓾��T�C�Y�̍ő�l���̃o�b�t�@���m�ۂ��Ă���
	lSamples.reserve(samples.size());
	rSamples.reserve(samples.size());
	node->lSamples.reserve(samples.size());
	node->rSamples.reserve(samples.size());

	//�T���v����DOT�����ێ��p�̔z��
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


	// �]���l���ŏ������镪�����ʂ�T������
	double bestGain = DBL_MAX;				
	float bestThreshold = 0;
	int bestTrand_id = 0;
	std::vector<const instance *> bestlSamples;
	std::vector<const instance *> bestrSamples;
	const instance* bestTemplate;
	const instance* Template;
	bestlSamples.reserve(samples.size());
	bestrSamples.reserve(samples.size());


	//�m�[�h�ɕۑ�����e���v���[�g���|�W�e�B�u�T���v�����烉���_���ɑI��
	for(int ft=0; ft<featureTests; ++ft){

		int Trand_id = 
			(int)(0+(float)rand()/(float)(RAND_MAX) * ((TemplateCandidate.size() - 1) - 0));

		if( TemplateCandidate.size() < Trand_id ){
			printf("�v�f���F%d �ԍ��F%d\n", TemplateCandidate.size(), Trand_id);
		}
		Template = TemplateCandidate[Trand_id];

		for(int tt=0; tt<thresholdTest; ++tt){

			// ����ɂ����������l������ʂ̍ő�E�ŏ��̊ԂŃ����_���Ɍ���
			float rand_threshold = float(0.0+(float)rand()/(float)(RAND_MAX)
			                       *((float)1.0 - 0.0 ));

			//
			// ��L�̃����_���Ȃ������l�ŕ���������
			//
			lSamples.clear();
			rSamples.clear();

			//�T���v���𕪊�
			for(unsigned int s=0; s<(int)samples.size(); ++s){

				const instance* tmp;
				tmp = samples[s];
				if(splitFunction(tmp, Template, rand_threshold)){

					//	���q�m�[�h�̃T���v���ɒǉ�
					lSamples.push_back(samples[s]);
				}else{

					//	�E�̎q�m�[�h�ɃT���v����ǉ�
					rSamples.push_back(samples[s]);
				}
			}

			//
			//gainSwitch �ɂ��]���֐��̑I��
			//
			double gain;
			if( depth % 2 == 0 && (int)samples.size() > SPLIT_SELECT_MIN ){

				// ��񗘓��̎Z�o
				gain = computeInformationGain(lSamples, rSamples, il);
			}else{

				// ���U�̎Z�o
				gain = computeVariance(lSamples, rSamples);
			}


			// �]���l�̍ŏ��l��T��
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
		} //tt���[�vend
	}	//ft���[�vend

	// �ŗǂ������p�����[�^���m�[�h�ɕۑ�
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

	//�m�[�h�̃p�b�`��ۑ�
	for( int s = 0; s < samples.size(); s++ ){

		node->allSamples.push_back(const_cast<instance*>( samples[s] ));

		//�|�W�e�B�u�p�b�`�݂̂�ۑ�
		if( samples[s]->label == POS_LABEL ){

			node->samples.push_back( samples[s] );
		}
	}
	
	//���[�m�[�h�ɓ��B����������
	if(DBL_MIN > bestGain || maxDepth < depth || bestlSamples.empty() || bestrSamples.empty() || samples.size() < SAMPLE_MIN){
		
		node->nodeType = false;
		node->weightDistribution.resize(numClass, 0.0);
		node->weightSubDistribution.resize( numClass, 0.0 );

		if( samples.size() == 0 ){

			printf("no sample\n");
			return NULL;
		}
	//����m�[�h
	}else{

		node->nodeType = true;
	}

	return node;
}

//����؂̍쐬���s���֐�
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
	cout << "�T���v���̑����F" << samples.size() << " �|�W�e�B�u�T���v�����F" << pos_n << endl;

	if(samples.empty()  || Tsamples.empty()){

		return NULL;
	}
	
	//�m�[�h�ۑ��p�̃L���[
	queue<Node *> splitNode;
	queue<Node *> leafNode;

	Node *node = new Node();
	node->nodeType = true;
	Node *root;
	root = node;

	//���[�g�m�[�h���쐬
	node = createNode( node, samples, Tsamples, il, 0, 0 );
	splitNode.push( node );

	int nowDepth = 0;
	int nodeNum = 0;
	//�I������(�w�K�̏I��)�ɒB����܂Ŗ������[�v
	printf("\n");
	while(true){
		cout << "�[���F" << nowDepth << " �m�[�h�F" << nodeNum << "\r";

		// �w�K�I��
		if( splitNode.size() == NULL ){

			//�d�ݍX�V�֐�
//			weightUpdate( splitNode, leafNode, negRate, sub );
//			weightNormalization( samples );

			break;
		}

		//splitNode�̐擪�v�f�����o��
		node = splitNode.front();

		//�K�w�̐؂�ւ��𔻒�
		if( nowDepth != node->depth ){

			//�d�ݍX�V�֐�
			weightUpdate( splitNode, leafNode );
			weightNormalization( samples );

			nowDepth++;
			nodeNum = 0;
		}
		
		//���̎q�m�[�h�̃|�C���^�������ꍇ�C�m�[�h�𐶐�
		if( node->left == NULL ){

			//printf("���̎q�m�[�h�쐬��...\n");
			node->left = new Node();
			node->left = createNode( node->left, node->lSamples, Tsamples, il, node->depth+1, node->index+1 );

			//nodeType�ɉ������L���[��node->left���X�^�b�N
			if( node->left->nodeType == true ){

				splitNode.push( node->left );
			}else{

				leafNode.push( node->left );
			}
			
		//�E�̎q�m�[�h�̃|�C���^�������ꍇ�C�m�[�h�𐶐�
		}else if( node->right == NULL ){

			//printf("�E�̎q�m�[�h�쐬��...\n");
			node->right = new Node();
			node->right = createNode( node->right, node->rSamples, Tsamples, il, node->depth+1, node->index+2 );

			//nodeType�ɉ������L���[��node->left���X�^�b�N
			if( node->right->nodeType == true ){

				splitNode.push( node->right );
			}else{

				leafNode.push( node->right );

			}
		}else if( node->left != NULL && node->right != NULL ){

			//�s�v�ƂȂ����T���v�����폜
			if( node->nodeType == true ){
				node->allSamples.clear();
				node->samples.clear();
				node->lSamples.clear();
				node->rSamples.clear();
			}

			//���E�̎q�m�[�h���쐬�����̂Őe�m�[�h���L���[����폜
			splitNode.pop();
		}
		nodeNum++;
	}
	return root;
}

//�d�݂̐��K��
inline bool Decisiontree::weightNormalization( const vector<const instance *> &samples ){
	
	double tatal = 0.0;
	double tatal_sub = 0.0;
	double max = DBL_MIN;
	double min = DBL_MAX;
	double max_sub = DBL_MIN;
	double min_sub = DBL_MAX;

	//�d�݂̑��a���Z�o
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

	//���K��
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


//�d�ݍX�V
inline bool Decisiontree::weightUpdate( queue<Node *> &splitNode, queue<Node *> &leafNode ){

	Node *tmpNode;

	//����m�[�h�ɑ΂��鏈��
	size_t splitNodeNum = splitNode.size();
	if( splitNode.size() != NULL ){
		 
		ofs1 << "\n" << endl;
		for( int i = 0; i < splitNodeNum; i++ ){

			tmpNode = splitNode.front();

			//�d�݂��m�����Z�o
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
			//�d�݂��m�����Z�o(�T�u�N���X)
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

			//�d�ݍX�V
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

	//���[�m�[�h�ɑ΂��鏈��
	size_t leafNodeNum = leafNode.size();
	if( leafNode.size() != NULL ){
		ofs1 << "\n" << endl;
		for( int i = 0; i < leafNodeNum; i++ ){

			tmpNode = leafNode.front();

			//�d�݂��m�����Z�o
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

			//�d�݂��m�����Z�o(�T�u�N���X)
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

			//�d�ݍX�V
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

//��񗘓��̌v�Z
inline double Decisiontree::computeInformationGain( std::vector<const instance *> &lSamples, std::vector<const instance *> &rSamples, vector<double> &il ){
	vector<double> ldist(numClass, 0.0);
	vector<double> rdist(numClass, 0.0);

	// ���̊m�����x�֐����쐬
	for(unsigned int s=0; s<lSamples.size(); s++){

		ldist[lSamples[s]->label] += lSamples[s]->weight;
	}

	// �E�̊m�����x�֐����쐬
	for(unsigned int s=0; s<rSamples.size(); s++){

		rdist[rSamples[s]->label] += rSamples[s]->weight;
	}

	// ���E�̃q�X�g�O�����𐳋K��
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

		// ���̃G���g���s�[
		if( ldist[ i ] != 0.0 ){
			lentropy = (double)lentropy + ldist[ i ] * (double)log10( ldist[ i ] );
		}

		// �E�̃G���g���s�[
		if( rdist[ i ] != 0.0 ){
			rentropy = (double)rentropy + rdist[ i ] * (double)log10( rdist[ i ] );
		}
	}
		
	// ��񗘓��̒l��ԋp
	return (double)lSamples.size()  * -lentropy + (double)rSamples.size() * -rentropy;
}

//�T�u�N���X�̏�񗘓����Z�o
inline double Decisiontree::computeInformationGainSub( std::vector<const instance *> &lSamples, std::vector<const instance *> &rSamples, vector<double> &il ){
	vector<double> ldist(numClass, 0.0);
	vector<double> rdist(numClass, 0.0);

	// ���̊m�����x�֐����쐬
	for(unsigned int s=0; s<lSamples.size(); s++){

		ldist[lSamples[s]->subLabel] += lSamples[s]->weight_sub;
	}

	// �E�̊m�����x�֐����쐬
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
		// ���̃G���g���s�[
		if( ldist[ i ] != 0.0 ){
			lentropy = (double)lentropy + ldist[ i ] * (double)log10( ldist[ i ] );
		}
		// �E�̃G���g���s�[
		if( rdist[ i ] != 0.0 ){
			rentropy = (double)rentropy + rdist[ i ] * (double)log10( rdist[ i ] );
		}
	}
	
	// ��񗘓��̒l��ԋp
	return (double)lSamples.size()  * -lentropy + (double)rSamples.size() * -rentropy;
}

//���U�̎Z�o
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
	//���̃T���v���̃Z���g���C�h�̎Z�o
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
		//�E�̃T���v���̃Z���g���C�h�̎Z�o
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

	//���̕��U�̎Z�o
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


	//�E�̕��U�̎Z�o
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

// �T���v�����g���o�[�T������֐�
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

		//�|�W�e�B�u�T���v���̂ݖ��[�m�[�h�ɕۑ�
		if(samp->label == POS_LABEL){
			node->samples.push_back(samp);
		}
	}

	return true;
}


///���[�m�[�h�̃I�t�Z�b�g����ъp�x�̃��X�g���쐬
inline bool Decisiontree::normalization( Node* node){

	//printf("�A�h���X:%d\n", node);
	if(node->nodeType == true){
		// ����m�[�h
		//printf("����m�[�h(�[���F%d �C���f�b�N�X�F%d �������l�F%d ID�F%d) �T���v�����F%d\n", node->depth, node->index, node->threshold, node->Trand_id, node->samples.size());
		normalization(node->left);
		normalization(node->right);
		
	}else{
		// ���[�m�[�h
		int min = INT_MAX, max = INT_MIN;
		for( int s = 0; s < (int)node->samples.size(); s++){


			if( min > node->samples[ s ]->angle ){

				min = node->samples[ s ]->angle;
			}
			if( max < node->samples[ s ]->angle ){

				max = node->samples[ s ]->angle;
			}
		}

		// �m�����x�֐��̐��K��
		double total = 0.0;
		for( int i = 0; i < numClass; i++ ){
			total = total + node->weightDistribution[ i ];
		}
		for( int i = 0; i < numClass; i++ ){
			node->weightDistribution[ i ] = node->weightDistribution[i] / total;
		}

		// �m�����x�֐��̐��K��(�T�u�N���X)
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

//�m�[�h�̏���ۑ�
inline bool Decisiontree::saveNode( Node *node, ofstream & ofs ){
	
	if(node == NULL){
		return false;
	}

	// ���̃m�[�h�̏��
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

		// ����m�[�h�Ȃ̂ōċA�����Ŏq�m�[�h����������
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

		////���[�m�[�h�ɕۑ�����Ă���Z���g���C�h���ƃl�K�e�B�u�T���v�����̏����o��
//		ofs << node->samples.size() << " " << NegNum << endl;
		ofs << node->samples.size() << endl;

		// ���[�m�[�h�Ȃ̂ŃN���X�m�������o��
		for( int i =  0; i < numClass; i++ ){
			
			ofs << node->weightDistribution[ i ] << endl;
		}
		//for( int i = 0; i < numClass; i++ ){

		//	ofs << 0.0 << endl;
		//}

		//�|�W�e�B�u�T���v���̏��������o��
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

		////�l�K�e�B�u�T���v���̏��������o��
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
//����؂𑖍�����֐�
//
bool Decisiontree::Traversal(vector<const instance *> samples, vector<double> &il){
	Node *node = root;
	printf("�g���o�[�T���J�n �T���v����:%d\n", samples.size());

	for(int i=0; i<samples.size(); i++){

		traversalSample(root, samples[i], il);
	}

	normalization(root);

	return true;
}

//����؂̕ۑ������s
bool Decisiontree::save( void ){
	char filename[ 256 ];
	sprintf( filename, "%sForest%d.dat", output.c_str(),  treeIndex );
	ofstream ofs( filename );
	saveNode( root, ofs );
	return true;
}

//����؂̏�����
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

//�R���X�g���N�^
Decisiontree::Decisiontree( int _numTrees, int _maxDepth, int _featureTests, int _thresholdTest, int _numClass ){
	numTrees = _numTrees;
	maxDepth = _maxDepth;
	featureTests = _featureTests;
	thresholdTest = _thresholdTest;
	numClass = _numClass;
	nodeNum = 0;
}

//�f�t�H���g�R���X�g���N�^
Decisiontree::Decisiontree(void){
	numTrees = 5;
	maxDepth = 10;
	featureTests = 400;
}

//�f�X�g���N�^
Decisiontree::~Decisiontree(void)
{
}

//�w�K�����s����֐�
bool Decisiontree::LearnVerWeight( const vector<const instance *> &samples, const vector<const instance *> &Tsamples, vector<double> &il, vector<double> &il_sub ){

	root = buildVerWeight( samples, Tsamples, il, il_sub, 0, 0 );

	return true;
}
