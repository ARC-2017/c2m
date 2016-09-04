#include "GA.h"


//-----------------------------------------------------------------------
// Set関連
//-----------------------------------------------------------------------
void GA::setN_ind( int n_ind ){

	this->n_ind = n_ind;
}

void GA::setLength( int length ){

	this->length = length;
}

void GA::setSumFitness( double sum ){

	this->sum_fitness = sum;
}

//-----------------------------------------------------------------------
// Get関連
//-----------------------------------------------------------------------
int GA::getN_ind(){

	return this->n_ind;
}

int GA::getGeneLength(){

	return this->length;
}

double GA::getAveFitness(){

	return this->ave_fitness;
}

double GA::getSumFitness(){

	return this->sum_fitness;
}

int	GA::getN_on_bit( int n ){

	int n_on;
	n_on = 0;
	for( int i=0 ; i<this->length ; i++ ){
		if( this->ind[n].chrom[i] ) n_on++;
	}

	return n_on;
}

//-----------------------------------------------------------------------
// コアの処理
//-----------------------------------------------------------------------
void GA::generateInitialIndividuals( int num, int length, int max_on ){

	//個体群の初期化
	this->setN_ind( num );
	this->setLength( length );
	this->ind.resize( num );
	for( int j=0 ; j<num ; j++ ){
		this->ind[j].chrom.resize( length );
		for( int i=0 ; i<length ; i++ ){
			this->ind[j].chrom[i] = false;
		}
		this->ind[j].fitness = 0.0;
		this->ind[j].lethal_gene = false;
		this->ind[j].born = INITIAL;
		this->ind[j].score_MA = 0.0;
		this->ind[j].score_NL = 0.0;
		this->ind[j].score_UA = 0.0;
		this->ind[j].score_PC = 0.0;
	}
	//初期個体の生成
	//最大max_on個のビットを1にする
	int on_bit;
	for( int j=0 ; j<num ; j++ ){
		for( int i=0 ; i<max_on ; i++ ){
			on_bit = rand()%length;
			this->ind[j].chrom[ on_bit ] = true;
		}
	}

	this->crossover_occured = false;
	this->mutation_occured = false;

}

int GA::selectParent(){

	double	parent_sum;
	int parent;

	parent_sum = this->getSumFitness()*((double)rand() / RAND_MAX);

	double	parent_inc;
	parent_inc = 0.0;
	parent = 0;
	for( int i=0 ; i<this->n_ind ; i++ ){
		parent_inc += this->ind[i].fitness;
		if( parent_sum < parent_inc ){
			parent = i;
			break;
		}
	}

	return parent;

}

void GA::crossover(){

	int parent1, parent2;
	individuall	tmp_child[2];

	parent1 = GA::selectParent();
	parent2 = GA::selectParent();

	tmp_child[0].chrom.resize( this->length );
	tmp_child[1].chrom.resize( this->length );

	std::vector<unsigned char> mask;
	mask.resize( this->length );
	for( int i=0 ; i<this->length ; i++ ){
		mask[i] = rand() % 2;
	}

	for( int i=0 ; i<this->length ; i++ ){
		if( mask[i] == 0 ){
			tmp_child[0].chrom[i] = this->ind[parent1].chrom[i];
			tmp_child[1].chrom[i] = this->ind[parent2].chrom[i];
		}else{
			tmp_child[0].chrom[i] = this->ind[parent2].chrom[i];
			tmp_child[1].chrom[i] = this->ind[parent1].chrom[i];
		}
	}
	tmp_child[0].born = CROSSOVER;
	tmp_child[1].born = CROSSOVER;

	tmp_child[0].lethal_gene = false;
	tmp_child[1].lethal_gene = false;

	this->child.push_back( tmp_child[0] );
	this->child.push_back( tmp_child[1] );

	this->crossover_occured = true;

}
void GA::mutation(){

	int		mutate_id;
	int		mutate_locus;
	individuall	tmp_child;

	tmp_child.chrom.resize( this->length );

	mutate_id = rand() % this->n_ind;
	mutate_locus = rand() % this->length;

	// 突然変異させる個体をコピー
	for( int i=0 ; i<this->length ; i++ ){
		tmp_child.chrom[i] = this->ind[mutate_id].chrom[i];
	}

	// mutate_locusの遺伝子を反転
	if( tmp_child.chrom[ mutate_locus ] ){
		tmp_child.chrom[ mutate_locus ] = false; 
	}else{
		tmp_child.chrom[ mutate_locus ] = true; 
	}
	tmp_child.born = MUTATION;
	tmp_child.lethal_gene = false;

	this->child.push_back( tmp_child );

	this->mutation_occured = true;
}

void GA::calcAveFitness(){

	double sum_fitness;
	sum_fitness = 0.0;
	for( int i=0 ; i<this->n_ind ; i++ ){
		sum_fitness += this->ind[i].fitness;
	}
	GA::setSumFitness( sum_fitness );
	this->ave_fitness = sum_fitness / (double)this->n_ind;

}


bool decrease( const individuall& left, const individuall& right ){

	return right.fitness < left.fitness;
}

void GA::SortIndividuals(){

	std::sort( this->ind.begin(), this->ind.end(), decrease );
	
}

void GA::update(){

	int	n_push_back;
	n_push_back = 0;

	for( int i=0 ; i<this->child.size() ; i++ ){
		// 親個体の最低適応度の個体よりも子個体の適応度が高かったら，その子個体を登録する
		if( this->ind[ this->n_ind-1 ].fitness < this->child[i].fitness ){
			this->ind.push_back( this->child[i] );
			n_push_back++;
		}
	}

	GA::SortIndividuals();

	//親個体の数を一定に保つ（登録した分，末端から順に削除する）
	for( int i=0 ; i<n_push_back ; i++ ){
		this->ind.pop_back();
	}

	this->max_fitness = this->ind[0].fitness;
	this->crossover_occured = false;
	this->mutation_occured = false;
	this->child.clear();
}


//-----------------------------------------------------------------------
// 可視化関連・デバッグ用関数など
//-----------------------------------------------------------------------

void GA::showIndividual( int n ){

	fprintf( stderr,"%d th individual\n", n );
	for( int i=0 ; i<this->length ; i++ ){
		if( this->ind[n].chrom[i] ){
			fprintf( stderr,"1" );
		}else{
			fprintf( stderr,"0" );
		}
	}
	fprintf( stderr,"-> fitness: %.5lf\n ", this->ind[n].fitness );
	if( this->ind[n].born == CROSSOVER ){
		fprintf( stderr,"Crossover\n" ); 
	}else if( this->ind[n].born == MUTATION ){
		fprintf( stderr,"Mutation\n" ); 
	}else if( this->ind[n].born == INITIAL ){
		fprintf( stderr,"Initial\n" ); 
	}else if( this->ind[n].born == MODIFIED ){
		fprintf( stderr,"Modified\n" ); 
	}
	fprintf( stderr,"ScoreMA  : %.3lf\n", this->ind[n].score_MA ); 
	fprintf( stderr,"ScoreNL  : %.3lf\n", this->ind[n].score_NL ); 
	fprintf( stderr,"ScoreUA  : %.3lf\n", this->ind[n].score_UA ); 
	fprintf( stderr,"ScorePC  : %.3lf\n", this->ind[n].score_PC ); 

	fprintf( stderr,"\n" ); 
	//致死遺伝子フラグは現在不使用
//	if( this->ind[n].lethal_gene ){
//		fprintf( stderr,"This is the Lethal Gene.\n" );
//	}
}

void GA::showAllIndividuals(){

	for( int i=0 ; i<this->n_ind ; i++ ){
		GA::showIndividual( i );
	}
}

void GA::showChildIndividual( int n ){

	fprintf( stderr,"%d th child\n", n );
	for( int i=0 ; i<this->length ; i++ ){
		if( this->child[n].chrom[i] ){
			fprintf( stderr,"1" );
		}else{
			fprintf( stderr,"0" );
		}
	}
	fprintf( stderr,"-> fitness: %.5lf ", this->child[n].fitness );
	if( this->child[n].born == CROSSOVER ){
		fprintf( stderr,"Crossover\n" ); 
	}else if( this->child[n].born == MUTATION ){
		fprintf( stderr,"Mutation\n" ); 
	}else{
		fprintf( stderr,"Initial\n" ); 
	}
	if( this->child[n].lethal_gene ){
		fprintf( stderr,"This is the Lethal Gene.\n", this->child[n].fitness );
	}
}

void GA::Ztest1(){

	//ダミーの適応度をセット
	double fitness;
	fitness = 1.0;
	for( int i=0 ; i<this->n_ind ; i++ ){
		this->ind[i].fitness = fitness; 
		fitness *= 0.9;
	}

	GA::calcAveFitness();

	std::vector<int> count;
	count.resize( this->n_ind );
	for( int i=0 ; i<this->n_ind ; i++ ){
		count[i] = 0;
	}
	for( int i=0 ; i<1000000 ; i++ ){
		count[ GA::selectParent() ]++;
	}

	for( int i=0 ; i<this->n_ind ; i++ ){
		fprintf( stderr,"fitness: %lf, count[%d] = %d\n", this->ind[i].fitness, i, count[i] );
	}
}


void GA::Ztest2(){

	//ダミーの適応度をセット
	for( int i=0 ; i<this->n_ind ; i++ ){
		this->ind[i].fitness = (double)rand()/(double)RAND_MAX; 
	}

	fprintf( stderr,"Before sort\n");
	GA::showAllIndividuals();
	GA::SortIndividuals();
	fprintf( stderr,"After sort\n");
	GA::showAllIndividuals();
}

void GA::Ztest3(){


	//ダミーの適応度をセット
	for( int i=0 ; i<this->n_ind ; i++ ){
		this->ind[i].fitness = (double)rand()/(double)RAND_MAX; 
	}
	//入れ替え用の個体を生成
	this->ind[this->n_ind-3].fitness = 0.0;
	this->ind[this->n_ind-2].fitness = 0.0;
	this->ind[this->n_ind-1].fitness = 0.0;

	GA::SortIndividuals();

	fprintf( stderr,"\nGeneration 1st.\n");
	GA::showAllIndividuals();

	GA::crossover();
	GA::mutation();

	fprintf( stderr,"\nChildren.\n");
	GA::showChildIndividual( 0 );
	GA::showChildIndividual( 1 );
	GA::showChildIndividual( 2 );
	this->child[0].fitness = 1.0;
	this->child[1].fitness = 1.0;
	this->child[2].fitness = 1.0;

	GA::update();

	fprintf( stderr,"\nGeneration 2nd.\n");
	GA::showAllIndividuals();
}