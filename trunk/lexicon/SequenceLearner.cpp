#include "SequenceLearner.h"

SequenceLearner::~SequenceLearner(){

}

void SequenceLearner::init() {


}

//this function is same for any kind of output dist
void SequenceLearner::packA(Bottle &dst, int n) {

	for (int i = 0; i < r; i++) {
		Bottle &tmp = dst.addList();
		for (int j = 0; j < r; j++) {
			tmp.add(p[n]->A->ptr[i][j]);
		}
	}

}

//don't define this, it will be different for all derived classes
void SequenceLearner::packObs(Bottle &dst, int n) {

}

void SequenceLearner::printAll() {

	for (int i = 0; i < nInitialized; i++) {
		p[i]->print(500,10);
		pi[i]->print(500,10);
	}
}

void SequenceLearner::printToFile(string baseName) {


}

bool SequenceLearner::generateSequence(IMat &data, int n, double dscale) {

	return false;

}

bool SequenceLearner::generateSequence(IVecInt &data, int initPos, int length, int n) {

	return false;

}
