#include "SequenceLearner.h"

SequenceLearner::~SequenceLearner(){

}

void SequenceLearner::init() {


}

void SequenceLearner::printAll() {

	for (int i = 0; i < nInitialized; i++) {
		p[i]->print(500,10);
		pi[i]->print(500,10);
	}
}

void SequenceLearner::printToFile() {


}
