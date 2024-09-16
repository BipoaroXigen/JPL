#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <vector>

#include "multiann.h"
#include "ANN.h"

using namespace std;

extern "C" {

MPNN::MultiANN<int> * build_tree(const int dimension) {

    int *topology = new int[dimension];
    MPNN::ANNcoord *scale = new MPNN::ANNcoord[dimension];
    for(int i=0;i<dimension;i++) {
        scale[i] = 1.0; //scale of dimension
        topology[i] = 1; // indication to use euclidean distance
    }

    MPNN::MultiANN<int> * kdTree = new MPNN::MultiANN<int>(dimension,1,topology,(MPNN::ANNpoint)scale);

    return kdTree;
}

int append_kd(MPNN::MultiANN<int> * kdTree, int id, double * point, int dimension) {
    MPNN::ANNpoint pt = MPNN::annAllocPt(dimension);
    
    for(int j=0;j<dimension;j++) {
        pt[j] = point[j];
    }
    
    kdTree->AddPoint(pt, id);
    
    MPNN::annDeallocPt(pt);
	
    return 0;
}

int find_nn(MPNN::MultiANN<int> * kdTree, double * point, int dimension/*length of point array*/) {
    int idx; // = 0;
    double best_dist = INFINITY;

    MPNN::ANNpoint query = MPNN::annAllocPt(dimension);
    for(int j=0;j<dimension;j++) {
        query[j] = point[j];
    }

    // find nearest neighbor to the point 'query'. The result is 'nearest' which is index to 'data'
    int nearest = (int)kdTree->NearestNeighbor(query, idx, best_dist);

    // add to kd-tree and also to 'data' array.
    MPNN::annDeallocPt(query);

    return nearest; 
}

int* find_nnn(MPNN::MultiANN<int> * kdTree, double * point, int dimension, int n) {
    //int idx; // = 0;
    //double best_dist = INFINITY;
    int* idx = new int[n];
    int* idxx = new int[n];

    MPNN::ANNpoint query = MPNN::annAllocPt(dimension);
    MPNN::ANNpoint dists = MPNN::annAllocPt(n);
    for(int j=0;j<dimension;j++) {
        query[j] = point[j];
    }

    kdTree->NearestNeighbor(query, dists, idx, idxx);

    std::cout << "\n\n\ndistances\n";
    for(int i=0; i<n; i++){
    	std::cout << dists[i] << "\n";
    }
    std::cout << "idx\n";
    for(int i=0; i<n; i++){
    	std::cout << idx[i] << "\n";
    }
    std::cout << "idxx\n";
    for(int i=0; i<n; i++){
    	std::cout << idxx[i] << "\n";
    }
    std::cout << "\n\n\n";

    //delete [] idx;
    //delete [] idxx;
    MPNN::annDeallocPt(query);
    MPNN::annDeallocPt(dists);

    return idx;
    //return 1; 
}

int free_tree(MPNN::MultiANN<int> * kdTree) {
    delete kdTree;
    return 0;
}
}
