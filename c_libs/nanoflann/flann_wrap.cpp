#include <stdlib.h>
#include <vector>
#include <iostream>

#include "nanoflann.hpp"


struct point_t{
	double x;
	double y;
	double z;
	double a;
	double b;
	double c;
};

struct points_t{
    std::vector<point_t> points;

    inline size_t kdtree_get_point_count() const { return points.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0)
            return points[idx].x;
        else if (dim == 1)
            return points[idx].y;
        else if (dim == 2)
            return points[idx].z;
        else if (dim == 3)
            return points[idx].a;
        else if (dim == 4)
            return points[idx].b;
        else
            return points[idx].c;
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const
    {
        return false;
    }
};


//typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
//		nanoflann::L2_Simple_Adaptor<double, points_t>,
//		points_t,
//		6                         /* dim */
//		> d_kd_tree_t ;

//typedef nanoflann::KDTreeSingleIndexAdaptor<
//		nanoflann::L2_Simple_Adaptor<double, points_t>,
//		points_t,
//		6                         /* dim */
//		> kd_tree_t ;

//struct container{
//	kd_tree_t index;
//};

//using kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
//       	nanoflann::L2_Simple_Adaptor<double, points_t>,
//       	points_t,
//       	6/* dim */>;
	
using kd_tree_t = nanoflann::KDTreeSingleIndexDynamicAdaptor<
       	nanoflann::L2_Simple_Adaptor<double, points_t>,
       	points_t,
       	6/* dim */>;

points_t pts;
//kd_tree_t index(6/*dim*/, pts, {10/* max leaf */});
kd_tree_t index(6/*dim*/, pts, {10/* max leaf */});

extern "C" {
int alive(int a){
	return a;
}

void* construct_points(){
	
	points_t pts;

	return &pts;
}
void* construct_index(int dimensions, int max_leaf_volume, points_t* pts){
	
	//kd_tree_t index(dimensions /*dim*/, pts, {max_leaf_volume/* max leaf */});
	kd_tree_t index(6/*dim*/, *pts, {10/* max leaf */});

	return &index;
}

int add_point_dynamic(double* point){

	const size_t idx = pts.kdtree_get_point_count();
		
	point_t pt;
	pt.x=point[0];
	pt.y=point[1];
	pt.z=point[2];
	pt.a=point[3];
	pt.b=point[4];
	pt.c=point[5];
	pts.points.push_back(pt);


//	pts.points.emplace_back(point[0], point[1], point[2], point[3], point[4], point[5]);
	index.addPoints(idx, idx);

	return idx;
}

int remove_point(size_t idx){
	index.removePoint(idx-1);

	return idx;
}

int return_point(size_t idx){
	index.addPoints(idx, idx);

	return idx;
}




//int query_d(point_t point, double radius, d_kd_tree index){
//	return index;
//}

//std::vector<nanoflann::ResultItem<uint32_t, double>> query(double* point, double radius){
/*int query_static(double* point, double radius){
	std::vector<nanoflann::ResultItem<uint32_t, double>> ret_matches;
	const size_t nMatches = index.radiusSearch(point, radius, ret_matches);
	std::cout << "matches: \n";
	std::cout << nMatches;
	for (int i = 0; i < nMatches; i++){
		std::cout << ret_matches[i].first << "   ";	
		std::cout << ret_matches[i].second << "\n";	
	}
	std::cout << "\n";
	//return ret_matches;
	return ret_matches[2].first;
}
*/
int* query(double* point, double radius, int* length){
        std::vector<nanoflann::ResultItem<size_t, double>> indices_dists;
        nanoflann::RadiusResultSet<double, size_t> resultSet(radius, indices_dists);
        index.findNeighbors(resultSet, point);
	
	length[0] = indices_dists.size();

	int* result = new int[indices_dists.size()];
	for (int i = 0; i < indices_dists.size(); i++){
		result[i] = indices_dists[i].first;
	}
	return result;

}

int nn_query(double* point){
        const size_t num_results = 1;
        size_t ret_index;
        double out_dist_sqr;
        nanoflann::KNNResultSet<double> resultSet(num_results);
        resultSet.init(&ret_index, &out_dist_sqr);
        index.findNeighbors(resultSet, point, {10});

	return ret_index;
}
}
