#ifndef ADJ_ARRAY_H
#define ADJ_ARRAY_H

#include <algorithm>
#include <cassert>

#include <cpp-utils/log.hpp>
#include <cpp-utils/operators.hpp>

namespace detail {
	/**
	 * Builds a vector with the cumulative number of arcs in a referecnes graph
	 *
	 * The features are:
	 * @li size is @c node_count + 1;
	 * @li each index represents a vertex in the graph;
	 * @li each value in the vector is the number of arcs having that indexes vertex as source;
	 *
	 * For example, if there is the graph
	 *
	 * @dot
	 * A [pos="0,0!"];
	 * B [pos="0,1!"];
	 * C [pos="1,0!"];
	 * D [pos="1,1!"];
	 * A--B;
	 * A--C;
	 * B--D;
	 * B--C;
	 * C--D;
	 * @enddot
	 *
	 * If the nodes are order alphabetically, then we will generate the vector `0, 2, 4, 5` because out arc of A, B, C and D are,
	 * respectively, 2, 2, 1, and 0.
	 *
	 * @param[out] a vector that will contain the cumulative sum of the number of outgoing edges in the graph. This sequence always
	 * 	starts with 0;
	 * @param[in] node_count number fo vertices in the graph
	 * @param[in] arc_count number fo arcs in the graph
	 * @param[in] source a function that given the i-th arc retrieve the sourc eof such arc.
	 *  Semantic of which is the i-th arc is left to the lambda implementation
	 */
	template<class Vector, class ArcSource>
	void build_begin_array(Vector&out_begin, int node_count, int arc_count, ArcSource source){
		out_begin.resize(node_count+1);

		std::fill(out_begin.begin(), out_begin.begin()+node_count, 0);

		//in out_begin each value in the vector is the number of arcs having that indexes vertex as source;
		for(int i=0; i<arc_count; ++i){
			int x = source(i);
			assert(x >= 0);
			assert(x<node_count);
			++out_begin[x];
		}
		//debug("temporary out_begin is ", out_begin);
		
		//now in out_begineach
		/*
		 * example:
		 * [3 4 1 2]
		 * [0 3 7 8]
		 */
		int sum = 0;
		for(int i=0; i<=node_count; ++i){
			int tmp = out_begin[i];
			out_begin[i] = sum;
			sum += tmp;
		}
	}
}

// Builds an adjacency such that the outgoing arcs of node v have the ids 
// out_begin[v]...out_begin[v-1]. Further the out_dest array is filled with 
// some arc data. (Such as nearly always at least the arc's tail/target.)
//
// To acheive this the number the number of nodes and arcs must be given. Further
// source is a functor that maps an arc id onto its head/source node. arc_data
// maps an arc id onto the data that should be stored in out_dest.

/**
 * Builds an adjacency vector with given properties
 *
 * The properties are as follows:
 * @li outgoing arcs of node `v` have ids `out_begin[v]... out_begin[v-1]`;
 * @li @c out_dest array is filled with some arc data such that as nearly always at least the arc's tail/target
 *
 * @param[out] out_begin an array repersenting, for each vertices, the index in @c out_dest where the
 * 	outgoing arcs from said vertex begins (remember: the arcs in out_dest related to a vertex are contiguous)
 * @param[out] out_dest an array containing the output of @c arc_data. arc data concerning the same vertex happens in a contiguous
 * 	area of @c out_dest.
 * @param[in] node_count number of vertices in the graph
 * @param[in] arc_count number of arcs in the graph
 * @param[in] source a lambda which gives the source of the i-th arc
 * @param[in] arc_data a lambda which gives the data we want to put in @c out_dest of the i-th arc
 */
template<class Vector1, class Vector2, class ArcSource, class ArcData>
void build_adj_array(Vector1&out_begin, Vector2&out_dest,
	int node_count, int arc_count, ArcSource source, ArcData arc_data){

	detail::build_begin_array(out_begin, node_count, arc_count, source);

	out_dest.resize(arc_count);

	/*
	 * if arcs are [3 4 1] then out_begin will be [0 3 7 8].
	 * They represents the idnex if out_dest where to put the data of each arc. Since the first node has
	 * 3 arcs, the first 3 slots of out_dest (i.e., 0, 1, 2) contains the 3 data of arcs. out_begin initial values
	 * represents the index of out_dest where to start putting the values of each arc. the "++" of out_begin[source(i)]++
	 * is used to move the index for the next arc data to add in out_dest
	 */
	for(int i=0; i<arc_count; ++i)
		out_dest[out_begin[source(i)]++] = arc_data(i);

	//we reset the "++" index generated from the previous for loop.
	for(int i=0; i<arc_count; ++i)
		--out_begin[source(i)];
}

#endif

