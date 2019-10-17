#ifndef _COMPRESSED_PATH_DATABASE_LIST_GRAPH_HEADER__
#define _COMPRESSED_PATH_DATABASE_LIST_GRAPH_HEADER__

struct Arc;
struct ListGraph;
class Mapper;

#include <vector>
#include <algorithm>
#include <string>
#include <cassert>
#include "range.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <functional>
#include <cpp-utils/igraph.hpp>
#include <pathfinding-utils/types.hpp>

namespace compressed_path_database::datastructures {

/**
 * represents an arc in the graph.
 *
 * Weights are assumed to be integers
 *
 * TODO the field should be nodeid_t
 */
struct Arc {
public:
	int source;
	int target;
	int weight;

	size_t hash() const;
};

}

namespace std {

	template <>
	struct hash<compressed_path_database::datastructures::Arc> {
		/**
		 * @note
		 * Used to be able to use Arc in unordered_map as key
		 *
		 * @return hash value of the arc
		 */
		std::size_t operator()(const compressed_path_database::datastructures::Arc& k) const;
	};
};

namespace compressed_path_database::datastructures {

/**
 * check if 2 arcs are the same
 *
 * the checking involves only the sources and the sink. The weight of the arc is not checked at all
 *
 * @param[in] l the first arc to consider
 * @param[in] r the second arc to consider
 * @return
 *  @li true if the 2 arcs are the same;
 *  @li false otherwise
 */
inline bool operator==(Arc l, Arc r){
	return l.source == r.source && l.target == r.target;
}

/**
 * check if 2 arcs are not similare
 *
 * the checking involves only the sources and the sink. The weight of the arc is not checked at all
 *
 * @param[in] l the first arc to consider
 * @param[in] r the second arc to consider
 * @return
 *  @li true if the 2 arcs are not the same;
 *  @li false otherwise
 */
inline bool operator!=(Arc l, Arc r){
	return !(l == r);
}

/**
 * a graph where each arc is stored in a list
 *
 * Nodes are assumed to be just integers with no paylaod whatsoever
 */
struct ListGraph {
public:
	/**
	 * number of vertices in the graph
	 */
	int n;
	/**
	 * a list of all the arcs in the graph
	 */
	std::vector<Arc>arc;
public:
	/**
	 * generates a totally empty graph
	 */
	ListGraph(): n{0} {

	}
	/**
	 * generates a graph with a certain number of vertices
	 *
	 * @param[in] node_count the number of vertices the graph has
	 */
	explicit ListGraph(int node_count): n{node_count} {

	}

	/**
	 * @return the number of vertices in the graph
	 */
	int node_count() const {
		return n;
	}

	/**
	 * check if the graph is valid
	 *
	 * @note
	 * a graph is valid if every arc in the graph has a source and sink which is greater than 0 and less than
	 * of the return value of ListGraph::node_count
	 *
	 * @return
	 *  @li true if the graph is valid,
	 *  @li false otherwise
	 */
	bool is_valid() const {
		bool ok = true;
		for(auto a:arc){
			ok &= (a.source != -1);
			ok &= (a.source >= 0);
			ok &= (a.source < node_count());
			ok &= (a.target != -1);
			ok &= (a.target >= 0);
			ok &= (a.target < node_count());
		}
		return ok;
	}

	/**
	 * Generate an image of the graph created
	 *
	 * @param[in] baseName the name of the image (no extension!)
	 */
	void print(const Mapper& mapper, const std::string& baseName);
};

/**
 * compare 2 graphs
 *
 * 2 graphs are the same if they have the same number of vertices and the exact arcs
 *
 * @param[in] l the first graph to check
 * @param[in] r the second graph to check
 * @return
 *  @li true if the graphs are the same;
 *  @li false otherwise;
 */
inline bool operator==(const ListGraph&l, const ListGraph&r){
	return l.arc == r.arc && l.n == r.n;
}

/**
 * compare 2 graphs
 *
 * 2 graphs are the same if they have the same number of vertices and the exact arcs
 *
 * @param[in] l the first graph to check
 * @param[in] r the second graph to check
 * @return
 *  @li true if the graphs are **not** the same;
 *  @li false otherwise;
 */
inline bool operator!=(const ListGraph&l, const ListGraph&r){
	return !(l == r);
}

/**
 * @brief convert cpp utils list graph into cpd list graph
 * 
 * @tparam G payload of list graph
 * @tparam V payload of each vertex in the graph
 * @param g graph to convert
 * @return compressed_path_database::datastructures::ListGraph 
 */
template <typename G, typename V>
compressed_path_database::datastructures::ListGraph fromCppUtilsListGraphToCpdListGraph(const cpp_utils::graphs::ListGraph<G, V, pathfinding::cost_t>& g) {
	compressed_path_database::datastructures::ListGraph result{static_cast<int>(g.numberOfVertices())};

	for (cpp_utils::graphs::nodeid_t sourceId=0; sourceId<g.numberOfVertices(); ++sourceId) {
		for (auto outEdge: g.getOutEdges(sourceId)) {
			result.arc.push_back(Arc{
				static_cast<int>(sourceId), 
				static_cast<int>(outEdge.getSinkId()), 
				static_cast<int>(outEdge.getPayload())
			});
		}
	}

	return result;
}

/**
 * extract
 *
 * @param[in] g the graph we're operating on
 * @param[in] g_to_top_level
 * @param[in] is_in_s
 * @param[in] g_to_s  Maps an ID from g onto the id in the subgraph or -1
 * @param[in] s_to_top_level
 * @return
 *
 */
template<class IsNodeInSubgraph>
ListGraph extract_node_induced_subgraph(
		const ListGraph&g,
		const std::vector<int>&g_to_top_level,
		const IsNodeInSubgraph&is_in_s,
		std::vector<int>&g_to_s, //! Maps an ID from g onto the id in the subgraph or -1
		std::vector<int>&s_to_top_level
){
	assert(g.is_valid());
	assert(g.node_count() == (int)g_to_top_level.size());
	int s_node_count = 0;
	for(int i=0; i<g.node_count(); ++i)
		if(is_in_s(i))
			++s_node_count;

	ListGraph s(s_node_count);
	s_to_top_level.resize(s_node_count);

	g_to_s.resize(g.node_count());

	int n = 0;
	for(int i=0; i<g.node_count(); ++i)
		if(is_in_s(i)){
			s_to_top_level[n] = g_to_top_level[i];
			g_to_s[i] = n;
			++n;
		}else
			g_to_s[i] = -1;

	for(auto a:g.arc)
		if(g_to_s[a.source] != -1 && g_to_s[a.target] != -1)
			s.arc.push_back({g_to_s[a.source], g_to_s[a.target]});

	assert(s.is_valid());
	return std::move(s);
}

template<class IsNodeInSubgraph>
ListGraph extract_node_induced_subgraph(const ListGraph&g, const IsNodeInSubgraph&is_in_s){
	std::vector<int>ignore1(g.node_count()), ignore2, ignore3;
	return extract_node_induced_subgraph(g, ignore1, is_in_s, ignore2, ignore3);
}

template<class IsNodeInSubgraph>
ListGraph extract_node_induced_subgraph(const ListGraph&g, const std::vector<int>&g_to_top_level, const IsNodeInSubgraph&is_in_s, std::vector<int>&s_to_top_level){
	std::vector<int>ignore;
	return extract_node_induced_subgraph(g, g_to_top_level, is_in_s, ignore, s_to_top_level);
}

}

#endif
