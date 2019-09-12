#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <algorithm>
#include <vector>
#include <limits>

//TODO remove #include "adj_graph.h"
#include <cpp-utils/adjacentGraph.hpp>
#include <cpp-utils/KHeaps.hpp>
//#include "heap.h"

namespace cpd {

using namespace cpp_utils::graphs;
using namespace pathfinding;

template <typename G, typename V>
class Dijkstra {
private:
	/**
	 * @brief the graph where we need to run dijkstra over
	 * 
	 */
	const AdjacentGraph<G,V,cost_t>& g;
	/**
	 * @brief queue of dijkstra algorithm
	 * 
	 */
	cpp_utils::min_id_heap<nodeid_t, cost_t> q;
	/**
	 * distance array
	 *
	 * node ids are indexes of this array. The value in each cell represents the minimum distance got up until now between the source node and the given one.
	 * For example if in index 4 there is the value 60 it means the minimum path distance from source_node and the vertex with id 4 is 60.
	 *
	 */
	std::vector<cost_t> dist;
	/**
	 * reachable array
	 *
	 * an array where the index is a node id while the values are the best direction to keep if you want to optimally go to the given dpf::nodeid_t
	 * starting from source_node. the number should be interpreted as a bit set. A 1 in a given bit \f$ i \f$ means that you can optimally reach
	 * the node id vertex following the direction \f$ i \f$ of the AdjGraph.
	 */
	std::vector<moveid_t> allowed;
public:
	Dijkstra(const AdjacentGraph<G,V,cost_t>& g): g{g}, q(g.node_count()), dist(g.node_count()), allowed(g.node_count()){

	}

	/**
	 * execute dijkstra algorithm
	 *
	 * @param[in] source_node id of the node where we start disjkstra from
	 * @return
	 * 	@li an array containing the best direction to use to reach every node starting from @c source_node;
	 * 	@li if a cell has "0" there is no best directions;
	 */
	const std::vector<moveid_t>& run(nodeid_t source_node) {
		std::fill(dist.begin(), dist.end(), cost_t::INFTY);
		std::fill(allowed.begin(), allowed.end(), 0);

		dist[source_node] = 0;		
		allowed[source_node] = 0;

		auto reach = [&](nodeid_t v, cost_t d, moveid_t first_move){
			if(d < dist[v]) {
				q.pushOrDecrease(v, d);
				dist[v] = d;
				allowed[v] = first_move;
			} else if(d == dist[v]) {
				allowed[v] |= first_move;
			}
		};

		for(moveid_t i=0; i<g.out_deg(source_node); ++i){
			auto a = g.out(source_node, i);
			reach(a.target, a.weight, 1 << i);
		}

		while(!q.isEmpty()) {
			int x = q.pop();

			for(auto a: g.out(x)) {
				reach(a.target, dist[x] + a.weight, allowed[x]);
			}
		}

		DO_ON_DEBUG {
			for(int u=0; u<g.node_count(); ++u) {
				for(auto uv : g.out(u)) {
					int v = uv.target;
					if(!(dist[u] >= dist[v] - uv.weight)) {
						error("source:", source_node, "u:", u, " v:", v, "dist[u] >= dist[v] - uv.weight (", dist[u], ">=", dist[v], "-", uv.weight);
						throw cpp_utils::exceptions::ImpossibleException{};
					}
				}
			}
		}

		return allowed;
	}

	cost_t distance(nodeid_t target) const {
		return dist[target];
	}
};

}

#endif
