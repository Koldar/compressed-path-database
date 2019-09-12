/**
 * @file
 *
 * Contaisn cut order
 *
 * @note
 * the content of this file were previously in order.h file but it was moved to allow source/header compilation
 *
 *  Created on: Nov 5, 2018
 *      Author: koldar
 */

#ifndef CUT_ORDER_H_
#define CUT_ORDER_H_

#include "order.hpp"
#include <vector>
#include "list_graph.hpp"

namespace cpd {

template<class CutAlgo>
void compute_cut_order(
	int id_begin, const cpd::datastructures::ListGraph& g, const std::vector<int>& g_to_top_level, const CutAlgo& algo, NodeOrdering& order,
	std::vector<int>& lower_deg,
	std::vector<int>& higher_deg
){
	std::vector<bool> is_lower = remove_isolated_nodes_from_cut(g, algo(g));

	int lower_size = 0, upper_size = 0;
	for(auto x:is_lower)
		if(x)
			++lower_size;
		else
			++upper_size;

	if(std::find(is_lower.begin(), is_lower.end(), !is_lower[0]) == is_lower.end()){
		std::vector<int>inner_order(g.node_count());
		for(int i=0; i<g.node_count(); ++i)
			inner_order[i] = g_to_top_level[i];

		std::sort(inner_order.begin(), inner_order.end(),
			[&](int u, int v){
				u = higher_deg[u]-lower_deg[u];
				v = higher_deg[v]-lower_deg[v];
				return u < v;
			}
		);

		for(int i=0; i<g.node_count(); ++i)
			order.map(inner_order[i], id_begin+i);
	}else{
		long long int order_value = 0;
		for(int i=0; i<g.node_count(); ++i){
			if(is_lower[i]){
				order_value += lower_deg[g_to_top_level[i]];
				order_value -= higher_deg[g_to_top_level[i]];
			}else{
				order_value -= lower_deg[g_to_top_level[i]];
				order_value += higher_deg[g_to_top_level[i]];
			}
		}

		if(order_value < 0)
			for(int i=0; i<g.node_count(); ++i)
				is_lower[i] = !is_lower[i];

		std::vector<int>
			lower_to_top_level,
			higher_to_top_level;

		cpd::datastructures::ListGraph
			lower = extract_node_induced_subgraph(g, g_to_top_level, [&](int x){return is_lower[x];}, lower_to_top_level),
			higher = extract_node_induced_subgraph(g, g_to_top_level, [&](int x){return !is_lower[x];}, higher_to_top_level);

		for(auto a:g.arc){
			if(is_lower[a.source] && !is_lower[a.target]){
				++higher_deg[g_to_top_level[a.source]];
				++lower_deg[g_to_top_level[a.target]];
			}else if(!is_lower[a.source] && is_lower[a.target]){
				++higher_deg[g_to_top_level[a.target]];
				++lower_deg[g_to_top_level[a.source]];
			}
		}

		compute_cut_order(id_begin+lower.node_count(), std::move(higher), std::move(higher_to_top_level), algo, order, lower_deg, higher_deg);
		compute_cut_order(id_begin, std::move(lower), std::move(lower_to_top_level), algo, order, lower_deg, higher_deg);
	}
}

template<class CutAlgo>
NodeOrdering compute_cut_order(const cpd::datastructures::ListGraph&g, const CutAlgo&algo){
	NodeOrdering order(g.node_count());
	std::vector<int>higher_deg(g.node_count(), 0), lower_deg(g.node_count(), 0);

	std::vector<int>g_to_top_level(g.node_count());
	for(int i=0; i<g.node_count(); ++i)
		g_to_top_level[i] = i;

	compute_cut_order(0, g, std::move(g_to_top_level), algo, order, lower_deg, higher_deg);
	assert(order.is_complete());
	return std::move(order);
}

}

#endif /* CUT_ORDER_H_ */
