#include "prefer_zero_cut.hpp"
#include "adj_array.hpp"
#include <stack>
#include <utility>

namespace compressed_path_database {

// compile with -O3 -DNDEBUG

std::vector<bool> prefer_zero_cut(const datastructures::ListGraph& g, std::function<std::vector<bool>(const datastructures::ListGraph& g)>default_cutter) {
	std::vector<int>out_begin, out_dest;

	build_adj_array(
		out_begin, out_dest, 
		g.node_count(), g.arc.size(),
		[&](int x){return g.arc[x].source;},
		[&](int x){return g.arc[x].target;}
	);

	std::vector<bool>was_visited(g.node_count(), false);

	std::stack<int>to_visit;
	to_visit.push(0);
	was_visited[0] = true;

	while(!to_visit.empty()){
		int x = to_visit.top();
		to_visit.pop();
		for(int i = out_begin[x]; i<out_begin[x+1]; ++i)
			if(!was_visited[out_dest[i]]){
				was_visited[out_dest[i]] = true;
				to_visit.push(out_dest[i]);
			}
	}

	#ifndef NDEBUG
	for(auto a:g.arc)
		assert(!was_visited[a.source] || was_visited[a.target]);
	#endif

	for(auto x:was_visited)
		if(!x)
			return std::move(was_visited);
	return default_cutter(g);
}

}