#ifndef PREFERE_ZERO_CUT_H
#define PREFERE_ZERO_CUT_H


#include "list_graph.hpp"
#include <vector>
#include <functional>

namespace cpd {

std::vector<bool> prefer_zero_cut(const cpd::datastructures::ListGraph& g, std::function<std::vector<bool>(const cpd::datastructures::ListGraph& g)>);

template<class F>
struct PreferZeroCutFunc {
	std::vector<bool>operator()(const cpd::datastructures::ListGraph& g) const {
		return prefer_zero_cut(g, default_cutter);
	}

	const F&default_cutter;
};

template<class F>
PreferZeroCutFunc<F> prefer_zero_cut(const F& f) {
	return {f};
}

}

#endif

