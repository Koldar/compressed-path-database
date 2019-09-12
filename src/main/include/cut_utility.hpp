#ifndef CUT_UTILITY_H
#define CUT_UTILITY_H

#include "list_graph.hpp"

namespace cpd {

std::vector<bool>remove_isolated_nodes_from_cut(const ListGraph&g, std::vector<bool>cut);

}

#endif
