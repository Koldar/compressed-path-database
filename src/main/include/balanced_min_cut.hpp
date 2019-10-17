#ifndef BALANCED_MIN_CUT_H
#define BALANCED_MIN_CUT_H

#include "list_graph.hpp"
#include <vector>

namespace compressed_path_database {

std::vector<bool> balanced_min_cut(const compressed_path_database::datastructures::ListGraph& g);

}

#endif
