#ifndef _COMPRESSED_PATH_DATABASE_ORDER_HEADER__
#define _COMPRESSED_PATH_DATABASE_ORDER_HEADER__

namespace compressed_path_database {

class NodeOrdering;

}

#include "list_graph.hpp"
#include "adj_array.hpp"
#include "cut_utility.hpp"
#include <cpp-utils/operators.hpp>
#include <cassert>
#include <string>
#include <utility>
#include <stdexcept>

namespace compressed_path_database {

/**
 * represents a mapping between 2 index systems
 *
 * For example you have 3 nodes A, B and C.
 *
 * In a system (e.g., alpha) A has index 1, B has index 2 and C has index 3.
 * Suppose that you want to have another system beta where A having the index 3, B index 2 and C index 1.
 * You might want to have a mapping between these 2 systems.
 *
 * The class does exactly this.
 */
class NodeOrdering {
	friend bool operator==(const NodeOrdering& l, const NodeOrdering& r);
private:
	/**
	 * @brief each index is the id of a vertex in the old coordinate system.
	 * 
	 * each cell contains the id of the same vertex but in the new coordinate system
	 * 
	 */
	std::vector<int> to_new_array;
	/**
	 * @brief each index is the id of a vertex in the new coordinate system.
	 * 
	 * each cell contains the id of the same vertex but in the old coordinate system
	 * 
	 */
	std::vector<int> to_old_array;
public:
	NodeOrdering() {

	}
	/**
	 * create a ordering where both new and old array are all filled with -1 and have length @c node_count
	 *
	 * @param[in] node_count the size of vectors to create
	 */
	explicit NodeOrdering(int node_count): to_new_array(node_count, -1), to_old_array(node_count, -1){

	}

	template <typename NUM>
	std::vector<NUM> getToNewArray() const {
		std::vector<NUM> result{};
		for (int i=0; i<this->to_new_array.size(); ++i) {
			result.push_back(to_new_array[i]);
		}
		return result;
	}

	template <typename NUM>
	std::vector<NUM> getToOldArray() const {
		std::vector<NUM> result{};
		for (int i=0; i<this->to_old_array.size(); ++i) {
			result.push_back(to_old_array[i]);
		}
		return result;
	}

	const std::vector<int>& getToNewArray() const {
		return to_new_array;
	}

	const std::vector<int>& getToOldArray() const {
		return to_old_array;
	}

	/**
	 * set the values of both old and new array to -1
	 */
	void clear();

	/**
	 * @return the size
	 */
	int node_count() const;

	/**
	 * retrieve the coordinate of an index in the new system
	 *
	 * @param[in] x the index of the old system to convert
	 * @return the index of @c in the new system
	 */
	int to_new(int x) const;

	/**
	 * retrieve the coordinate of an index in the old system
	 *
	 * @param[in] x the index of the new system to convert
	 * @return the index of @c in the old system
	 */
	int to_old(int x) const;

	/**
	 * Set, if feasable, a mapping between the old index and the new one
	 *
	 * @post
	 *  @li NodeOrdering::to_new_array contains the new index in the old position;
	 *  @li NodeOrdering::to_old_array contains the old index in the new position;
	 *
	 * @param[in] old_id the old index;
	 * @param[in] new_id the new index;
	 */
	void map(int old_id, int new_id);

	/**
	 * check that every index has indeed a value
	 *
	 * @return
	 *  @li true if you can map every node of the old system into the new one;
	 *  @li false if even one node with the index of the old system does not have associated index of the new system;
	 */
	bool is_complete() const;

	/**
	 * sort indexes between the given 2
	 *
	 * @param[in] new_id_begin a starting id (using the new system) where we need to start the sorting
	 * @param[in] new_id_end an ending id (using the new system) where we need to end the sorting
	 */
	void sort_range(int new_id_begin, int new_id_end);
	
	bool next_range_permutation(int new_id_begin, int new_id_end);

	std::vector<int>store_range(int new_id_begin, int new_id_end);

	void load_range(int new_id_begin, const std::vector<int>&v);

	void save(std::FILE*f) const;

	void load(std::FILE*f);



	void check_for_errors();
};

bool operator==(const NodeOrdering&l, const NodeOrdering&r);

inline
bool operator!=(const NodeOrdering&l, const NodeOrdering&r){
	return !(l == r);
}

NodeOrdering compute_real_dfs_order(const compressed_path_database::datastructures::ListGraph &g);

}



#endif

