#ifndef CPD_H
#define CPD_H

#include <vector>
#include <algorithm>
#include <string>
//#include "adj_graph.h"
#include "binary_search.hpp"
#include "range.hpp"
#include <cpp-utils/serializers.hpp>
#include <cpp-utils/imemory.hpp>
//#include "vec_io.h"

namespace compressed_path_database {

	//! Compressed Path database. Allows to quickly query the first out arc id of
	//! any shortest source-target-path. There may be at most 15 outgoing arcs for
	//! any node.
	class CPD: public cpp_utils::IMemorable {
	private:

		/**
		 * Tells the indexes where it starts a new optimal direction set for a particular source_node
		 *
		 * for exampel if begin[5] == 12 it means that the fifth source_node optimal directions
		 * starts in cpd::entry in index 12.
		 *
		 * The index of the vector is the index of the source_node to consider
		 */
		std::vector<int> begin;
		/**
		 * value containing the optimal direction you have to take to reach a goal from a particular source node.
		 *
		 * each integer follows this scheme xxxxxxxxxxyyyy:
		 * @li x is the vertex id where we start this common optimal direction has started;
		 * @li y the optimal common direction (number between 1 and 15) of the vertices between xxxx and the next element of cpd::entry;
		 *
		 * a given start location may have several integer representing the optimal directions for every target. Such integer represneting
		 * optimal direction for a given source_node are contiguous in this vector
		 */
		std::vector<int> entry;
	public:
		CPD(): begin{0} {

		}

		CPD(const CPD& other) = delete;
		CPD(CPD&& other): begin{std::move(other.begin)}, entry{std::move(other.entry)} {

		}

		CPD& operator =(const CPD& other) = delete;
		CPD& operator =(CPD&& other) {
			this->begin = std::move(other.begin);
			this->entry = std::move(other.entry);
			return *this;
		}
		virtual ~CPD() {
			debug("destroying CPD at ", this);
		}

		//! Adds a new node s to the CPD. first_move should be an array that 
		//! maps every target node onto a 15-bit bitfield that has a bit set
		//! for every valid first move. get_first_move is free to return any of
		//! them.
		void append_row(int source_node, const std::vector<unsigned short>&first_move);

		void append_rows(const CPD&other);

		//! Get the first move. 
		//! An ID of 0xF means that there is no path. 
		//! If source_node == target_node then return value is undefined. 
		unsigned char get_first_move(int source_node, int target_node)const{
			target_node <<= 4;
			target_node |= 0xF;
			return *binary_find_last_true(
				entry.begin() + begin[source_node],
				entry.begin() + begin[source_node+1],
				[=](int x){return x <= target_node;}
			) & 0xF;
		}

		int node_count()const{
			return begin.size()-1;
		}

		int entry_count()const{
			return entry.size();
		}

		friend bool operator==(const CPD&l, const CPD&r){
			return l.begin == r.begin && l.entry == r.entry;
		}

		void save(std::FILE* f) const {
			cpp_utils::serializers::saveToFile<int>(f, begin);
			cpp_utils::serializers::saveToFile<int>(f, entry);
		}

		void load(std::FILE* f) {
			cpp_utils::serializers::loadFromFile<int>(f, begin);
			cpp_utils::serializers::loadFromFile<int>(f, entry);
		}
	public:
		virtual cpp_utils::MemoryConsumption getByteMemoryOccupied() const {
			return 0
				+ sizeof(int)*this->begin.size() 
				+ sizeof(int)*this->entry.size() 
				+ sizeof(*this);
		}

	};

	inline bool operator !=(const CPD&l, const CPD&r){
		return !(l == r);
	}

}

#endif
