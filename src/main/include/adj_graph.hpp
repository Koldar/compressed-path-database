/**
 * @file
 *
 * Represents an Adjacency Graph which is tuned for square cell in path planning
 *
 * @attention
 * This is not a general module for handling adjacency graph!
 *
 * @author: unknown
 * @date: unknown
 */

#ifndef ADJ_GRAPH_H
#define ADJ_GRAPH_H

namespace compressed_path_database::datastructures {

class AdjGraph;

}

#include <vector>
#include <functional>
#include "list_graph.hpp"
#include "range.hpp"
#include "adj_array.hpp"
#include "types.hpp"
//TODO remove #include "xyLoc.h"
//#include "gridmap_path.h"
//#include "pbmImage.h"
//#include "gridmap.h"
#include <vector>
#include <unordered_set>

namespace compressed_path_database::datastructures {

using namespace cpp_utils::graphs;

/**
 * represents an arc which goes out froma vertex
 *
 * @note
 * The structure does**not** represents the source of the arc itself
 */
struct OutArc {
	///the sink node the arc is going to
	int target;
	///the label of the arc
	int weight;
	/**
	 * true if the edge weight has been changed
	 */
	bool perturbated;

	/**
	 * @return
	 *  @li true if the arc has been perturbated;
	 *  @li false otherwise
	 */
	bool hasBeenPerturbated() const;

};

class AdjGraph;

bool operator ==(const OutArc& thiz, const OutArc& other);

bool operator !=(const OutArc& thiz, const OutArc& other);

std::ostream& operator <<(std::ostream& stream, const OutArc& a);

std::ostream& operator<<(std::ostream& stream, const AdjGraph& a);

/**
 * A graph where each information is encoded via a adjacency vector
 */
class AdjGraph {
	friend std::ostream& operator<<(std::ostream& stream, const AdjGraph& a);
public:
	/**
	 * creates an empty graph
	 */
	AdjGraph();

	/**
	 * copy constructor of a graph
	 *
	 * @param[in] g the graph to clone
	 */
	AdjGraph(const AdjGraph& g);

	/**
	 * Create a graph storing the edges as successors of vertices
	 *
	 * @param[in] g the graph to convert
	 */
	AdjGraph(ListGraph g);

	/**
	 * Overload of assignment of ListGraph into a AdjGraph
	 *
	 * @param[in] o the graph where we need to fetch information from our AdjGraph
	 */
	AdjGraph& operator=(const ListGraph& o);

	AdjGraph& operator=(const AdjGraph& other);

	/**
	 * Compute the number of vertices in this graph
	 *
	 * @return number of vertices in the graph
	 */
	unsigned int node_count() const;

	/**
	 * @brief compute the number of vertices in the graph
	 * 
	 * @return number of vertices
	 */
	unsigned int size() const;

	Range<std::vector<OutArc>::const_iterator> out(int v) const;

	/**
	 * @param[in] v the vertex where arcs are going out
	 * @return iterator from the first outgoing arc from vertex @c
	 */
	std::vector<OutArc>::const_iterator arcBegin(int v) const;

	/**
	 * @param[in] v the vertex where arcs are going out
	 * @return iterator or the last outgoing arc from vertex @c
	 */
	std::vector<OutArc>::const_iterator arcEnd(int v) const;

	/**
	 * get the i-th out arc from vertex v
	 *
	 * @pre
	 *  @li \f$ i < g.out\_deg(v); \f$
	 *
	 * @param[in] v the vertex involved
	 * @param[in] i the i-th arc of vertex @c v
	 * @return a copy of the arc specifying the i-th arc coming out from vertex @c v
	 */
	OutArc out(int v, int i) const;

	/**
	 * @param[in] v the vertex involved
	 * @return number of arcs coming out from @c v
	 */
	int out_deg(int v) const;

	/**
	 * @brief check if the vertex has sink
	 * 
	 * This operation takes \f$ O(1) \f$
	 * 
	 * @param v the vertex to check
	 * @return true if the vertex has no outgoing edges
	 * @return false otherwise
	 */
	bool hasVertexNoSinks(int v) const;

	/**
	 * Change the weight of the arc going from @c v to @c w
	 *
	 * @post
	 *  @li both arc v-w and w-v weill be changed to the new weight;
	 *
	 * @attention
	 *  it is not performance optimized
	 *
	 * @param[in] v the source of the arc to alter
	 * @param[in] w the sink of the arc to alter
	 * @param[in] newWeight the new wight the arc between v and w has
	 */
	void changeWeightOfArc(int v, int w, int newWeight);

	/**
	 * Change the weight of a single ar. It does no automatically update the opposite arc
	 *
	 * @attention
	 * This function should be consider unsafe since it can lead to a Adjgraph with weird semantic!
	 *
	 * The function alters, given a source and a sink, only a specific arc. For, for example calling:
	 * @code
	 *  changeWeightOfDirectedArc(A, B, 100);
	 * @endcode
	 *
	 * will alter only the edge \f$ A \rightarrow B\f$, but not the other one
	 *
	 * @dot
	 * digraph {
	 * 	A;
	 * 	B;
	 * 	A -> B [label="10 (after goes to 100)"];
	 * 	B -> A [label="10"];
	 * }
	 * @enddot
	 *
	 * @attention
	 *  it is not performance optimized
	 *
	 * @param[in] v the source where rthe arc starts from
	 * @param[in] w the sink where the arc ends to
	 * @param[in] newweight the new weight to associate to the arc
	 */
	void changeWeightOfDirectedArc(int v, int w, int newWeight);

	/**
	 * @attention
	 * This operation is O(b) where "b" is the out-degree of the graph
	 *
	 * @param[in] v the source vertex
	 * @param[in] w the sink vertex
	 * @return the weight fo the arc between @c v and @c w
	 */
	int getWeightOfArc(int v, int w) const;

	/**
	 * like ::getIthOutArc but returns the weight fo the arc, not the arc itself
	 *
	 * @pre
	 *  @li \f$ i < g.out\_deg(v); \f$
	 *
	 * @param[in] v the vertex involved
	 * @param[in] i the i-th arc of vertex @c v
	 */
	int getWeightOfIthArc(int v, int i) const;

	/**
	 * @pre
	 *  @li \f$ i < g.out\_deg(v); \f$
	 *
	 * @param[in] v the vertex involved
	 * @param[in] i the i-th arc of vertex @c v
	 * @return the **reference** of the actual OutArc
	 */
	const OutArc& getIthOutArc(int v, int i) const;

	/**
	 *  get the arc between v and w
	 *
	 *  @attention
	 *  it is not performance optimized
	 *
	 *  @param[in] v the first vertex
	 *  @param[in] w the second vertex
	 *  @return the reference of the actual OutArc in the structure
	 */
	const OutArc& getArc(int v, int w) const;

	/**
	 * @attention
	 *  it is not performance optimized
	 *
	 * @return true if the arcs is present in the graph
	 *  @li false otherwise
	 */
	bool containsArc(nodeid_t source, nodeid_t sink) const;

	/**
	 * check if the arc has been revised
	 *
	 * Arcs normally have wieght etierh of 1000 (horizontal/vertical weights) or 1414 (diagonal ones).
	 * If an arc doesn't have it, thea rc has been altered
	 *
	 * @attention
	 * it is not performance optimzed
	 *
	 * @param[in] v source of the arc
	 * @param[in] w sink of the arc
	 * @return
	 *  @li true if the arc has been revised;
	 *  @li false otherwise
	 */
	bool hasArcBeenPerturbated(int v, int w) const;

	/**
	 * Check if the graph has been perturbated
	 *
	 * @attention
	 * it is not performance optimzed
	 *
	 * @return
	 *  @li true if at elast one arc in the map has been perturbated;
	 *  @li false otherwise;
	 * @see hasArcBeenPerturbated
	 */
	bool hasBeenPerturbated() const;

	/**
	 *
	 *
	 * @return the total number of arcs
	 */
	std::size_t getTotalNumberOfArcs() const;

	/**
	 * Compute the cost of a path
	 *
	 * @attention
	 *  it is not performance optimized
	 *
	 * @param[in] path the path considered
	 * @param[in] mapper the mapper used to convert locations into node ids
	 * @return the cost of the path @c path
	 */
	//TODO remove cost_t getCostOfPath(const std::vector<xyLoc>& path, const Mapper& mapper) const;

	/**
	 * Get the arcs composing the path
	 *
	 * For example, if the path is <tt>A,B,C,D</tt> the function will generate <tt>A-B, B-C, C-D</tt>
	 *
	 * @param[in] path the path on the given map
	 * @param[in] mapper the mapper used to convert node id into locations
	 * @return an ordered list of arc rpresenting the path
	 */
	//TODO remove std::vector<Arc> getArcsOverMap(const gridmap_path& path, const Mapper& mapper) const;

	/**
	 * Get the number of arc in the given path that satisfy the given condition
	 *
	 * @param[in] mapper structure used to convert the node ids into locations
	 * @param[in] path the path considered
	 * @param[in] filter the constraint that we want to check. Within it:
	 * 	@li the first parameter is the index of the arc considered;
	 * 	@li the second parameter is the  source where the arc coming from;
	 * 	@li the third parameter is the arc itself
	 * @return the number of arcs in @c path satisfiying @c filter
	 */
	//TODO remove std::size_t getNumberOfArcsInPathSuchThat(const Mapper& mapper, const std::vector<xyLoc>& path, std::function<bool(dpf::big_integer i, dpf::nodeid_t, OutArc a)> filter) const;

	/**
	 *
	 * For exampel if the graph has 10 arcs nad we pass the ratio 0.2, we will return 2 (since the 20% of 10 is indeed 2)
	 *
	 * @param[in] ratio the ratio involved
	 * @return the number of arc represented by the ratio
	 */
	//std::size_t getNumberOfArcsFromRatio(double ratio) const;

	/**
	 * Generate a SVG image representing the adjacence graph.
	 *
	 * @post
	 *  @li this iwll create an image in the CWD
	 *
	 * @param[in] mapper the mapper used to swap between int coordinates and locations
	 * @param[in] baseName the name of the file to create (extension excluded)
	 * @param[in] shouldPrintNode a lambda that tells if a specific node should be printed. if nullptr we will print every node
	 * @param[in] shouldPrintArc a lambda that tells if a specific arc should be printed. If nullptr we will print every arc
	 * @param[in] colorArca lamda returning a string representing the color of an arc. If nullptr everything weill be printed with black
	 */
	//TODO remove void print(const Mapper& mapper, const string& baseName, std::function<bool(dpf::nodeid_t)>* shouldPrintNode=nullptr, std::function<bool(dpf::nodeid_t, OutArc)>* shouldPrintArc=nullptr, std::function<const string(dpf::nodeid_t, OutArc)>* colorArc=nullptr) const;

	/**
	 * like ::AdjGraph::print but prints on the map a list of paths
	 *
	 * @param[in] mapper the mapper used to convert nodeid_t into locations
	 * @param[in] baseName name of the image to generate (no extension)
	 * @param[in] paths a vector fo paths to print out
	 */
	//TODO remove void printPaths(const Mapper& mapper, const string& baseName, const std::vector<gridmap_path>& paths) const;

	//TODO remove PPMImage getImageWith(const Mapper& mapper, const GridMap& map, const dpf::nodeid_t start, const dpf::nodeid_t goal, const std::vector<dpf::nodeid_t> expandedList, color_t expandedColor, color_t startColor=GREEN, color_t goalColor=BLUE, color_t perturbatedColor=RED, color_t backgroundColor=WHITE) const;

	/**
	 * @return all the weights such that at least one arc in the graph has
	 */
	//TODO remove std::unordered_set<int> getWeights() const;

	//TODO remove const std::vector<OutArc>& getArcs() const;

	/**
	 * Save the current graph into a file
	 *
	 * @pre
	 *  @li @c f open with "wb";
	 * @post
	 *  @li @c f modified;
	 *  @li @c f cursor modified;
	 *
	 *
	 * @param[in] f the file to save the graph into
	 */
	//void save(FILE* f) const;

	/**
	 * Load a graph from a file in the filesystem
	 *
	 * @pre
	 *  @li @c f open in "rb";
	 * @post
	 *  @li @c f cursor modified;
	 *
	 * @param[in] f the file to read the graph from;
	 * @return the graph loaded
	 */
	//static AdjGraph load(FILE* f);

private:
	/**
	 * @pre
	 *  @li \f$ i < g.out\_deg(v); \f$
	 *
	 * @param[in] v the vertex involved
	 * @param[in] i the i-th arc of vertex @c v
	 * @return the **reference** of the actual OutArc
	 */
	OutArc& getIthOutArc(int v, int i);



	/**
	 *  get the arc between v and w
	 *
	 *  @param[in] v the first vertex
	 *  @param[in] w the second vertex
	 *  @return the reference of the actual OutArc in the structure
	 */
	OutArc& getArc(int v, int w);


private:
	friend bool operator==(const AdjGraph& thiz, const AdjGraph& other);
	/**
	 * a vector where the index is the id of a node and the value is the index of AdjGraph::out_arc where the
	 * arcs going out from the vertex start occuring.
	 *
	 * Its length is the number of vertices in the graph
	 */
	std::vector<int>out_begin;
	/**
	 * a list of arcs. Outgoing arcs from a given nodes are contiguous in indexing
	 *
	 * Its length is the number of arcs in the graph
	 */
	std::vector<OutArc>out_arc;
};

/**
 * check if 2 AdjGraph are the same
 *
 * 2 graphs are the same if the have the same vertices, arcs and arcs weight.
 *
 * @param[in] other the other AdjGraph
 * @return
 *  @li true if the are equal;
 *  @li false otherwise
 */
bool operator==(const AdjGraph& thiz, const AdjGraph& other);

bool operator!=(const AdjGraph& thiz, const AdjGraph& other);


/**
 * Macro for iterating over graph edges
 *
 * @attention
 * Thism acro cannot be nested in other same macro calls!
 *
 * @code
 * 	AdjGraph g{};
 * 	ITERATE_OVER_EDGES(g, sourceId, i, arc) {
 *		info("the source", sourceId, "has an outgoing arc whose sink is", arc.target, "and weight is", arc.weight);
 * 	}
 * @endcode
 *
 * @param[in] adjGraph a Adjgraph instance to loop over
 * @param[in] an unused variable identifier used to represents a graph vertex
 * @param[in] ithOutArc an unused variable identifier used to represents the id of the i-th arc @c outArc
 * @param[in] outArc the @ithOutArc out going arc of @c sourceid

 */
#define ITERATE_OVER_EDGES(adjGraph, sourceId, ithOutArc, outArc) \
	for (dpf::nodeid_t sourceId = 0; sourceId < ((const AdjGraph)adjGraph).node_count(); ++sourceId) \
		for (int ithOutArc=0; ithOutArc<((const AdjGraph)(adjGraph)).out_deg(static_cast<int>(sourceId)); ++ithOutArc) \
			for (bool it=true; it; ) \
				for (OutArc outArc=((const AdjGraph)(adjGraph)).getIthOutArc(static_cast<int>(sourceId), ithOutArc); it; it=false)

#endif


}