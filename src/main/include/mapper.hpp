// #ifndef MAPPER_H
// #define MAPPER_H

// namespace compressed_path_database::datastructures {

// class Mapper;

// }

// #include <vector>
// #include <cstdint>
// #include "list_graph.hpp"
// #include "CpdManager.hpp"
// #include "order.hpp"
// #include <cpp-utils/log.hpp>
// #include <functional>

// namespace compressed_path_database::datastructures {

// /**
//  * A mapper is data structure allowing you to create mapping between
//  * the dealing of 2D coordinate points and 1D coordinate vector.
//  *
//  * You can also retrieve the position both via id of vertices and xy_loc.
//  *
//  *
//  *
//  *
//  */
// class Mapper{
// public:
// 	Mapper(){}
// 	Mapper(const std::vector<bool>&field, int width, int height);
// 	// TODO remove Mapper(const GridMap& map);

// 	/**
// 	 * @return the width of the map
// 	 */
// 	int width()const;

// 	/**
// 	 * @return the height of the map
// 	 */
// 	int height()const;

// 	/**
// 	 * @return number of cells which are traversable in the current map
// 	 */
// 	int node_count()const;

// 	/**
// 	 * @param[in] x the index of Mapper::node_to_pos_ where our location is positioned
// 	 * @return the location we need
// 	 */
// 	xyLoc operator()(dpf::nodeid_t x)const;

// 	/**
// 	 * @param[in] p the location whose index we want to have
// 	 * @return
// 	 *  @li the index of location @p (if traversible
// 	 *  @li -1 if the location is not traversible or simply does not exist;
// 	 */
// 	dpf::nodeid_t operator()(const xyLoc& p)const;

// 	/**
// 	 * @brief convert a list of node ids into a list of locations
// 	 * 
// 	 * Usually the list of locations are a path
// 	 * 
// 	 * @param ids list of ids to convert
// 	 * @return std::vector<xyLoc> a list of same size of `ids` containing locations
// 	 */
// 	std::vector<xyLoc> convert(const std::vector<dpf::nodeid_t> ids) const;

// 	/**
// 	 * @brief convert a list of xyLoc into a list of node ids
// 	 * 
// 	 * Usually the list of locations are a path
// 	 * 
// 	 * @param ids list of xyLoc to convert
// 	 * @return std::vector<dpf::nodeid_t> a list of same size of `ids` containing ids
// 	 */
// 	std::vector<dpf::nodeid_t> convert(const std::vector<xyLoc> ids) const;

// 	/**
// 	 * reorder the underlying vector of locations with a new ordering
// 	 *
// 	 * @post
// 	 *  @li pos_to_node_ has now a new ordering of locations, which is the one expressed by the "new" system @c order
// 	 *
// 	 * @param[in] order the new ordering of the locations
// 	 */
// 	void reorder(const NodeOrdering& order);
// private:
// 	/**
// 	 * @brief width of the gridmap
// 	 * 
// 	 */
// 	int width_;
// 	/**
// 	 * @brief height of the gridmap
// 	 * 
// 	 */
// 	int height_;
// 	/**
// 	 * @brief number of traversable nodes
// 	 * 
// 	 */
// 	int node_count_;
// 	/**
// 	 * @brief convert indices into grid map locations
// 	 * 
// 	 * if the i-th cell contains a non negative value, it represents the index in ::node_to_pos
// 	 * where the actual coordinate is saved.
// 	 * if the i-th cell contains a -1, it means that the cell is untraversable
// 	 * 
// 	 */
// 	std::vector<dpf::nodeid_t>pos_to_node_;
// 	/**
// 	 * given a points, contains the index of Mapper::pos_to_node_ where such location is positioned
// 	 */
// 	std::vector<xyLoc>node_to_pos_;
// };

// /**
//  * generates a graph representing the map we're handling.
//  *
//  * In this map, each traversible cell is a vertex. if 2 vertex are connected by an edge,
//  * it means we can reach one cell from the other one and viceversa. arcs are labeled wioth their weight
//  * (either 1000 or 1414)
//  *
//  * |----------------|
//  * |-1/-1|0/-1|+1/-1|
//  * |-1/0 |n.a.|+1/0 |
//  * |-1/+1|0/+1|+1/+1|
//  * |----------------|
//  *
//  * @param[in] mapper the map we want to convert
//  * @return the graph
//  *
//  */
// inline
// ListGraph extract_graph(const Mapper&mapper){

// 	std::int16_t dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
// 	std::int16_t dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
// 	//distances multipled by 1000. so from cell (0,0) and (-1,0) the distance is 1x1000 = 1000
// 	//for cell (0,0) and (1,1) the distance is sqrt(2)x1000 about = 1414
// 	int dw[] = {1414, 1000, 1414, 1000, 1000, 1414, 1000, 1414}; 
// 	ListGraph g(mapper.node_count());
// 	for(int u=0; u<mapper.node_count(); ++u){
// 		auto u_pos = mapper(u);
// 		for(int d = 0; d<8; ++d){
// 			xyLoc v_pos = {(u_pos.x + dx[d]), (u_pos.y + dy[d])};
// 			int v = mapper(v_pos);
// 			if(v != -1 && mapper(xyLoc{u_pos.x, (u_pos.y+dy[d])}) != -1 &&  mapper(xyLoc{(u_pos.x+dx[d]), u_pos.y}) != -1)
// 				g.arc.push_back({u, v, dw[d]});
// 		}
// 	}
// 	return g;
// }

// /**
//  * generate a list graph from the given map
//  *
//  * @param[in] map the map we need to convert to listgraph
//  * @param[in] mapper a structure allowing us to go from nodeid_t to xyLoc
//  * @param[in] costStrategy a function that tell us what is the cost associated to the arc starting from  the third parameter to the fourht parameter
//  * @return the ListGraph instance requested
//  */
// inline ListGraph getListGraphFrom(const GridMap& map, const Mapper& mapper, const std::function<int(const GridMap&, const Mapper&, xyLoc, xyLoc)>& costStrategy) {
// 	int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
// 	int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};

// 	ListGraph g(mapper.node_count());
// 	for(int u=0; u<mapper.node_count(); ++u){
// 		auto u_pos = mapper(u);
// 		for(int d = 0; d<8; ++d){
// 			xyLoc v_pos = {(u_pos.x + dx[d]), (u_pos.y + dy[d])};
// 			int v = mapper(v_pos);

// 			if((v != -1) && (mapper(xyLoc{u_pos.x, (u_pos.y+dy[d])}) != -1) &&  (mapper(xyLoc{(u_pos.x+dx[d]), u_pos.y}) != -1))
// 				g.arc.push_back(Arc{u, v, costStrategy(map, mapper, u_pos, v_pos)});
// 		}
// 	}
// 	return g;
// }

// /**
//  * save the map in a file
//  *
//  * @param[in] map the map to save
//  * @param[in] file the name of the file where we want to save the map
//  */
// void dump_map(const Mapper&map, const char*file);

// }

// #endif

